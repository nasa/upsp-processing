//
// Asynchronously read and/or write chunks of file(s).
// These routines present a reasonably easy to use interface,
// built on top of the standard "aio" package.
//
// There are aspects of the implementation that would be better written
// in C++.  We deliberately write it in C for ease of use (e.g. linking
// the code into a Fortran program).  We use two non-ansi-C constructs:
//   1) use of // as a comment
//   2) inline variable declarations
//

// Need to define this for pread/pwrite
#define  _XOPEN_SOURCE  500

// Need to define this for O_DIRECT
#ifndef  _GNU_SOURCE
#define  _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <aio.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

// For the BLKSSZGET ioctl (oddly, not in sys/ioctl.h)
#include <linux/fs.h>

#include "utils/asyncIO.h"


#define DIE_IF(expr) { if (expr) {perror( # expr ) ; exit(1);} }

// This assumes "__align" is a power-of-2
#define isAligned(__val, __align)  (0 == ((__val) & ((__align) - 1)))


typedef struct qEntry {
    asyncIO_ioBufferHeaderType bufHeader; // MUST be first item in struct
    ptrdiff_t checkWord;
    struct qEntry *tailward;
    struct qEntry *headward;
    int aiocb_numAllocated;
    int aiocb_numInUse;
    struct aiocb *aio;
} qEntryType;


typedef struct {
    qEntryType *head;
    qEntryType *tail;
} qHeader;


typedef struct {
    int checkWord;  // MUST be first item in struct
    int fdStandard;
    int fdDirect;
    int alignment;

    int numChunks;
    int nextChunk;
    long int *fileOffsets;
    long int *chunkSizes;
    long int *tags;

    qHeader pending;
    qHeader available;
} inQueueHeaderType;


typedef struct {
    int checkWord;  // MUST be first item in struct
    int fdStandard;
    int fdDirect;
    int alignment;

    long int nextFileOffset;
    long int initialBufferSizeInBytes;

    qHeader pending;
    qHeader available;
} outQueueHeaderType;


#define inQCheckWordValue   12345
#define outQCheckWordValue  67890

long int __asyncIO_align = -1;
long int __asyncIO_maxXactSizeInBytes = 256 * 1024 * 1024;



/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
// Utility routines

// Add the item to the tail of the queue
void
__asyncIO_enqueue (qHeader *q,  qEntryType *item)
{
    item->tailward = NULL;
    item->headward = q->tail;

    if (NULL == q->tail) {
        q->head = item;
    } else {
        q->tail->tailward = item;
    }
    q->tail = item;
}


// Remove and return the item at the head of the queue
qEntryType *
__asyncIO_dequeue (qHeader *q)
{
    qEntryType *rtnValue = q->head;

    if (rtnValue != NULL) {
        q->head = q->head->tailward;
        if (NULL == q->head) {
            q->tail = NULL;
        } else {
            q->head->headward = NULL;
        }
        rtnValue->tailward = rtnValue->headward = NULL;
    }

    return rtnValue;
}



// The aio did not successfully complete.  Retry the operation in the
// foreground using pread/pwrite in an attempt to recover.  If the
// operation fails again, give up.
ssize_t
__asyncIO_retryAioInForeground (struct aiocb *aio)
{
    unsigned char *buf = (unsigned char *) aio->aio_buf;
    size_t count = aio->aio_nbytes;
    off_t offset = aio->aio_offset;

    assert ( (aio->aio_lio_opcode == LIO_READ) || (aio->aio_lio_opcode == LIO_WRITE) );

    ssize_t amtTransferred = 0,  thisTransfer;
    char r[] = "read",  w[] = "write";
    char *name = (LIO_WRITE == aio->aio_lio_opcode) ? w : r;

    // The annoying cast of pread is needed because pread takes
    // a "void *", but pwrite takes a "const void *"
    ssize_t (*func)(int, const void *, size_t, off_t) =
                 (LIO_WRITE == aio->aio_lio_opcode) ? pwrite :
                 ((ssize_t(*)(int, const void *, size_t, off_t))  pread);

    int aioErrno = 0;

    while (amtTransferred < count) {
        errno = 0;
        thisTransfer = func (aio->aio_fildes, buf + amtTransferred,
                          count - amtTransferred, offset + amtTransferred);
        aioErrno = errno;

        // If it failed, or the transfer was zero length, abort.
        if (thisTransfer < 0) {
            char msg[1000];
            sprintf (msg, "ERROR: attempting to recover from aio %s fail", name);
            perror (msg);
            errno = aioErrno;
            return -1;
        }
        else if (0 == thisTransfer) {
            char msg[1000];
            sprintf (msg, "ERROR: no progress recovering from aio %s fail", name);
            perror (msg);
            errno = aioErrno;
            return amtTransferred; 
        }

        // Forward progress is being made, so keep trying
        amtTransferred += thisTransfer;
    }

    errno = 0;
    assert (amtTransferred == aio->aio_nbytes);
    return amtTransferred; 
}



// Wait for the given aio to complete, and verify it completed correctly
void
__asyncIO_completeOneAIO (struct aiocb *aio)
{
    long int  rtnVal;

    // Check the current status of the aio
    int aioErrno = aio_error (aio);

    // If it is still in progress, wait for it to complete
    if (EINPROGRESS == aioErrno) {
        aio_suspend ((const struct aiocb * const*)&aio, 1, NULL);
        assert ((aioErrno = aio_error (aio)) != EINPROGRESS);
    }

    // The aio is no longer in progress, so get the final return status
    rtnVal = aio_return (aio);


    // If the operation was partially sucessful (but not completely
    // successful), retry silently.
    if ((rtnVal >= 0) && (rtnVal < aio->aio_nbytes)) {
        errno = 0;
        rtnVal = __asyncIO_retryAioInForeground (aio);
        aioErrno = errno;
    }

    // If the operation failed for a reason that might possibly be
    // transient and recoverable, print some messages, and retry.
    else if ( (rtnVal < 0) &&
         ((EINTR == aioErrno) || (EAGAIN == aioErrno) || (EIO == aioErrno)) )
    {
        errno = aioErrno;  // So we can use perror
        perror ("WARNING: aio request failed");
        fprintf (stderr, "Retry %s request ...\n", 
                  (LIO_WRITE == aio->aio_lio_opcode) ? "write" : "read");

        errno = 0;
        rtnVal = __asyncIO_retryAioInForeground (aio);
        aioErrno = errno;

        if (rtnVal == aio->aio_nbytes) {
            fprintf (stderr, "Retry succeeded\n");
        }
    }


    // At this point, the aio is as complete as we were able to do.  If
    // it was not completely successful, abort.  If it succeeded, return.


    // Die if the i/o failed (don't attempt any additional retries)
    if (rtnVal < 0) {
        errno = aioErrno;
        perror ("ERROR: aio request failed");
        abort();
    }

    // Die if the i/o was not completely successful (the retry routine has
    // already attempted to continue in the face of partial results; the
    // only real reason for this to happen would be an unexpected EOF).
    if (rtnVal != aio->aio_nbytes) {
        fprintf (stderr, "ERROR: short aio %s op: %ld vs. %ld\n",
                  (LIO_WRITE == aio->aio_lio_opcode) ? "write" : "read",
                  rtnVal, aio->aio_nbytes);
        abort();
    }

    // I guess it must have worked
}


long int
__asyncIO_completeAIO (qEntryType *qe)
{
    int ii;
    long int sz = 0;

    for (ii = 0;  ii < qe->aiocb_numInUse;  ++ii) {
        __asyncIO_completeOneAIO (&(qe->aio[ii]));
        sz += qe->aio[ii].aio_nbytes;
    }
    qe->aiocb_numInUse = 0;

    return sz;
}



void *
__asyncIO_allocateBuffer (long int sz, long int alignment)
{
    void *p = NULL;
    if (sz > 0) {
        errno = posix_memalign (&(p), alignment, sz);
        if (errno != 0) {
            perror ("ERROR: posix_memalign");
            abort();
        }
    }
    return p;
}


qEntryType *
__asyncIO_allocateBufHeader (void)
{
    qEntryType *qe = calloc (1, sizeof(qEntryType));
    qe->checkWord = (ptrdiff_t) &(qe->checkWord);
    qe->bufHeader.buffer = NULL;
    qe->tailward = NULL;
    qe->headward = NULL;
    qe->aio = NULL;
    qe->aiocb_numAllocated = 0;
    qe->aiocb_numInUse = 0;
    return qe;
}



/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
// initInput  routines

// asyncIO_initInputQ_fn_full is the "root" initInput routine.  The other
// versions just munge their arguments as appropriate, and call this.
void *
asyncIO_initInputQ_fn_full (
  const char *fn,
  int count,
  long int *fileOffsets,
  long int *chunkSizes,
  long int *tags,
  int numBuffers,
  long int minBufferSizeInBytes,
  int flags)
{
    int ii;
    struct stat statBuf;

    // Adjust the inputs
    if (count < 0) count = 0;
    if (numBuffers < 0) numBuffers = 0;
    if (minBufferSizeInBytes < 0) minBufferSizeInBytes = 0;

    //////////////////////////////////////////////////////////////
    // Verify the input args.  Use extra-long name aliases to
    // improve the clarity of any 'assert' output
    const char *asyncIO_initInputQ_filename = fn;
    int asyncIO_initInputQ_count = count;
    long int *asyncIO_initInputQ_fileOffsets = fileOffsets;
    long int *asyncIO_initInputQ_chunkSizes = chunkSizes;
    long int *asyncIO_initInputQ_tags = tags;
    int asyncIO_initInputQ_numBuffers = numBuffers;
    long int asyncIO_initInputQ_numBufferSize = minBufferSizeInBytes;

    if (stat (asyncIO_initInputQ_filename, &statBuf) < 0) {
        perror ("ERROR trying to stat input file:");
        fprintf (stderr, "File: '%s'\n", asyncIO_initInputQ_filename);
        exit (1);
    }
    assert (asyncIO_initInputQ_count > 0);
    assert (asyncIO_initInputQ_fileOffsets != NULL);
    assert (asyncIO_initInputQ_chunkSizes != NULL);

    for (ii = 0;  ii < count;  ++ii) {
        assert (asyncIO_initInputQ_fileOffsets[ii] >= 0);
        assert (asyncIO_initInputQ_fileOffsets[ii] <= statBuf.st_size);
        assert (asyncIO_initInputQ_chunkSizes[ii] >= 0);
        assert (asyncIO_initInputQ_fileOffsets[ii] + asyncIO_initInputQ_chunkSizes[ii] <= statBuf.st_size);
    }



    //////////////////////////////////////////////////////////////
    // Allocate and init the inQueue header
    inQueueHeaderType *qq = calloc (1, sizeof(inQueueHeaderType));
    qq->checkWord = inQCheckWordValue;
    qq->fdStandard = qq->fdDirect = qq->numChunks = qq->nextChunk = -1;
    qq->fileOffsets = qq->chunkSizes = qq->tags = NULL;
    qq->pending.head = qq->pending.tail = NULL;
    qq->available.head = qq->available.tail = NULL;

    // Open the file.  Note that if O_DIRECT is not enabled, then
    // fdDirect is set to the same value as fdStandard
    int openFlags = O_RDONLY;
    DIE_IF ((qq->fdDirect = qq->fdStandard = open (fn, openFlags)) < 0);
    if ((flags & asyncIO_USE_O_DIRECT) || (flags & O_DIRECT)) {
        DIE_IF ((qq->fdDirect = open(fn, openFlags|O_DIRECT)) < 0);
    }


    //////////////////////////////////////////////////////////////
    // Copy the args to the inQ header
    qq->fileOffsets = malloc (count * sizeof(long int));
    qq->chunkSizes = malloc (count * sizeof(long int));
    qq->tags = malloc (count * sizeof(long int));

    for (ii = 0;  ii < count;  ++ii) {
        qq->fileOffsets[ii] = fileOffsets[ii];
        qq->chunkSizes[ii] = chunkSizes[ii];
        qq->tags[ii] = (tags != NULL) ? tags[ii] : ii;
    }
    qq->numChunks = count;
    qq->nextChunk = 0;


    ///////////////////////////////////////////////////////////////////
    // Check the alignment setting.
    // Currently (May2017), Lustre file systems return the error
    // "inappropriate ioctl" for the BLKSSZGET ioctl, despite the fact
    // that they do support O_DIRECT.  After some debate, the decision
    // was that if the user knows what they are doing, and they want to
    // use O_DIRECT even if the file system doesn't support BLKSSZGET,
    // they have to set __asyncIO_align.  If __asyncIO_align is not set,
    // and the BLKSSZGET ioctl fails, then O_DIRECT is disabled.
    if (__asyncIO_align > 0) {
        qq->alignment = __asyncIO_align;
    } else {
        if (ioctl (qq->fdStandard, BLKSSZGET, &(qq->alignment)) < 0) {
            qq->alignment = statBuf.st_blksize;  // Fill in a value here ..
            qq->fdDirect = qq->fdStandard; // .. but disable O_DIRECT
        }
    }


    //////////////////////////////////////////////////////////////
    // Allocate and initialize the buffers, and start the reads
    if (numBuffers > count) numBuffers = count;
    for (ii = 0;  ii < numBuffers;  ++ii) {
        qEntryType *qe = __asyncIO_allocateBufHeader ();

        // Initiate a read for this buffer, and put it in the
        // "pending" queue.
        asyncIO_putEmptyInputBuffer (qq, &(qe->bufHeader));
    }


    return qq;
}



void *
asyncIO_initInputQ_fn (
  const char *fn,
  long int fileOffsetInBytes,
  long int totalSizeToReadInBytes,
  int numBuffers,
  long int bufferSizeInBytes,
  int flags)
{
    int ii;
    struct stat statBuf;

    if (stat (fn, &statBuf) < 0) {
        perror ("ERROR trying to stat input file:");
        fprintf (stderr, "File: '%s'\n", fn);
        exit (1);
    }

    assert (fileOffsetInBytes >= 0);
    if (totalSizeToReadInBytes < 0) {
        totalSizeToReadInBytes = statBuf.st_size - fileOffsetInBytes;
    }
    assert (fileOffsetInBytes + totalSizeToReadInBytes <= statBuf.st_size);
    assert (totalSizeToReadInBytes >= 0);
    assert (numBuffers > 0);
    assert (bufferSizeInBytes > 0);

    int numChunks = ((totalSizeToReadInBytes - 1) / bufferSizeInBytes) + 1;
    if (numChunks <= 0) numChunks = 1;
    if (numBuffers > numChunks) numBuffers = numChunks;

    long int offsets[numChunks], sizes[numChunks], tags[numChunks];
    for (ii = 0;  ii < numChunks;  ++ii) {
        offsets[ii] = fileOffsetInBytes + (ii * bufferSizeInBytes);
        sizes[ii] = bufferSizeInBytes;
        tags[ii] = ii;
    }
    // The last chunk might be short
    sizes[numChunks-1] = (fileOffsetInBytes + totalSizeToReadInBytes) -
                          offsets[numChunks-1];

    return  asyncIO_initInputQ_fn_full (
                 fn, numChunks, offsets, sizes, tags,
                 numBuffers, bufferSizeInBytes, flags);
}



// aio does not guarantee that the file position pointer will be unchanged.
// We want to offer that guarantee, so we use the linux "/proc/self/fd/"
// hack to get access to the filename, and reopen the file on a new
// fd.  This also has the advantages that we don't need to trust that the
// user has opened the file with the correct options, and it allows us to
// close the (new) fd during cleanup.

void *
asyncIO_initInputQ_fd_full (
  int fd,
  int count,
  long int *fileOffsets,
  long int *chunkSizes,
  long int *tags,
  int numBuffers,
  long int minBufferSizeInBytes,
  int flags)
{
    assert (fd >= 0);
    char name[FILENAME_MAX];
    sprintf(name, "/proc/self/fd/%d", fd);
    return asyncIO_initInputQ_fn_full (name,
                count, fileOffsets, chunkSizes, tags,
                numBuffers, minBufferSizeInBytes, flags);
}


void *
asyncIO_initInputQ_fd (
  int fd,
  long int fileOffsetInBytes,
  long int totalSizeToReadInBytes,
  int numBuffers,
  long int bufferSizeInBytes,
  int flags)
{
    assert (fd >= 0);
    char name[FILENAME_MAX];
    sprintf(name, "/proc/self/fd/%d", fd);
    return  asyncIO_initInputQ_fn (name, fileOffsetInBytes,
                 totalSizeToReadInBytes, numBuffers, bufferSizeInBytes, flags);
}



/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
// initOutput  routines

void *
asyncIO_initOutputQ_fn (
  const char *fn,
  int numBuffers,
  long int bufferSize,
  long int initialOffset,
  int flags)
{
    int ii;

    if (numBuffers < 0) numBuffers = 0;
    if (bufferSize < 0) bufferSize = 0;


    ////////////////////////////////////////////////////////////////////
    // Allocate and initialize the outQueue header
    outQueueHeaderType *qq = calloc (1, sizeof(outQueueHeaderType));
    qq->checkWord = outQCheckWordValue;
    qq->fdStandard = qq->fdDirect = -1;
    qq->nextFileOffset = initialOffset;
    qq->initialBufferSizeInBytes = bufferSize;
    qq->pending.head = qq->pending.tail = NULL;
    qq->available.head = qq->available.tail = NULL;


    // Open the file.  Note that if O_DIRECT is not enabled, then
    // fdDirect is set to the same value as fdStandard
    int openFlags = O_WRONLY|O_CREAT;

    int fd = open (fn, openFlags, 0644);
    if (fd < 0) {
        perror ("ERROR can't open output file");
        fprintf (stderr, "File: '%s'\n", fn);
        exit (1);
    }
    qq->fdDirect = qq->fdStandard = fd;

    if ((flags & asyncIO_USE_O_DIRECT) || (flags & O_DIRECT)) {
        DIE_IF ((qq->fdDirect = open(fn, openFlags|O_DIRECT, 0644)) < 0);
    }


    // Stat the file
    struct stat statBuf;
    if (stat (fn, &statBuf) < 0) {
        perror ("ERROR trying to stat output file:");
        fprintf (stderr, "File: '%s'\n", fn);
        exit (1);
    }

    // Handle initialOffset of -1
    if (initialOffset < 0) {
        qq->nextFileOffset = statBuf.st_size;
    }


    ///////////////////////////////////////////////////////////////////
    // Check the alignment setting.
    // Currently (May2017), Lustre file systems return the error
    // "inappropriate ioctl" for the BLKSSZGET ioctl, despite the fact
    // that they do support O_DIRECT.  After some debate, the decision
    // was that if the user knows what they are doing, and they want to
    // use O_DIRECT even if the file system doesn't support BLKSSZGET,
    // they have to set __asyncIO_align.  If __asyncIO_align is not set,
    // and the BLKSSZGET ioctl fails, then O_DIRECT is disabled.
    if (__asyncIO_align > 0) {
        qq->alignment = __asyncIO_align;
    } else {
        if (ioctl (qq->fdStandard, BLKSSZGET, &(qq->alignment)) < 0) {
            qq->alignment = statBuf.st_blksize;  // Fill in a value here ..
            qq->fdDirect = qq->fdStandard; // .. but disable O_DIRECT
        }
    }



    ////////////////////////////////////////////////////////////////////
    // Setup the output headers
    for (ii = 0;  ii < numBuffers;  ++ii) {
        qEntryType *qe = __asyncIO_allocateBufHeader ();
        if (bufferSize > 0) {
            qe->bufHeader.buffer =
                     __asyncIO_allocateBuffer (bufferSize, qq->alignment);
            qe->bufHeader.maxSizeInBytes = bufferSize;
        }
        __asyncIO_enqueue (&(qq->available), qe);
    }

    return qq;
}


void *
asyncIO_initOutputQ_fd (
  int  fd,
  int numBuffers,
  long int bufferSize,
  long int initialOffset,
  int flags)
{
    assert (fd >= 0);
    char name[FILENAME_MAX];
    sprintf(name, "/proc/self/fd/%d", fd);
    return  asyncIO_initOutputQ_fn (name, numBuffers,
                         bufferSize, initialOffset, flags);
}



/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
// get/put  buffers

void
asyncIO_putEmptyInputBuffer (void *q, asyncIO_ioBufferHeaderType *bh)
{
    // If there is more input for this queue, use this released buffer
    // to initiate a read, and put it into the "pending" queue.  If there
    // is no more input, just put the buffer in the "available" queue.

    inQueueHeaderType *qq = (inQueueHeaderType *)q;
    assert (inQCheckWordValue == qq->checkWord);

    // Convert the asyncIO_ioBufferHeaderType pointer (a type visible to
    // the user) into its enclosing qEntryType pointer (an opaque type).
    qEntryType *qe = (qEntryType *) bh;

    // Check that the user didn't pass us a bogus pointer
    assert ( ((ptrdiff_t) &(qe->checkWord)) == qe->checkWord );

    // Check for more input
    if (qq->nextChunk >= qq->numChunks) {
        // No more input
        bh->k = bh->n = bh->tag = -1;
        __asyncIO_enqueue (&(qq->available), qe);
    }
    else {
        int indx = qq->nextChunk;
        long int off = qq->fileOffsets[indx];
        long int sz = qq->chunkSizes[indx];
        if (sz < 0) sz = 0;
        bh->tag = qq->tags[indx];
        bh->k = bh->n = 0;

        // If needed, re-allocate the buffer
        if ( (NULL == bh->buffer) || (bh->maxSizeInBytes < sz) )
        {
            free (bh->buffer);
            bh->buffer = __asyncIO_allocateBuffer (sz, qq->alignment);
            bh->maxSizeInBytes = sz;
        }

        // Figure out how many pieces we need, and allocate the aio
        // control block(s) as needed.
        int ii, nn = ((sz - 1) / __asyncIO_maxXactSizeInBytes) + 1;
        if (qe->aiocb_numAllocated < nn) {
            free (qe->aio);
            qe->aio = malloc (nn * sizeof(struct aiocb));
            assert (qe->aio != NULL);
            qe->aiocb_numAllocated = nn;
        }

        // Start each piece
        for (ii = 0;  ii < nn;  ++ii) {
            long int pieceOffset = ii * __asyncIO_maxXactSizeInBytes;

            long int thisOffset = off + pieceOffset;
            long int thisSize = (ii != (nn - 1)) ?
                    __asyncIO_maxXactSizeInBytes : (sz - pieceOffset);
            void *thisBuf = (void *) (((ptrdiff_t)bh->buffer) + pieceOffset);

            ptrdiff_t all = ((ptrdiff_t)thisBuf) | thisOffset | thisSize;
            int thisFD = isAligned (all, qq->alignment) ?
                          qq->fdDirect : qq->fdStandard;

            memset (&(qe->aio[ii]), 0, sizeof(qe->aio[ii]));
            qe->aio[ii].aio_fildes = thisFD;
            qe->aio[ii].aio_offset = thisOffset;
            qe->aio[ii].aio_buf = thisBuf;
            qe->aio[ii].aio_nbytes = thisSize;
            qe->aio[ii].aio_lio_opcode = LIO_READ;
            DIE_IF (aio_read(&(qe->aio[ii])) < 0);
        }
        qe->aiocb_numInUse = nn;

        __asyncIO_enqueue (&(qq->pending), qe);
        qq->nextChunk += 1;
    }
}



asyncIO_ioBufferHeaderType *
asyncIO_getNextInputBuffer (void *q)
{
    inQueueHeaderType *qq = (inQueueHeaderType *)q;
    assert (inQCheckWordValue == qq->checkWord);

    // Check if any input ops are already in-flight
    if (NULL == qq->pending.head) {
        // If we're done, just return NULL
        if (qq->nextChunk >=  qq->numChunks) {
            return NULL;
        } else {
            // Not done, so allocate another buffer
            qEntryType *qe = __asyncIO_allocateBufHeader ();
            asyncIO_putEmptyInputBuffer (qq, &(qe->bufHeader));
        }
    }

    // Pop the head of the pending queue, and wait for it to complete
    assert (qq->pending.head != NULL);
    qEntryType *qe = __asyncIO_dequeue (&(qq->pending));
    qe->bufHeader.sizeInBytes = __asyncIO_completeAIO (qe);

    return &(qe->bufHeader);
}



void
asyncIO_putNextOutputBuffer (
  void *q,
  asyncIO_ioBufferHeaderType *bh,
  long int offset)
{
    outQueueHeaderType *qq = (outQueueHeaderType *)q;
    assert (outQCheckWordValue == qq->checkWord);

    qEntryType *qe = (qEntryType *) bh;
    assert ( ((ptrdiff_t) &(qe->checkWord)) == qe->checkWord );

    assert ((bh->sizeInBytes >= 0) && (bh->sizeInBytes <= bh->maxSizeInBytes));
    assert (bh->buffer != NULL);

    if (offset < 0) offset = qq->nextFileOffset;

    // Figure out how many pieces we need, and allocate the aio
    // control block(s) as needed.
    int ii, nn = ((bh->sizeInBytes - 1) / __asyncIO_maxXactSizeInBytes) + 1;
    if (qe->aiocb_numAllocated < nn) {
        free (qe->aio);
        qe->aio = malloc (nn * sizeof(struct aiocb));
        assert (qe->aio != NULL);
        qe->aiocb_numAllocated = nn;
    }

    // Start each piece
    for (ii = 0;  ii < nn;  ++ii) {
        long int pieceOffset = ii * __asyncIO_maxXactSizeInBytes;

        long int thisOffset = offset + pieceOffset;
        long int thisSize = (ii != (nn - 1)) ?
                __asyncIO_maxXactSizeInBytes : (bh->sizeInBytes - pieceOffset);
        void *thisBuf = (void *) (((ptrdiff_t)bh->buffer) + pieceOffset);

        ptrdiff_t all = ((ptrdiff_t)thisBuf) | thisOffset | thisSize;
        int thisFD = isAligned (all, qq->alignment) ?
                      qq->fdDirect : qq->fdStandard;

        memset (&(qe->aio[ii]), 0, sizeof(qe->aio[ii]));
        qe->aio[ii].aio_fildes = thisFD;
        qe->aio[ii].aio_offset = thisOffset;
        qe->aio[ii].aio_buf = thisBuf;
        qe->aio[ii].aio_nbytes = thisSize;
        qe->aio[ii].aio_lio_opcode = LIO_WRITE;
        DIE_IF (aio_write(&(qe->aio[ii])) < 0);
    }
    qe->aiocb_numInUse = nn;

    qq->nextFileOffset = offset + bh->sizeInBytes;
    __asyncIO_enqueue (&(qq->pending), qe);
}



asyncIO_ioBufferHeaderType *
asyncIO_getEmptyOutputBuffer (void *q)
{
    outQueueHeaderType *qq = (outQueueHeaderType *) q;
    assert (outQCheckWordValue == qq->checkWord);

    qEntryType *qe;

    // If there are empty buffer(s) already available, use one of those.
    // Otherwise, wait for the head of the pending queue to complete
    qe = __asyncIO_dequeue (&(qq->available));
    if (NULL == qe) {
        qe = __asyncIO_dequeue (&(qq->pending));
        if (qe != NULL) {
            __asyncIO_completeAIO (qe);
        } else {
            // Nothing was either available, nor pending, so allocate
            // another buffer
            qe = __asyncIO_allocateBufHeader ();
            if (qq->initialBufferSizeInBytes > 0) {
                qe->bufHeader.buffer = __asyncIO_allocateBuffer
                           (qq->initialBufferSizeInBytes, qq->alignment);
                qe->bufHeader.maxSizeInBytes = qq->initialBufferSizeInBytes;
            }
        }
    }

    qe->bufHeader.sizeInBytes = 0;
    qe->bufHeader.tag = 0;
    qe->bufHeader.k = 0;
    qe->bufHeader.n = 0;

    return &(qe->bufHeader);
}



/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
// Cleanup

// Complete any pending async output
void
asyncIO_finishPendingOutput (void *q)
{
    qEntryType *qe;
    outQueueHeaderType *qq = (outQueueHeaderType *) q;
    assert (outQCheckWordValue == qq->checkWord);

    while ( (qe = __asyncIO_dequeue (&(qq->pending))) != NULL ) {
        __asyncIO_completeAIO (qe);
        __asyncIO_enqueue (&(qq->available), qe);
    }
}



void
asyncIO_cleanupInputQ (void *q)
{
    qEntryType *qe;
    inQueueHeaderType *qq = (inQueueHeaderType *)q;
    assert (inQCheckWordValue == qq->checkWord);

    // Deal with any pending input requests
    while ( (qe = __asyncIO_dequeue (&(qq->pending))) != NULL ) {
        // We should probably cancel (rather than complete) pending input
        __asyncIO_completeAIO (qe);
        __asyncIO_enqueue (&(qq->available), qe);
    }

    // Free the buffers, the headers, and the queue itself

    while ( (qe = __asyncIO_dequeue (&(qq->available))) != NULL) {
        free (qe->bufHeader.buffer);
        free (qe->aio);
        free (qe);
    }

    assert (NULL == qq->pending.head);
    assert (NULL == qq->available.head);

    free (qq->fileOffsets);
    free (qq->chunkSizes);
    free (qq->tags);

    close (qq->fdStandard);
    close (qq->fdDirect);

    free (qq);
}



void
asyncIO_cleanupOutputQ (void *q)
{
    qEntryType *qe;
    outQueueHeaderType *qq = (outQueueHeaderType *) q;
    assert (outQCheckWordValue == qq->checkWord);

    asyncIO_finishPendingOutput(qq);

    // Free the buffers, the headers, and the queue itself
    while ( (qe = __asyncIO_dequeue (&(qq->available))) != NULL) {
        free (qe->bufHeader.buffer);
        free (qe->aio);
        free (qe);
    }

    close (qq->fdStandard);
    close (qq->fdDirect);

    free (qq);
}


