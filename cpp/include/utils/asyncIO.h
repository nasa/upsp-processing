
#if !defined(ASYNCIO_H)
#define ASYNCIO_H

#ifdef __cplusplus
extern "C"
{
#endif


//
// Asynchronous read/write.  These routines present a relatively simple
// interface for asynchronously reading and/or writing files in chunks
// of a user-specified size.  It uses the "aio" set of routines (in
// particular aio_read and aio_write) for the underlying functionality.
//
//
// [A note on terminology: A "buffer" is a contiguous area of *memory*.
// A "chunk" is a contiguous portion of the user's *data*.  Thus, a
// chunk might be in a file, or in a buffer, while a buffer may be
// empty, or have (some portion of) a chunk stored in it.]
//
//
// Overview:
// ----------
// When reading a file, the user specifies up front how the read of the
// input file is to be broken up, i.e. the chunk sizes and file offsets,
// and also how many buffers are to be initially allocated.  This is done
// by calling one of the "initInputQ" routines.  The init routines support
// a wide variety of input specifications, but allow simple defaults
// for the typical cases (e.g. reading a file from begining to end in
// fixed sized chunks).  The input chunks are returned one at a time, in
// order, by calling the "getNextInputBuffer" routine.  Once the chunk of
// data in the buffer is used up, the buffer is returned via the
// "putEmptyInputBuffer" call, which automatically initiates the read of
// the next chunk.  When the input is complete, a call to "cleanupInputQ"
// deletes the headers and buffers.
//
// Initializing for writing is much the same, except that the chunks do
// not need to be specified in advance.  The size of the chunk, and the
// offset within the file, is specified when each write is initiated
// via the "putNextOutputBuffer" call.
//
// When reading, you get a full buffer from the queue, and put an empty
// buffer into the queue (getNextInputBuffer and putEmptyInputBuffer).
// getNextInputBuffer supplies the read chunks one at a time, in the
// order specified by the init call.  When all the input has been read
// (e.g. end-of-file), further calls to getNextInputBuffer return NULL.
//
// When writing, you get an empty buffer from the queue, and put a full
// buffer into the queue (getEmptyOutputBuffer and putNextOutputBuffer).
// The user must specify how full the output buffer is (i.e. how much
// is to be written to the file).
//
// When finished, the buffers and data structures are deallocated by calling
// cleanupInputQ and/or cleanupOutputQ as appropriate.  As a side effect,
// cleanupOutputQ will also wait until all the pending aio writes have been
// completed.
//
//
// Note that these routines are insensitive to the current value of the
// file position pointer (e.g. the result of "lseek (fd, SEEK_CUR, 0)"),
// and the file position pointer is not changed by any of these routines.
//
//
//
// Buffer Management:
// ------------------
// A buffer contains a chunk of data.  It is controlled by a header
// struct of type "asyncIO_ioBufferHeaderType", described in detail
// below.  When a buffer header is "put" into an input queue, an
// asynchronous read is initiated to fill the buffer.  [This includes
// the set of buffers created when the queue is originally created.]
// If the existing buffer is not large enough to hold the next chunk,
// it is free(3)'d, and a new buffer is allocated.
// When a buffer header is "put" into an output queue, an asynchronous
// write is initiated to empty the buffer.
//
// If a call is made to "get" a buffer, the head of the queue is selected,
// and we wait until its asynchronous i/o operation completes.  The
// header is then returned to the user.  The depth of the queue is thus
// the number of "in-flight" operations taking place.  If there are no
// headers in the corresponding queue, a new header is created to fulfill
// the "get" request.
//
// The user may NOT "put" asyncIO_ioBufferHeaderType struct's of their
// own creation; they must only use ones supplied by a "get" routine.
// This is because there is extra "hidden" information associated with
// the headers.  That said, the headers themselves are agnostic about
// whether they are input or output, and the queues do not track the
// headers once they have been given to the user via a "get".  It is
// specifically permitted to get a header from an input queue, and then
// put that same header into an output queue (and vice-versa).
//
// It is generally a good idea to ensure that there is always at least
// one buffer waiting in a queue at all times, so that at least one
// asynchronous operation can be in-flight.  The typical way of ensuring
// this is to create a few buffers at queue init time, and then either
// put a buffer back into the same queue you got it from, or else
// exchange the buffer with one from the target queue.  For example:
//     Get an input buffer from each of two different input queues,
//     and an output buffer from an output queue.  Combine elements
//     of the input buffers together, putting the result into the output
//     buffer.  The three buffers are then each put back into the queue
//     they originated from.
// or
//     Get a buffer of input data.  Operate on the input data "in
//     place", i.e. write the results back into the input buffer.
//     Now get an (empty) buffer from the output queue.  Put the former
//     input buffer (which now contains the results) into the output
//     queue, and put the output buffer into the input queue.
//
// These schemes ensure that the buffers remain distributed among the
// queues, and allow the asynchrony to work.  But these work flow
// recommendations are only suggestions, not requirements.
//
//
// Headers:
// ---------
// The buffers are controlled by a header:
//
// typedef struct {
//     void *buffer;
//     long int  maxSizeInBytes;  // Size of the attached buffer
//     long int  sizeInBytes;     // Amount of data actually in the buffer
//
//     long int  tag;
//     long int  k,n;      // Available to the user (unused by the i/o routines)
// } asyncIO_ioBufferHeaderType;
//
// buffer   A pointer to the buffer memory.  This buffer is allocated
//          using posix_memalign(3), aligned with the returned value of
//            ioctl (fd, BLKSSZGET, &align)
//          which (should) make it suitable to be used with O_DIRECT.
//
// maxSizeInBytes   The size of the allocated buffer (in bytes).
//
// sizeInBytes   The amount of "active" data in the buffer (in bytes).
//
//               On Input, this is set by the input routines to be equal
//               to the amount read.
//
//               On Output, the *user* must set this field to indicate
//               how many bytes are to be written out of the buffer and
//               into the output file.  It can be any value in the range
//               [0,maxSizeInBytes] inclusive.
//
// tag   When initializing an input queue, the user may specify a tag
//       value to be returned with each chunk.  This can aid the user in
//       keeping track of which chunk is currently in the buffer.  If
//       the user does not specify a tag, the value is the number of
//       chunks returned previously (i.e. the first call to getNextInputBuffer
//       returns a tag of 0, the next call returns a tag of 1, and so on).
//       The tag has no meaning for output queues and is ignored.
//
// k,n   These fields are available for the user's convenience.  They are
//       not used by the i/o routines.  They would typically be used e.g.
//       to keep track of how much of the buffer the user has already
//       processed.  Both values are initially set to zero when the user
//       "get"s a buffer.
//
//
// It IS permissible for the user to replace the "buffer" field, e.g.
// exchange pointers with some other buffer to avoid copying data.
// However, the implementation might free(3) and/or re-allocate the
// buffer pointed at by the header, so the user must insure the buffer
// is allocated by one of the malloc(3) family of routines (i.e. the
// pointer value must be an acceptable argument to free(3)).  Also,
// if the buffer is replaced, the "maxSizeInBytes" field must be
// updated to reflect the size of the new buffer.  And additionally,
// if you wish to use O_DIRECT, the buffer must be aligned correctly
// (e.g. by using posix_memalign(3)).  Buffers allocated by the
// asynchIO routines will fulfill these requirements.
// Note in particular that it IS permissible for the user to free(3)
// the existing buffer, and set the buffer pointer to NULL (and the
// maxSizeInBytes to zero).  If the header is subsequently "put" into
// a queue, a new buffer will be allocated as needed.
//
//
// O_DIRECT:
// ----------
// Use of the O_DIRECT flag is supported.  The "flags" argument of the
// init call should include the value "asyncIO_USE_O_DIRECT" (defined
// in this header file).  As a convenience, we also support using the
// value "O_DIRECT" (from fcntl.h).  The value (either value) should be
// or'd into the flags argument.  If this flag is set, then any time the
// read (or write) is properly aligned, it will use an O_DIRECT operation.
// Each requested i/o is checked for alignment just before the request
// is issued.  Note that if the i/o is NOT properly aligned, the
// operation will be done anyway, but with O_DIRECT disabled.  Not all
// filesystem types guarrentee that it is acceptable to mix O_DIRECT
// and non-O_DIRECT i/o to the same file, so some caution is in order.
//
// The check for "properly aligned" uses the result of:
//     int align;
//     ioctl (directFD, BLKSSZGET, &align);
// for the alignment value.  It is up to the user to ensure that the
// chunk requests are properly aligned if they want O_DIRECT to be used.
//
// Under Linux, the O_DIRECT interface is not tightly defined, and there
// can be differences between different filesystem types.  In particular,
// the asyncIO implementation assumes that it is ok to mix O_DIRECT and
// normal reads and writes to the same file.  This may not be true for
// your particular filesystem.  If it is not true, you should either avoid
// using O_DIRECT, or manually ensure that *all* the chunks can use
// O_DIRECT.  A final option is that if there is an alignment value more
// restrictive than BLKSSZGET that would make things work, the user
// can set the alignment value directly by assigning it to the global
//   extern long int __asyncIO_align;
// before calling the init routine.  The value must be a power-of-two.
//
// Note: as of this writing (May2017), Lustre filesystems do not support
// the BLKSSZGET ioctl, despite the fact that they do support O_DIRECT.
// Normally, this implementation disables O_DIRECT if BLKSSZGET fails.
// To work around the problem until Lustre fixes the ioctl, you can set
// __asyncIO_align to the "st_blksize" value from an [f]stat call.
//
//////////////////////////////////////////////////////////////////////////////




// __asyncIO_maxXactSizeInBytes : this is the maximum size used for an
// individual i/o system call.  If the user requests a read (or write)
// of a chunk that exceeds this size, it is broken into multiple
// transactions of this size or less, which are then combined together
// to fullfill the original user request.  This all happens automatically,
// and normally the user need not concern themselves with this.
// If there is some reason why the underlying physical filesystem would
// benifit from a different transaction size, the user may optionally
// set this external.  If the user cares about O_DIRECT working, this
// size needs to be set to a multiple of the BLKSSZGET alignment value.
// If you don't care about O_DIRECT, any value may be used.
//
// Very large values for this number can sometimes lead to "starvation"
// of other processes doing i/o.  And in the extreme, there is typically
// some system imposed limit on the size of a single i/o transaction
// (on my current Linux box, this appears to be  2^31 - 4096  bytes).

extern long int __asyncIO_maxXactSizeInBytes; // Default: 256 * 1024 * 1024



// __asyncIO_align : the alignment to use for O_DIRECT access.  This is
// normally taken care of by the implementation.  If for some reason the
// user wishes to specify an even more restrictive alignment, they may
// optionally set this external.

extern long int __asyncIO_align;  // Default:  ioctl(BLKSSZGET)






typedef struct {
    void *buffer;
    long int  maxSizeInBytes;
    long int  sizeInBytes;
    long int  tag;
    long int  k,n;
} asyncIO_ioBufferHeaderType;

// Flag values
#define   asyncIO_USE_O_DIRECT   040000



///////////////////////////////////////////////////////////////////////////
// Prototypes:

// Init:
//
// For input files, the user needs to specify at init time how the file
// is to be read.  We need to know this in advance so that we can do
// read-ahead.  There are two versions of the init interface: a simplified
// version, which allows one to read a single contiguous part of a file
// (up to and including the whole file), and a more complicated version
// that allows one to specify arbitrary pieces of the file to be read,
// and in arbitrary order.  Each of these two versions is available in
// two "flavors": accepting either an open file descriptor, or a file name
// (e.g. as with the difference between "stat" and "fstat").
//
// The simple init looks like this:
// void *
// asyncIO_initInputQ_fd (
//   int fd,
//   long int fileOffsetInBytes,
//   long int totalSizeToReadInBytes,
//   int numBuffers,
//   long int bufferSizeInBytes,
//   int flags);
//
// fd : an open file descriptor.  [In the other flavor of this routine
//      (i.e. asyncIO_initInputQ_fn), this argument is replaced by
//      a (char*) pointing to a file name.]
//
// fileOffsetInBytes : offset from the begining of the file to begin
//      reading.  "Zero" means "start from the begining of the file".
//
// totalSizeToReadInBytes : The number of byes to read from the file,
//      starting from the offset.  "-1" means "to the end of the file".
//
// numBuffers : The number of buffers initially allocated to do input.
//      An initial aio_read will be issued for each of these buffers.
//
// bufferSizeInBytes : How big each of the buffers will be.  This will
//      also be the size of each asynchronous read (except possibly the
//      last one, which might be short depending on the file size).
//
// flags : Currently, the only supported flag is "asyncIO_USE_O_DIRECT",
//      or equivalently "O_DIRECT".  This requests the use of O_DIRECT i/o,
//      when the data alignments allow it.
//
// Examples:
//
//      int MiB = 1024*1024;
//      void *qh = asyncIO_initInputQ_fd (fd, 0, -1, 3, MiB, 0);
//
//   This requests that the entire file be read, front to back,
//   in one MiB pieces, starting with three buffers.
//
//
//      int MiB = 1024*1024;
//      long int GiB = 1024L * MiB;
//      void *qh = asyncIO_initInputQ_fd (fd, MiB, 10*GiB, 5, 20*MiB, O_DIRECT);
//
//   This requests we start reading one MiB in from the front of the file,
//   read 10 GiB, using 5 buffers, in 20 MiB chunks, using O_DIRECT i/o when
//   possible.
//
//
// The more complex init looks like this:
// void *
// asyncIO_initInputQ_fd_full (
//   int fd,
//   int count,
//   long int *fileOffsets,
//   long int *chunkSizes,
//   long int *tags,
//   int numBuffers,
//   long int minBufferSizeInBytes,
//   int flags);
//
// Here, instead of specifying a single contiguous block of the file to be
// read, we specify each individual chunk.  The offsets need not be
// contiguous, nor even ascending, and the chunkSizes need not be the same.
//
// fileOffsets : An array of offsets within the file, specifying where
//      to begin reading each chunk.  In bytes.
//
// chunkSizes : An array of sizes to be read, each one starting at the
//      corresponding fileOffset.
//
// count : The length of the fileOffsets and chunkSizes arrays.  The
//      two arrays must be the same length.  "count" must be positive.
//
// tags : If non-NULL, this is an array of length "count" containing
//      values that are returned in the "tag" field of the
//      asyncIO_ioBufferHeaderType when a chunk is read.  The value is
//      not used by the asyncIO routines (other than being returned),
//      and is for the convenience of the user, to help keep track
//      of the data.  If "tags" is NULL, the value returned in the "tag"
//      field is sequentially increasing integers, starting at zero.
//
// numBuffers : same as for the simpler interface.
//
// minBufferSizeInBytes : the buffers will initially be allocated to
//     be of this size.  However, a buffer may be re-allocated if the
//     chunkSize to be read is bigger than the current size of the
//     buffer.  [Note that the buffers are NOT re-allocated to shrink
//     them for smaller reads.]  It is legal for this value to be zero.
//
// flags : same as for the simpler interface.
//
//
//
// For output files, things are simpler since there is no need to specify
// up front how to do the output, since the output cannot begin until
// after you have obtained the results to be written.
//
// Init of an output queue looks like this:
//
// void *
// asyncIO_initOutputQ_fd (
//   int fd,
//   int numBuffers,
//   long int bufferSizeInBytes,
//   long int initialOffset,
//   int flags);
//
// initialOffset : When you "put" an output buffer, you specify the
//      offset of the write (see "asyncIO_putNextOutputBuffer" below).
//      A specification of '-1' means "contiguously after the previous
//      write".  The initialOffset argument sets this location for the
//      first output buffer (essentially analogous to "lseek(SEEK_SET)").
//      Typically, this value will be '0' (meaning start at the begining),
//      but could be some positive offset.  A value of '-1' means to start
//      writing at the end of the file.  Note that the initialOffset value
//      only has meaning for the first output buffer put into the queue
//      using the asyncIO_putNextOutputBuffer call, and only if the
//      specified "offset" argument of that buffer is '-1'.
//
// The other arguments have meaning analogous to the arguments of the
// same names in the simpler input init routine.
//
// For initOutputQ, it is allowed for bufferSizeInBytes to be zero.
// This will produce a buffer header with the buffer pointer set to NULL
// (and maxSizeInBytes set to zero).  Either the user must supply their
// own malloc'd buffer, or else the header should be "put" into an input
// queue (which will allocate a buffer of an appropriate size for the
// next read).
//
//

////////////////////////////////
// Use open file descriptor

// The simpler version: read a contiguous part of a file.  If
// totalSizeToReadInBytes is negative, read to the end of the file.
// Returns a queueHandle.
void *
asyncIO_initInputQ_fd (
  int fd,
  long int fileOffsetInBytes,
  long int totalSizeToReadInBytes,
  int numBuffers,
  long int bufferSizeInBytes,
  int flags);

// All the bells and whistles.  It MUST be the case that fileOffsets and
// chunkSizes are both non-NULL, and "count" is set to reflect their length.
// Returns a queueHandle.
void *
asyncIO_initInputQ_fd_full (
  int fd,
  int count,
  long int *fileOffsets,
  long int *chunkSizes,
  long int *tags,
  int numBuffers,
  long int minBufferSizeInBytes,
  int flags);

// Returns a queueHandle.
void *
asyncIO_initOutputQ_fd (
  int fd,
  int numBuffers,
  long int bufferSizeInBytes,
  long int initialOffset,
  int flags);


////////////////////////////////
// Use a file name

// The simpler version: read a contiguous part of a file.  If
// totalSizeToReadInBytes is negative, read to the end of the file.
// Returns a queueHandle.
void *
asyncIO_initInputQ_fn (
  const char *fileName,
  long int fileOffsetInBytes,
  long int totalSizeToReadInBytes,
  int numBuffers,
  long int minBufferSizeInBytes,
  int flags);

// All the bells and whistles.  It MUST be the case that fileOffsets and
// chunkSizes are both non-NULL, and "count" is set to reflect their length.
// Returns a queueHandle.
void *
asyncIO_initInputQ_fn_full (
  const char *fileName,
  int count,
  long int *fileOffsets,
  long int *chunkSizes,
  long int *tags,
  int numBuffers,
  long int minBufferSizeInBytes,
  int flags);

// Returns a queueHandle.
void *
asyncIO_initOutputQ_fn (
  const char *fileName,
  int numBuffers,
  long int bufferSizeInBytes,
  long int initialOffset,
  int flags);



///////////////////////////////////////////////////////////////////////////
// Reading (input)

// Get the next buffer full of data from the file.
// (The input argument is the value returned from the init call.)
asyncIO_ioBufferHeaderType *
asyncIO_getNextInputBuffer (void *queueHandle);


// When finished with a buffer, this gives the buffer to the input
// queue, and begins the next async read.
void
asyncIO_putEmptyInputBuffer (
    void *queueHandle,
    asyncIO_ioBufferHeaderType *bh);

///////////////////////////////////////////////////////////////////////////
// Writing (output)

// Get an empty buffer from the output queue
asyncIO_ioBufferHeaderType *
asyncIO_getEmptyOutputBuffer(void *queueHandle);

// Begin an async write of a buffer of data to the output.
// Note that the user must set the bh->sizeInBytes field to
// specify how much data is to be written.  If the offset
// argument is negative, the buffer is written contiguously
// after the previously written buffer.
void
asyncIO_putNextOutputBuffer (
    void *queueHandle,
    asyncIO_ioBufferHeaderType *bh,
    long int offset);



///////////////////////////////////////////////////////////////////////////
// Termination

// Wait until the pending writes have completed
void
asyncIO_finishPendingOutput (void *queueHandle);

// Tear down the queue and release the memory
void
asyncIO_cleanupInputQ (void *queueHandle);

// Tear down the queue and release the memory
// (this also first calls "asyncIO_finishPendingOutput").
void
asyncIO_cleanupOutputQ (void *queueHandle);


#ifdef __cplusplus
}
#endif


#endif

