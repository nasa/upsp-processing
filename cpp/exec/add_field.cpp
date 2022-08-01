
//
// Add the "pressure" flat-file to an HDF5 file.
// What we want to do is pretty simple, but we have to deal with
// the full generality of the HDF5 interfaces
//

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>

#include <iostream>

#include "H5Cpp.h"
#include "utils/asyncIO.h"



int
main (int argc, char *argv[])
{
  assert (5 == argc);
  char *hdf5FileName = argv[1];
  char *dataSetName = argv[2];
  char *flatFileName = argv[3];
  unsigned int extent = atoi (argv[4]);
  assert (extent > 0);

  ///////////////////////////////////
  // Verify the flat file
  int fd = open (flatFileName, O_RDONLY);
  assert (fd >= 0);

  struct stat statBuf;
  int result = fstat (fd, &statBuf);
  assert (0 == result);

  unsigned long int nBytesPerRow = extent * sizeof(float);
  assert ((statBuf.st_size % nBytesPerRow) == 0);
  unsigned int nRows = statBuf.st_size / nBytesPerRow;


  //////////////////////////////////////////
  // Initiate asynch read of the flat file
  void *inQ = asyncIO_initInputQ_fd (fd, 0, -1, 3, nBytesPerRow * 100, 0);


try {

  ///////////////////////////////////
  // Define the new hdf5 data set

  // Set up the property list
  H5::DSetCreatPropList plist;
  float fillValue = 0.0;
  H5::DataType dType = H5::PredType::NATIVE_FLOAT;
  plist.setFillValue (dType, &fillValue);
  hsize_t chunkSizes[2] = {1, extent};
  plist.setChunk (2, chunkSizes);

  // Make a descriptor of the space we are going to create
  hsize_t dataDims[2] = {nRows, extent};
  H5::DataSpace dspace(2, dataDims);


  ///////////////////////////////////
  // Create the new hdf5 data set
  H5::H5File file (hdf5FileName, H5F_ACC_RDWR);
  auto dataSet = file.createDataSet (dataSetName, dType, dspace, plist);

  ///////////////////////////////////
  // Write to the new hdf5 data set

  unsigned int numRowsWritten = 0;
  asyncIO_ioBufferHeaderType *inBuf;
  while ( (inBuf = asyncIO_getNextInputBuffer (inQ)) != NULL) {

    // Find out how much data is in the buf
    unsigned int thisNumRows = inBuf->sizeInBytes / nBytesPerRow;
    assert ((thisNumRows * nBytesPerRow) == inBuf->sizeInBytes);

    // Make a descriptor for the thing we are going to write
    hsize_t writeDims[2] = {thisNumRows, extent};
    H5::DataSpace dataInMemoryDescriptor (2, writeDims);

    // Make a descriptor for the place in the file where we are going to write it
    H5::DataSpace dataInFileDescriptor = dataSet.getSpace();
    hsize_t nBlocks[2] = {1, 1};
    hsize_t offset[2] = {numRowsWritten, 0};
    hsize_t blockSize[2] = {thisNumRows, extent};
    dataInFileDescriptor.selectHyperslab (H5S_SELECT_SET, nBlocks, offset, NULL, blockSize);

    // Actually write it
    dataSet.write (inBuf->buffer, dType, dataInMemoryDescriptor, dataInFileDescriptor);
    numRowsWritten += thisNumRows;

    asyncIO_putEmptyInputBuffer (inQ, inBuf);
    dataInMemoryDescriptor.close();
    dataInFileDescriptor.close();
  }

  dataSet.close();
  dspace.close();
  file.close();
}
catch (H5::FileIException error) {
  std::cout << "Cannot open hdf5 file '" << hdf5FileName
            << "' : " << error.getDetailMsg() << std::endl;
}
catch (H5::GroupIException error) {
  std::cout << "Cannot parse hdf5 file '" << hdf5FileName
            << "' : " << error.getDetailMsg() << std::endl;
}
catch (H5::DataSetIException error) {
  std::cout << "HDF5 file missing dataSet '" << dataSetName
            << "' : " << error.getDetailMsg() << std::endl;
}

  asyncIO_cleanupInputQ (inQ);
  return 0;
}
