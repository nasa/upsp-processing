
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <climits>
#include <string.h>

#include <mpi.h>
#include <omp.h>


unsigned long int msize;  // Number of model nodes
unsigned long int number_frames;  // Number of frames

#ifndef HAS__MY_MPI_RANK
#define HAS__MY_MPI_RANK
int my_mpi_rank = 0;    
#endif
int num_mpi_ranks = 1;
std::vector <int> rank_start_frame;
std::vector <int> rank_num_frames;
std::vector <int> rank_start_node;
std::vector <int> rank_num_nodes;

int num_openmp_threads = -1;

volatile void *ptr_pressure_data = NULL;
volatile void *ptr_pressure_transpose_data = NULL;
volatile void *ptr_temp_workspace = NULL;


///////////////////////////////////////////////


void
timedBarrierPoint (
  bool do_output,
  const char *label)
{
  static double base = -1.0;
  static double previous = -1.0;

  double before_barrier = MPI_Wtime();

  if (base < 0.0) {
    base = before_barrier;
    previous = before_barrier;
  }

  MPI_Barrier (MPI_COMM_WORLD);
  double after_barrier = MPI_Wtime();

  if (do_output) {
    std::cout << "+++ " << label << "   [total elapsed:" << after_barrier - base
	      << ",  this thread since previous:" << before_barrier - previous
	      << "  barrier (load imblance): " << after_barrier - before_barrier << "]" << std::endl;
  }
  previous = after_barrier;
}


/////////////////////////////////////////////// 


void
apportion (
  unsigned long int value,
  unsigned long int nBins,
  int *start,
  int *extent)
{
  assert ((value >= 0) && (nBins > 0));
  unsigned long int blockSize = value / nBins;
  unsigned long int remainder = value - (blockSize * nBins);

  unsigned long int nextStart = 0;
  unsigned long int curBin;
  for (curBin = 0;  curBin < nBins;  ++curBin)
  {
    start[curBin] = nextStart;
    extent[curBin] = blockSize + (curBin < remainder);
    nextStart += extent[curBin];
  }
  assert (nextStart == value);
}


/////////////////////////////////////////////// 


void
local_transpose (
  float * __restrict__ src_ptr,
  int x_extent,
  int y_extent,
  float * __restrict__ dst_ptr)
{
  int block_size = 100;

  float (* __restrict__ src)[x_extent] = (float(*)[x_extent]) src_ptr;
  float (* __restrict__ dst)[y_extent] = (float(*)[y_extent]) dst_ptr;

  int full_x_blocks = x_extent / block_size;
  int full_y_blocks = y_extent / block_size;

#pragma omp parallel for  collapse(2)  schedule(dynamic,1)
  for (int y_block = 0;  y_block < full_y_blocks + 1;  ++y_block) {
    for (int x_block = 0;  x_block < full_x_blocks + 1;  ++x_block) {
      
      int y_block_start_index = y_block * block_size;
      int y_block_extent = ( (y_block < full_y_blocks) ?  block_size :
			     y_extent - (full_y_blocks * block_size));

      int x_block_start_index = x_block * block_size;
      int x_block_extent = ( (x_block < full_x_blocks) ?  block_size :
			     x_extent - (full_x_blocks * block_size));

      for (int jj = 0;  jj < y_block_extent;  ++jj) {
	for (int ii = 0;  ii < x_block_extent;  ++ii) {
	  dst[x_block_start_index + ii][y_block_start_index + jj]
	    = src[y_block_start_index + jj][x_block_start_index + ii];
	}
      }
      
    }
  }

}


// Transpose a distributed rectangular array.
// Conceptually, the full input is an array AA[number_rows][number_cols].
// Each MPI rank has a slice of full-length rows of this conceptual array,
// starting at row "rank_start_row[my_mpi_rank]", and extending for
// "rank_num_rows[my_mpi_rank]" rows.  We want to transpose this into
// another conceptual distributed array TT[number_cols][number_rows], with
// each MPI rank have a slice of this array, starting at row
// "rank_start_col[my_mpi_rank]", with "rank_num_cols[my_mpi_rank]"
// rows.  The local slice of the input array is in "src", while the
// slice of the transpose goes into "dst".

void
general_global_transpose (float *ptr_src, float *ptr_dst, 
			  unsigned long int number_rows, unsigned long int number_cols,
			  std::vector <int> rank_start_row, std::vector <int> rank_num_rows,
			  std::vector <int> rank_start_col, std::vector <int> rank_num_cols)
{
  int my_num_rows = rank_num_rows[my_mpi_rank];
  int my_num_cols = rank_num_cols[my_mpi_rank];
  MPI_Request requests[num_mpi_ranks];

  float (*src)[number_cols] = (float (*)[number_cols]) ptr_src;
  float (*dst)[number_rows] = (float (*)[number_rows]) ptr_dst;
  float (*temp)[my_num_rows] = (float (*)[my_num_rows]) ptr_temp_workspace;

  ///////////////////////////////////////////////////////////////////
  // Transpose the local slice from src into temp
  if (0 == my_mpi_rank) std::cout << "  Local transpose ..." << std::endl;
  local_transpose (&(src[0][0]), number_cols, my_num_rows, &(temp[0][0]));

  ///////////////////////////////////////////////////////////////////
  // MPI_Isend slices of "temp" to the appropriate ranks.
  if (0 == my_mpi_rank) std::cout << "  Global transpose exchange ..." << std::endl;
  for (int rank = 0;  rank < num_mpi_ranks;  ++rank) {
    long int start_col = rank_start_col[rank];
    long int num_cols = rank_num_cols[rank];
    assert ((num_cols * my_num_rows) < INT_MAX);
    MPI_Isend (&(temp[start_col][0]), num_cols * my_num_rows,
        MPI_FLOAT, rank, 0, MPI_COMM_WORLD, &(requests[rank]));
  }

  ///////////////////////////////////////////////////////////////////
  // MPI_Recv slices from the ranks (including ourself), and collect
  // the slices together into full-length rows of the transposed
  // matrix
  long int max_count = rank_num_rows[0] * rank_num_cols[0];
  float *recv_buf = (float *) malloc (max_count * sizeof(float));
  assert (recv_buf != NULL);
  for (int count = 0;  count < num_mpi_ranks;  ++count) {
    MPI_Status status;
    // Recv a message
    MPI_Recv (recv_buf, max_count, MPI_FLOAT, MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &status);

    // Figure out who it's from, and what size we expect it to be
    int sender_rank = status.MPI_SOURCE;
    int sender_num_rows = rank_num_rows[sender_rank];
    int expected_num_items = my_num_cols * sender_num_rows;

    // Verify it actually is the size we expected it to be
    int actual_num_items;
    MPI_Get_count (&status, MPI_FLOAT, &actual_num_items);
    assert (actual_num_items == expected_num_items);

    // Copy the recv'd data into the dst buf
    float (*recv_trans)[sender_num_rows] = (float (*)[sender_num_rows]) recv_buf;
    long int start_row = rank_start_row[sender_rank];
    for (long int col_offset = 0;  col_offset < my_num_cols;  ++col_offset) {
      for (long int row_offset = 0;  row_offset < sender_num_rows;  ++row_offset) {
        dst[col_offset][start_row + row_offset] = recv_trans[col_offset][row_offset];
      }
    }
  }
  free (recv_buf);

  // Make sure all the messages we sent have gotten out
  MPI_Waitall (num_mpi_ranks, requests, MPI_STATUSES_IGNORE);
}


///////////////////////////////////////////////


void
allocate_global_data (void)
{
  // Allocate the large data buffers.  The size given to rank 0
  // will be the maximum size.
  long int buf_size1 = rank_num_frames[0] * msize         * sizeof(float);
  long int buf_size2 = rank_num_nodes[0]  * number_frames * sizeof(float);
  long int large_buf_size = std::max(buf_size1, buf_size2);

  ptr_pressure_data = malloc (large_buf_size);
  assert (ptr_pressure_data != NULL);
  ptr_pressure_transpose_data = malloc (large_buf_size);
  assert (ptr_pressure_transpose_data != NULL);
  ptr_temp_workspace = malloc (large_buf_size);
  assert (ptr_temp_workspace != NULL);
}


///////////////////////////////////////////////


void
pwrite_full (int fd, const void *buf, size_t nbytes, off_t file_offset)
{
  unsigned char *src = (unsigned char *) buf;
  errno = 0;
  size_t amt_remaining = nbytes;
  while (amt_remaining > 0) {
    long int amt_written = pwrite (fd, src, amt_remaining, file_offset);
    if (amt_written < 0) perror ("pwrite_full");
    assert (amt_written > 0);
    amt_remaining -= amt_written;
    file_offset += amt_written;
    src += amt_written;
  }
}


///////////////////////////////////////////////


void read_data (float *ptrInputDataBuf, const char inputDataFileName[], unsigned long int number_cols, unsigned int my_start_row, unsigned int my_num_rows)
{
  FILE *ptrInputDataFile;

  if ( (ptrInputDataFile = fopen(inputDataFileName, "rb")) == NULL ) {
    std::cerr << "Error! input data file not opened.\n" << std::endl;
    exit(1);
  }
  fseek(ptrInputDataFile, sizeof(float) * my_start_row * number_cols, SEEK_SET);
  fread(ptrInputDataBuf, sizeof(float), my_num_rows * number_cols, ptrInputDataFile);
  fclose(ptrInputDataFile);
}


///////////////////////////////////////////////


int main(int argc, char *argv[]) 
{
  int flag_transpose, file_fd;
  std::string inputDataFileName, outputDataFolderName, outputDataFileNameStr;


  // Note that my_mpi_rank and num_mpi_ranks are global
  MPI_Init (&argc, &argv);
  MPI_Comm_rank (MPI_COMM_WORLD, &my_mpi_rank);
  MPI_Comm_size (MPI_COMM_WORLD, &num_mpi_ranks);

  timedBarrierPoint (0 == my_mpi_rank, "Begin");

  omp_set_nested(false);
  num_openmp_threads = omp_get_max_threads();

  if (0 == my_mpi_rank) {
    std::cout << num_mpi_ranks << " MPI ranks, each with "
	      << num_openmp_threads << " OpenMP threads" << std::endl;
  }
  rank_start_frame.resize(num_mpi_ranks);
  rank_num_frames.resize(num_mpi_ranks);
  rank_start_node.resize (num_mpi_ranks);
  rank_num_nodes.resize (num_mpi_ranks);


  // Read input arguments     
  msize = std::stoi(argv[1]);
  number_frames = std::stoi(argv[2]);
  flag_transpose = std::stoi(argv[3]);
  inputDataFileName = argv[4];
  outputDataFolderName = argv[5];

  if (my_mpi_rank == 0) {
    std::cout << "\nInput arguments:\n" << std::endl;
    std::cout << "msize = "<< msize << std::endl;
    std::cout << "numbeR_frames = " << number_frames << std::endl;
    std::cout << "flag_transpose = " << flag_transpose << std::endl;
    std::cout << "inputDataFileName = " << inputDataFileName << std::endl;
    std::cout << "outputDataFolderName = " << outputDataFolderName << std::endl;
  }

    
  // Calculate the frames assigned to each mpi rank 
  apportion (number_frames, num_mpi_ranks, &(rank_start_frame[0]), &(rank_num_frames[0]));
  unsigned int my_first_frame = rank_start_frame[my_mpi_rank];
  unsigned int my_num_frames = rank_num_frames[my_mpi_rank];

  // Calculate the node points assigned to each mpi rank
  apportion (msize, num_mpi_ranks, &(rank_start_node[0]), &(rank_num_nodes[0]));
  unsigned int my_first_node = rank_start_node[my_mpi_rank];
  unsigned int my_num_nodes = rank_num_nodes[my_mpi_rank];
    
  // std::cout << "\nRank: " << my_mpi_rank << "/" << num_mpi_ranks << 
  //   "  num frames: "  << number_frames  << "  model size: " << msize         <<
  //   "  first frame: " << my_first_frame << "  num frames: " << my_num_frames << 
  //   "  first node: "  << my_first_node  << "  num nodess: " << my_num_nodes  << std::endl;
   
  // Allocate the large data arrays  
  allocate_global_data();
  timedBarrierPoint (0 == my_mpi_rank, "allocated global data");
  MPI_Barrier (MPI_COMM_WORLD);
 
    
  // Read the data
  if (flag_transpose == 0) {
    if (0 == my_mpi_rank) std::cout << "Reading pressure data ..." << std::endl;
    read_data ((float*)ptr_pressure_data, inputDataFileName.c_str(), msize, my_first_frame, my_num_frames);
  }
  else {
    if (0 == my_mpi_rank) std::cout << "Reading pressure_transpose data ..." << std::endl;
    read_data ((float*)ptr_pressure_transpose_data, inputDataFileName.c_str(), number_frames, my_first_node, my_num_nodes);
  }
  timedBarrierPoint (0 == my_mpi_rank, "data reading complete");
  MPI_Barrier (MPI_COMM_WORLD);
  if (0 == my_mpi_rank) std::cout << "Data reading complete" << std::endl;
        
  // Construct the transpose
  if (0 == my_mpi_rank) std::cout << "Construct the transpose" << std::endl;
  float (*pressure_buf)[msize] = (float (*)[msize]) ptr_pressure_data;
  float (*pressure_transpose_buf)[number_frames] = (float (*)[number_frames]) ptr_pressure_transpose_data;
  if (flag_transpose == 0) {
    general_global_transpose (&(pressure_buf[0][0]), &(pressure_transpose_buf[0][0]), number_frames, msize, 
			      rank_start_frame, rank_num_frames, rank_start_node, rank_num_nodes);
  }
  else {
    general_global_transpose (&(pressure_transpose_buf[0][0]), &(pressure_buf[0][0]), msize, number_frames, 
			      rank_start_node, rank_num_nodes, rank_start_frame, rank_num_frames);
  }
  timedBarrierPoint (0 == my_mpi_rank, "transpose complete");
  MPI_Barrier (MPI_COMM_WORLD);
  if (0 == my_mpi_rank) std::cout << "transpose complete" << std::endl;

  // Create the output file
  std::string outputDataFolderNameStr = outputDataFolderName;
  if (flag_transpose == 0) {
    if (0 == my_mpi_rank) std::cout << "Create pressure_transpose file ..." << std::endl;
    outputDataFileNameStr = outputDataFolderNameStr + "/pressure_transpose";
  }
  else {
    if (0 == my_mpi_rank) std::cout << "Create pressure file ..." << std::endl;
    outputDataFileNameStr = outputDataFolderNameStr + "/pressure";
  }

  if (0 == my_mpi_rank) {
    // For Lustre file systems: create the files with lots of stripes 
    // (send errors to /dev/null in case it is not a Lustre file system).
    std::string cmd = "/bin/rm -f " + outputDataFileNameStr;
    cmd += " ; lfs setstripe -c 60 " + outputDataFileNameStr + " >& /dev/null";
    system (cmd.c_str());

    // Now "creat" the file, in case it was NOT a Lustre file system
    file_fd = open (outputDataFileNameStr.c_str(), O_WRONLY|O_CREAT, 0644);
    assert (file_fd >= 0);
    close (file_fd);
  }
  MPI_Barrier (MPI_COMM_WORLD);
  
  file_fd = open (outputDataFileNameStr.c_str(), O_WRONLY, 0644);
  assert (file_fd >= 0);
  timedBarrierPoint (0 == my_mpi_rank, "created output file");
  MPI_Barrier (MPI_COMM_WORLD);
  
  // Write the data
  if (flag_transpose == 0) {
    if (0 == my_mpi_rank) std::cout << "Writing pressure_transpose data ..." << std::endl;
    long int nbytes = number_frames * rank_num_nodes[my_mpi_rank] * sizeof(float);
    long int file_offset = number_frames * rank_start_node[my_mpi_rank] * sizeof(float);
    pwrite_full (file_fd, (void*)ptr_pressure_transpose_data, nbytes, file_offset);
  }
  else {
    if (0 == my_mpi_rank) std::cout << "Writing pressure data ..." << std::endl;
    long int nbytes = msize * rank_num_frames[my_mpi_rank] * sizeof(float);
    long int file_offset = msize * rank_start_frame[my_mpi_rank] * sizeof(float);
    pwrite_full (file_fd, (void*)ptr_pressure_data, nbytes, file_offset);
  }
  if (0 == my_mpi_rank) std::cout << "Data writing complete" << std::endl;
  timedBarrierPoint (0 == my_mpi_rank, "data writing complete");
  MPI_Barrier (MPI_COMM_WORLD);

    
  close (file_fd);

  free ((void*)ptr_pressure_data);
  free ((void*)ptr_pressure_transpose_data);
  free ((void*)ptr_temp_workspace);
  
  if (0 == my_mpi_rank) std::cout << "Wait for all ranks to complete" << std::endl;
  
  timedBarrierPoint (0 == my_mpi_rank, "Matrix transpsoe done");

  MPI_Finalize();

  return 0;
}
