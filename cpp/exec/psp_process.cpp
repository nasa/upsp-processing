#include <fcntl.h>
#include <mpi.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <boost/iterator/transform_iterator.hpp>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <tuple>
#include <opencv2/opencv.hpp>
#include <unordered_map>

#include "logging.h"
#include "plot3d.h"
#include "upsp.h"
#include "utils/file_writers.h"
#include "utils/file_readers.h"
#include "PSPVideo.h"
#include "CineReader.h"
#include "MrawReader.h"

////////////////////////////////////////////////////////////////////////////////
// SANDSTROM: 2019-Jun-10 ...
////////////////////////////////////////////////////////////////////////////////

#include "utils/pspKdtree.h"
#include "utils/pspRT.h"
#include "utils/cv_extras.h"

union UserData {
  void* ptr;
  uint64_t val;
};


template <typename M>
std::shared_ptr<rt::BVH> createBVH(const M& model,
                                   std::vector<int>& triNodes) {
  std::vector<float> tris;
  model.extract_tris(tris, triNodes);
  // a vector of triangles
  std::vector<std::shared_ptr<rt::Primitive>> triPrims =
      rt::CreateTriangleMesh(tris, 3);
  return std::make_shared<rt::BVH>(triPrims, 4);
}  // createBVH

template <typename M>
void getTargets(const M& model, const upsp::CameraCal cal,
                const std::vector<upsp::Target>& orig_targs,
                std::shared_ptr<rt::BVH> scene, float obliqueThresh,
                std::vector<upsp::Target>& targs) {
  // Remove targets that are not visible from camera, defined as:
  // - another part of the model lies between the target and the camera
  // - angle between model normal at target location and the
  //   camera view angle is too large
  const kdtree* kdroot = model.kdRoot();
  const std::vector<cv::Point3_<float>>& nrm =
      model.get_n();  // in tunnel frame
  cv::Point3_<float> cam_center = cal.get_cam_center();
  Imath::V3f orig(cam_center.x, cam_center.y, cam_center.z);
  int nvisible = 0;
  cv::Size sz = cal.size();

  for (auto targ : orig_targs) {
    // SANDSTROM: 2019-Jul-19 ...
    cv::Point_<float> img_pt = cal.map_point_to_image(targ.xyz);
    if (img_pt.x < 0 or img_pt.y < 0 or img_pt.x >= sz.width or
        img_pt.y >= sz.height)
      continue;
    // SANDSTROM: ... 2019-Jul-19

    Imath::V3f pos(targ.xyz.x, targ.xyz.y, targ.xyz.z);
    Imath::V3f dir = (pos - orig);
    float distFromEye = dir.length();

    dir.normalize();
    rt::Ray ray(orig, dir);
    rt::Hit hitrec;
    bool hit = scene->intersect(ray, &hitrec);
    if (not hit) continue;

    bool occluded = hitrec.t < (distFromEye - 1e-3);  // hackish
    if (not occluded) {
      double hitpos[3] = {hitrec.pos[0], hitrec.pos[1], hitrec.pos[2]};

      UserData udata;
      struct kdres* res = kd_nearest(const_cast<kdtree*>(kdroot), hitpos);
      udata.ptr = kd_res_item_data(res);
      int nearestNidx = static_cast<int>(udata.val);
      kd_res_free(res);

      cv::Point3_<float> tunnelNormal = nrm[nearestNidx];
      Imath::V3f nrm(tunnelNormal.x, tunnelNormal.y, tunnelNormal.z);
      float cos_theta = nrm.dot(dir);
      float ang = acos(cos_theta);
      bool forward = ang > obliqueThresh;

      if (forward) {
        ++nvisible;
        targs.push_back(targ);
      }

    }  // if not occluded

  }  // loop over all targets

}  // getTargets

template <typename M>
void get_target_diameters(const M& model, const upsp::CameraCal cal,
                          const std::vector<upsp::Target>& targs,
                          std::shared_ptr<rt::BVH> scene,
                          std::vector<float>& diams) {
  const kdtree* kdroot = model.kdRoot();
  diams.resize(targs.size());

  for (unsigned int i = 0; i < targs.size(); ++i) {
    // Check that target is in frame
    if ((targs[i].diameter == 0.0) ||
        (not upsp::contains(cal.size(), targs[i].uv))) {
      diams[i] = 0;
      continue;
    }

    // Get the model normal at the target using the nearest node
    double pos[3] = {targs[i].xyz.x, targs[i].xyz.y, targs[i].xyz.z};
    UserData udata;
    struct kdres* res = kd_nearest(const_cast<kdtree*>(kdroot), pos);
    udata.ptr = kd_res_item_data(res);
    int nearestNidx = static_cast<int>(udata.val);
    kd_res_free(res);

    auto node = model.node(nearestNidx);
    cv::Point3_<float> normal = node.get_normal();

    assert(cv::norm(normal) != 0.0);

    // Define a circle in the normal plane
    cv::Point3_<float> a, b;
    a = upsp::get_perpendicular(normal);
    b = a.cross(normal);

    // Get 4 points around the circle, project each to image, and
    // average the diameter
    float theta = 0.0;
    float out_diameter = 0.0;
    for (unsigned int j = 0; j < 4; ++j) {
      cv::Point3_<float> est_pt =
          targs[i].xyz + 0.5 * targs[i].diameter * std::cos(theta) * a +
          0.5 * targs[i].diameter * std::sin(theta) * b;
      cv::Point_<float> proj_est_pt = cal.map_point_to_image(est_pt);
      out_diameter += 2.0 * cv::norm(proj_est_pt - targs[i].uv);
      theta += 2 * PI / 4;
    }
    diams[i] = out_diameter / 4.0;
  }
}

template <typename M>
void create_projection_mat(const M& model, const upsp::CameraCal cal,
                           std::shared_ptr<rt::BVH> scene,
                           const std::vector<int>& triNodes,
                           const float obliqueThresh,
                           Eigen::SparseMatrix<float, Eigen::RowMajor>& smat,
                           std::vector<float>& uv,
                           cv::Mat_<uint8_t>& number_projected_model_nodes) {
  // psp::BlockTimer bt("create_projection_mat");
  cv::Size f_sz = cal.size();
  smat = Eigen::SparseMatrix<float, Eigen::RowMajor>(model.size(),
                                                     f_sz.width * f_sz.height);
  std::vector<Eigen::Triplet<float>> triplets;

  // Allocate space for UV coordinates
  // By default, populate each node's (u, v) with (0, 0)
  // (corner of image)
  uv.resize(model.size() * 2, 0.);

  // Convert 2D image point to 1D coordinate
  auto idx = [f_sz](cv::Point2i pix) { return pix.y * f_sz.width + pix.x; };

  // pre-calc'd normals in tunnel frame
  const std::vector<cv::Point3_<float>>& nrm = model.get_n();

  // Get the camera center for creating rays to model nodes
  cv::Point3_<float> cam_center = cal.get_cam_center();
  Imath::V3f orig(cam_center.x, cam_center.y, cam_center.z);

  // process each node in the model
  // system ("/usr/bin/date");

  unsigned int blockSize = 500;  // Adjust this value as appropriate

  unsigned int count = 0;
  typedef typename M::NodeIterator NodeIterator_t;
  std::vector<NodeIterator_t> blockBreaks;

  for (auto it = model.cnode_begin(); it != model.cnode_end(); ++it) {
    if ((count % blockSize) == 0) blockBreaks.push_back(it);
    ++count;
  }
  blockBreaks.push_back(model.cnode_end());
  unsigned int nBlocks = blockBreaks.size() - 1;

  volatile unsigned int nCompleted = 0;
  volatile unsigned int curBlock = 0;
  std::vector<Eigen::Triplet<float>> localTriplets[nBlocks];

// The "schedule" clause says "1", but note that this is 1 *block*
// (blockSize iterations).
#pragma omp parallel for schedule(dynamic, 1)
  for (unsigned int blockIndex = 0; blockIndex < nBlocks; ++blockIndex) {
    auto blockBegin = blockBreaks[blockIndex];
    auto blockEnd = blockBreaks[blockIndex + 1];
    unsigned int localTotalCount = 0;

    // Process the block of nodes

    bool lastWasVisible = false;
    for (auto it = blockBegin; it != blockEnd; ++it) {
      typename M::Node n = *it;
      ++localTotalCount;

      // todo-mshawlec update this comment ... model isn't transformed anymore!
      // TODO - mshawlec - verify this workaround
      // We don't have get_raw_position() anymore ...
      // ... and the model coordinates at this point
      // have been transformed. However, upstream of
      // this point in the code, we've already set_node_nondata
      // based on x_max so we can just check that instead
      //    // replicate the upsp::GreaterXPredicate
      // cv::Point3_<float> rpos = n.get_raw_position();
      // if( rpos.x > x_max )
      //    continue;
      if (!n.is_datanode()) {
        continue;
      }

      cv::Point3_<float> ipos = n.get_position();

      // map the 3D model point onto the 2D frame
      cv::Point_<float> pt = cal.map_point_to_image(ipos);

      // skip if the model point does not map onto the 2D frame
      if (not upsp::contains(f_sz, pt)) continue;

      // Define the ray from camera center to node
      Imath::V3f pos(ipos.x, ipos.y, ipos.z);
      Imath::V3f dir = (pos - orig).normalize();
      rt::Ray ray(orig, dir);

      rt::Hit hitrec;
      bool hit = scene->intersect(ray, &hitrec);
      if (not hit) continue;

      int nidx = n.get_nidx();
      int primID = hitrec.primID;
      bool visible = triNodes[primID * 3 + 0] == nidx or
                     triNodes[primID * 3 + 1] == nidx or
                     triNodes[primID * 3 + 2] == nidx;

      if (not visible) {
        const int NTESTS = 6;
        const float L = 1e-4;
        Imath::V3f spos[NTESTS] = {
            Imath::V3f(-L, 0, 0), Imath::V3f(L, 0, 0),  Imath::V3f(0, -L, 0),
            Imath::V3f(0, L, 0),  Imath::V3f(0, 0, -L), Imath::V3f(0, 0, L),
        };

        for (int tidx = 0; not visible and tidx < NTESTS; ++tidx) {
          Imath::V3f pos2(ipos.x + spos[tidx][0], ipos.y + spos[tidx][1],
                          ipos.z + spos[tidx][2]);
          Imath::V3f dir2 = (pos2 - orig);
          rt::Ray ray2(orig, dir2);
          dir2.normalize();

          rt::Hit hitrec2;
          bool hit2 = scene->intersect(ray2, &hitrec2);

          if (not hit2) continue;

          int primID2 = hitrec2.primID;

          visible = triNodes[primID2 * 3 + 0] == nidx or
                    triNodes[primID2 * 3 + 1] == nidx or
                    triNodes[primID2 * 3 + 2] == nidx;
        }
      }  // if not visible

      if (not visible) continue;

      lastWasVisible = visible;

      cv::Point3_<float> tunnelNormal = nrm[nidx];
      Imath::V3f nnrm(tunnelNormal.x, tunnelNormal.y, tunnelNormal.z);

      float cos_theta = nnrm.dot(dir);
      float theta = acos(cos_theta);
      bool forward = theta > obliqueThresh;

      if (not forward) continue;

      // Dump UV coordinate for this model node
      // (normalized image coords)
      //[u0 v0 u1 v1 u2 v2 ... uM vM]
      const float u = pt.x / f_sz.width;
      const float v = pt.y / f_sz.height;
      uv[nidx * 2] = u;
      uv[nidx * 2 + 1] = v;

      // Get the closest pixel
      cv::Point2i rpt(round(pt.x), round(pt.y));

      // Remember this node
      localTriplets[blockIndex].push_back({nidx, idx(rpt), 1.0});

    }  // loop over blocks

  }  // parallel

  // Combine the localTriplets
  for (unsigned int idx = 0; idx < nBlocks; ++idx) {
    triplets.insert(triplets.end(), localTriplets[idx].begin(),
                    localTriplets[idx].end());
  }

  // Populate nodes-per-pixel diagnostic image
  for (const auto& t : triplets) {
    const auto idx = t.col();
    const auto y = idx / f_sz.width;
    const auto x = idx - (y * f_sz.width);
    // nodes-per-pixel values saturate at 8-bits (256).
    // For diagnostics we mostly just care whether
    // it's grossly greater than 1 node per pixel
    const uint32_t count =
        static_cast<int>(number_projected_model_nodes.at<uint8_t>(y, x));
    const uint32_t new_count = count < UINT8_MAX ? count + 1 : count;
    number_projected_model_nodes.at<uint8_t>(y, x) =
        static_cast<uint8_t>(new_count);
  }

  // std::cout << " projected " << count << " model nodes, accepted " <<
  // triplets.size() << std::endl; system ("/usr/bin/date");

  // Fill the sparse matrix with triplets
  smat.setFromTriplets(triplets.begin(), triplets.end());

}  // create_projection_mat

std::string FilenameWithCameraPrefix(int cam, const std::string& name) {
  std::ostringstream oss;
  oss.str("");
  oss << "cam" << std::setfill('0') << std::setw(2) << cam << "-" << name;
  return std::move(oss.str());
}

void write_calibration_check_file(
    const cv::Mat& frame, const std::vector<upsp::Target>& input_targ_list,
    const std::string& out_dir, const std::string& out_name, int cam,
    int count) {
  // copy input targets as add_targets might modify them
  std::vector<upsp::Target> targ_list(input_targ_list);
  cv::Mat img_to_show;
  img_to_show =
      upsp::add_targets(targ_list, frame, cv::Scalar(0, 255, 0), false);

  std::ostringstream oss;
  oss << out_name << std::setfill('0') << std::setw(2) << cam
      << "  #m=" << count;
  cv::putText(img_to_show, oss.str(),
              cv::Point(img_to_show.cols / 8, 7 * img_to_show.rows / 8),
              cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255), 1);

  cv::imwrite(out_dir + "/" + FilenameWithCameraPrefix(cam, "final_cal.png"),
              img_to_show);
}

// Initialize video streams
// - ifile: uPSP input file deck (file paths and configuration parameters)
// - cams: container for video streams. Previous contents will be closed +
// cleared
// - number_frames_to_process: output - number of frames to process based both
//                             on configuration parameters and available frames
//                             in each input video stream.
bool InitializeVideoStreams(const upsp::FileInputs& input_deck,
                        std::vector<upsp::PSPVideo*>& camera_streams,
                        unsigned int& number_frames_to_process,
                        bool printf) {
  enum NumberFramesToProcessEvaluationMethod {
    NUMBER_FRAMES_MAX_AVAILABLE_FOR_ALL_CAMERAS,
    NUMBER_FRAMES_USER_PROVIDED
  } method;

  method = input_deck.number_frames < 0
               ? NUMBER_FRAMES_MAX_AVAILABLE_FOR_ALL_CAMERAS
               : NUMBER_FRAMES_USER_PROVIDED;

  switch (method) {
    case NUMBER_FRAMES_MAX_AVAILABLE_FOR_ALL_CAMERAS:
      number_frames_to_process = std::numeric_limits<unsigned int>::max();
      break;
    case NUMBER_FRAMES_USER_PROVIDED:
      number_frames_to_process = input_deck.number_frames;
      break;
  }

  for (auto stream : camera_streams) {
    delete stream;
  }
  camera_streams.resize(input_deck.cameras);

  for (int ii = 0; ii < input_deck.cameras; ii++) {
    const std::string filename(input_deck.camera_filenames[ii]);
    const auto ext = filename.substr(filename.rfind('.'));
    std::unique_ptr<upsp::VideoReader> reader;
    if (ext.compare(".cine") == 0) {
      reader = std::unique_ptr<upsp::VideoReader>(new upsp::CineReader(filename));
    } else if (ext.compare(".mraw") == 0) {
      reader = std::unique_ptr<upsp::VideoReader>(new upsp::MrawReader(filename));
    } else {
      if (printf) {
        std::cerr << "Unknown video file extension '" << ext << "'"
                  << " for '" << filename << "'."
                  << " Valid extensions: {'.cine', '.mraw'}" << std::endl;
      }
      return false;
    }
    camera_streams[ii] = new upsp::PSPVideo(std::move(reader));

    switch (method) {
      case NUMBER_FRAMES_MAX_AVAILABLE_FOR_ALL_CAMERAS:
        number_frames_to_process =
            std::min(number_frames_to_process, camera_streams[ii]->get_number_frames());
        break;
      case NUMBER_FRAMES_USER_PROVIDED:
        const auto number_frames_available =
            camera_streams[ii]->get_number_frames();
        if (number_frames_available < number_frames_to_process) {
          if (printf) {
            std::cerr << "(" << number_frames_to_process << ") frames"
                      << " requested but only (" << number_frames_available
                      << ") frames available in '" << filename << "'"
                      << std::endl;
          }
          return false;
        }
        break;
    }
    if (printf) {
      const auto& s = camera_streams[ii];
      std::cout << "Initialized video stream ['" << filename << "']" << std::endl;
      std::cout << "  Frames per second : " << s->get_frame_rate() << std::endl;
      std::cout << "  Frame size        : " << s->get_frame_size() << std::endl;
      std::cout << "  Bit depth         : " << s->get_bit_depth() << std::endl;
      const auto typ = s->get_frame(1).type();
      std::cout << "  cv::Mat type      : " << upsp::convert_type_string(typ) << std::endl;
    }
  }
  if (printf) {
    std::cout << "Will process (" << number_frames_to_process << ") frames"
              << std::endl;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// The full solution is an array with dimensions
//     solution_array [number of frames] [number of model nodes]
// and of course the transpose is
//     transpose_array [number of model nodes] [number of frames]
// These arrays are distributed in slices across the MPI ranks.
// Each slice is full-size in the stride-1 dimension, with a
// contiguous subset of the rows.

unsigned long int msize;          // Number of model nodes
unsigned long int number_frames;  // Number of frames
const float qNAN = std::numeric_limits<float>::quiet_NaN();

// Per MPI rank info about the contiguous subset of rows for
// both the solution and the transpose arrays.
// [The special treatment of "my_mpi_rank" allows us to make this
// available in lib/P3DModel.ipp]
#ifndef HAS__MY_MPI_RANK
#define HAS__MY_MPI_RANK
int my_mpi_rank = 0;  // "int" required by the MPI API
#endif
int num_mpi_ranks = 1;  // "int" required by the MPI API
std::vector<int> rank_start_frame;
std::vector<int> rank_num_frames;
std::vector<int> rank_start_node;
std::vector<int> rank_num_nodes;

// This rank's set of input frames (all cameras)
std::vector<cv::Mat> first_frames_raw;
std::vector<cv::Mat> first_frames_32f;
volatile bool first_frames_ready = false;

std::vector<std::vector<cv::Mat>> input_frames;
volatile long int input_frame_offset_ready = -1;

// TODO cleanup globals
// global in order to pass info from phase1() -> phase 2()
std::vector<float> coverage;
std::vector<float> sol_avg_final;
std::vector<float> sol_rms_final;
std::vector<double> rms;
std::vector<double> avg;
std::vector<double> gain;
pthread_t async_thread_id;

struct FileInfo {
  int fd;
  std::string path;
};
std::map<std::string, FileInfo> output_files;

const char* const files_to_create[]{
    "intensity",
    "intensity_transpose",
    "pressure_transpose",
    "intensity_avg", 
    "intensity_rms",
    "intensity_ratio_0",
    "avg",
    "rms",
    "coverage",
    "steady_state",
    "model_temp",
    "X",
    "Y",
    "Z",
    "gain",
};
const int num_filenames = sizeof(files_to_create) / sizeof(char*);

int num_openmp_threads = -1;

// Buffer management:
// Here, we use five large memory buffers.  We could get away with as
// few as two, but then we'd have to do a lot of waiting for a buffer
// to be written out before we could reuse it.  Earlier versions of
// this code used three or four buffers, which can eliminate *nearly*
// all of the waiting, but needed somewhat complicated buffer management.
// With five buffers, we guarentee that we never have to wait for
// buffer space, and the code is simple.  Note however that the local
// memory use is inversely proportional to the number of MPI ranks,
// so if you run out of memory you can just run on more ranks.
// And if you're running out of memory, it means that the problem is
// big, so you probably *want* to run on more ranks anyway.
// [These are global to communicate with the async thread(s).]
long int large_buf_size;
volatile void* ptr_intensity_data = NULL;
volatile void* ptr_intensity_transpose_data = NULL;
volatile void* ptr_pressure_transpose_data = NULL;
volatile void* ptr_temp_workspace = NULL;

// Flags for the async i/o thread.  The main code flags "ready" when
// the data have been put into the associated buffer and are available
// to be written, and the i/o thread flags "written" when the data have
// been written and the associated buffer is available for reuse.
volatile bool intensity_transpose_ready = false;
volatile bool intensity_written = false;
volatile bool intensity_transpose_written = false;
volatile bool pressure_written = false;
// volatile bool pressure_transpose_written = false;

// I'd like to make this "std::vector <volatile bool>" or even
// "volatile std::vector <bool>", but C++ has fits.
volatile bool* intensity_row_ready;
volatile bool* pressure_row_ready;

////////////////////

#include <chrono>

// Profile info.  Note that the "barrier" time is a measure of the
// load imbalance; the MPI_Barrier itself is blazingly fast.
void timedBarrierPoint(bool do_output, const char* label) {
  static double base = -1.0;
  static double previous = -1.0;

  double before_barrier = MPI_Wtime();

  if (base < 0.0) {
    base = before_barrier;
    previous = before_barrier;
  }

  MPI_Barrier(MPI_COMM_WORLD);
  double after_barrier = MPI_Wtime();

  if (do_output) {
    std::cout << "+++ " << label << "   [total elapsed:" << after_barrier - base
              << ",  this thread since previous:" << before_barrier - previous
              << "  barrier (load imbalance): "
              << after_barrier - before_barrier << "]" << std::endl;
  }
  previous = after_barrier;
}

// Divide the range [0 .. (value-1)] into equal-as-possible sized pieces.
// (This sees utility in e.g. computing the assignments of work to ranks
// and/or threads.)
void apportion(int value, int nBins, int* start, int* extent) {
  assert((value >= 0) && (nBins > 0));
  unsigned long int blockSize = value / nBins;
  unsigned long int remainder = value - (blockSize * nBins);

  unsigned long int nextStart = 0;
  unsigned long int curBin;
  for (curBin = 0; curBin < nBins; ++curBin) {
    start[curBin] = nextStart;
    extent[curBin] = blockSize + (curBin < remainder);
    nextStart += extent[curBin];
  }
  assert(nextStart == value);
}

// Like pwrite, except it keeps going after a partial write.
void pwrite_full(int fd, const void* buf, size_t nbytes, off_t file_offset) {
  unsigned char* src = (unsigned char*)buf;
  errno = 0;
  size_t amt_remaining = nbytes;
  while (amt_remaining > 0) {
    long int amt_written = pwrite(fd, src, amt_remaining, file_offset);
    if (amt_written < 0) perror("pwrite_full");
    assert(amt_written > 0);
    amt_remaining -= amt_written;
    file_offset += amt_written;
    src += amt_written;
  }
}

// Cache-friendly transpose of a rectangular array
// Note that "x_extent" is the extent in the src's contiguous direction,
// while "y_extent" is the src's extent in the non-contiguous direction.
// We require that src and dst be disjoint.
// Sadly, "restrict" is not a C++ standard keyword, so we rely on
// the g++ specific hack "__restrict__"
void local_transpose(float* __restrict__ src_ptr, int x_extent, int y_extent,
                     float* __restrict__ dst_ptr) {
  // It's not immediately clear if doing this in parallel is actually
  // any faster.  We are limited by the bandwidth to memory, which we
  // max out pretty quickly.  After that, more threads are likely to
  // just cause more cache interference.  There is probably some sweet-
  // -spot combination of block_size and num threads that gives
  // the best performance.  What I do here is mostly just a WAG; I have
  // not investigated throughly.  But empirically, this is pretty fast.

  int block_size = 100;

  float(*__restrict__ src)[x_extent] = (float(*)[x_extent])src_ptr;
  float(*__restrict__ dst)[y_extent] = (float(*)[y_extent])dst_ptr;

  int full_x_blocks = x_extent / block_size;
  int full_y_blocks = y_extent / block_size;

#pragma omp parallel for collapse(2) schedule(dynamic, 1)
  for (int y_block = 0; y_block < full_y_blocks + 1; ++y_block) {
    for (int x_block = 0; x_block < full_x_blocks + 1; ++x_block) {
      // By moving the "y_block" computations into the inner loop
      // (despite them being inner-loop invariant), we can use
      // "collapse" on the nested loops.
      int y_block_start_index = y_block * block_size;
      int y_block_extent =
          ((y_block < full_y_blocks) ? block_size
                                     : y_extent - (full_y_blocks * block_size));

      int x_block_start_index = x_block * block_size;
      int x_block_extent =
          ((x_block < full_x_blocks) ? block_size
                                     : x_extent - (full_x_blocks * block_size));

      for (int jj = 0; jj < y_block_extent; ++jj) {
        for (int ii = 0; ii < x_block_extent; ++ii) {
          dst[x_block_start_index + ii][y_block_start_index + jj] =
              src[y_block_start_index + jj][x_block_start_index + ii];
        }
      }
    }
  }
}

// Transpose a distributed rectangular array.
// Conceptually, the full input is an array AA[number_frames][msize].
// Each MPI rank has a slice of full-length rows of this conceptual array,
// starting at row "rank_start_frame[my_mpi_rank]", and extending for
// "rank_num_frames[my_mpi_rank]" rows.  We want to transpose this into
// another conceptual distributed array TT[msize][number_frames], with
// each MPI rank have a slice of this array, starting at row
// "rank_start_node[my_mpi_rank]", with "rank_num_nodes[my_mpi_rank]"
// rows.  The local slice of the input array is in "src", while the
// slice of the transpose goes into "dst".
// Note that this is not a general purpose routine: it is specifically
// for floats, and all the interesting information about counts and sizes
// are supplied by global variables.  [It wouldn't be all that hard to
// convert it to be general purpose, but we'd need to pass an unreasonably
// large number of arguments.]

void global_transpose(float* ptr_src, float* ptr_dst) {
  int my_num_frames = rank_num_frames[my_mpi_rank];
  int my_num_nodes = rank_num_nodes[my_mpi_rank];
  MPI_Request requests[num_mpi_ranks];

  float(*src)[msize] = (float(*)[msize])ptr_src;
  float(*dst)[number_frames] = (float(*)[number_frames])ptr_dst;
  float(*temp)[my_num_frames] = (float(*)[my_num_frames])ptr_temp_workspace;

  ///////////////////////////////////////////////////////////////////
  // Transpose the local slice from src into temp
  if (0 == my_mpi_rank) std::cout << "  Local transpose ..." << std::endl;
  local_transpose(&(src[0][0]), msize, my_num_frames, &(temp[0][0]));

  ///////////////////////////////////////////////////////////////////
  // MPI_Isend slices of "temp" to the appropriate ranks.
  if (0 == my_mpi_rank)
    std::cout << "  Global transpose exchange ..." << std::endl;
  for (int rank = 0; rank < num_mpi_ranks; ++rank) {
    long int start_row = rank_start_node[rank];
    long int num_rows = rank_num_nodes[rank];
    assert((num_rows * my_num_frames) < INT_MAX);
    MPI_Isend(&(temp[start_row][0]), num_rows * my_num_frames, MPI_FLOAT, rank,
              0, MPI_COMM_WORLD, &(requests[rank]));
  }

  ///////////////////////////////////////////////////////////////////
  // MPI_Recv slices from the ranks (including ourself), and collect
  // the slices together into full-length rows.
  long int max_count = rank_num_frames[0] * rank_num_nodes[0];
  float* recv_buf = (float*)malloc(max_count * sizeof(float));
  assert(recv_buf != NULL);
  for (int count = 0; count < num_mpi_ranks; ++count) {
    MPI_Status status;
    // Recv a message
    MPI_Recv(recv_buf, max_count, MPI_FLOAT, MPI_ANY_SOURCE, 0, MPI_COMM_WORLD,
             &status);

    // Figure out who it's from, and what size we expect it to be
    int sender_rank = status.MPI_SOURCE;
    int sender_num_frames = rank_num_frames[sender_rank];
    int expected_num_items = sender_num_frames * my_num_nodes;

    // Verify it actually is the size we expected it to be
    int actual_num_items;
    MPI_Get_count(&status, MPI_FLOAT, &actual_num_items);
    assert(actual_num_items == expected_num_items);

    // Copy the recv'd data into the dst buf
    float(*recv_trans)[sender_num_frames] =
        (float(*)[sender_num_frames])recv_buf;
    long int start_col = rank_start_frame[sender_rank];
    for (long int node_offset = 0; node_offset < my_num_nodes; ++node_offset) {
      for (long int frame_offset = 0; frame_offset < sender_num_frames;
           ++frame_offset) {
        dst[node_offset][start_col + frame_offset] =
            recv_trans[node_offset][frame_offset];
      }
    }
  }
  free(recv_buf);

  // Make sure all the messages we sent have gotten out
  MPI_Waitall(num_mpi_ranks, requests, MPI_STATUSES_IGNORE);
}

///////////////////////////////////////////////

// Asynchronous processing
// We use custom multi-threading to off-load some things from the main
// task.  Importantly, but not exclusively, we do i/o in the background
// overlapped with computation.

// Wait for some other thread to set a global flag to non-zero (i.e. "true")
// Sleep for "usec_interval" usecs in between checks of the flag.
#include <sched.h>
void wait_for(void* ptr_flag, int usec_interval) {
  volatile bool* p = (volatile bool*)ptr_flag;
  __sync_synchronize();
  while (not(*p)) {
    if (usec_interval <= 0)
      sched_yield();
    else
      usleep(usec_interval);
    __sync_synchronize();
  }
}

void allocate_global_data(void) {
  // TODO - we should check /proc/meminfo to be sure there's enough
  // Allocate the large data buffers.  The size given to rank 0
  // will be the maximum size.
  int max_num_frames = rank_num_frames[0];
  int max_num_nodes = rank_num_frames[0];

  long int buf_size1 = rank_num_frames[0] * msize * sizeof(float);
  long int buf_size2 = rank_num_nodes[0] * number_frames * sizeof(float);
  large_buf_size = std::max(buf_size1, buf_size2);

  ptr_intensity_data = malloc(large_buf_size);
  assert(ptr_intensity_data != NULL);
  ptr_intensity_transpose_data = malloc(large_buf_size);
  assert(ptr_intensity_transpose_data != NULL);
  ptr_pressure_transpose_data = malloc(large_buf_size);
  assert(ptr_pressure_transpose_data != NULL);
  ptr_temp_workspace = malloc(large_buf_size);
  assert(ptr_temp_workspace != NULL);

  intensity_row_ready =
      (volatile bool*)malloc(max_num_frames * sizeof(volatile bool));
  for (int ii = 0; ii < max_num_frames; ++ii) intensity_row_ready[ii] = false;

  pressure_row_ready =
      (volatile bool*)malloc(max_num_frames * sizeof(volatile bool));
  for (int ii = 0; ii < max_num_frames; ++ii) pressure_row_ready[ii] = false;
}

void open_output_files(const std::string& dir) {
  // Populate output_files paths
  for (int ii = 0; ii < num_filenames; ++ii) {
    std::string filename = dir + "/" + files_to_create[ii];
    output_files[files_to_create[ii]].path = filename;
  }
  // Filesystem-dependent initialization. Only execute once (use main MPI rank)
  if (0 == my_mpi_rank) {
    for (auto& info : output_files) {
      const auto& filename = info.second.path;
      // For Lustre file systems: create the files with lots of stripes
      // (send errors to /dev/null in case it is not a Lustre file system).
      std::string cmd = "/bin/rm -f " + filename;
      cmd += " ; lfs setstripe -c 60 " + filename + " >& /dev/null";
      system(cmd.c_str());
      // Now "creat" the file, in case it was NOT a Lustre file system
      int fd = open(filename.c_str(), O_WRONLY | O_CREAT, 0644);
      assert(fd >= 0);
      close(fd);
    }
  }
  MPI_Barrier(MPI_COMM_WORLD);
  // After filesystem-dependent initialization, populate file descriptors.
  for (auto& info : output_files) {
    info.second.fd = open(info.second.path.c_str(), O_WRONLY, 0644);
    assert(info.second.fd >= 0);
    if (0 == my_mpi_rank) {
      std::cout << "Opened '" << info.second.path << "'" << std::endl;
    }
  }
}

void close_output_files() {
  for (auto& info : output_files) {
    close(info.second.fd);
    if (0 == my_mpi_rank) {
      std::cout << "Closed '" << info.second.path << "'" << std::endl;
    }
  }
}

// Read the frame data from the file.
// Put the data into 'first_frames_raw' and 'input_frames' as appropriate
void* __async_read_ahead(void* arg) {
  std::vector<upsp::PSPVideo*>* ptr_cams = (std::vector<upsp::PSPVideo*>*)arg;

  unsigned int n_cams = ptr_cams->size();

  unsigned long int first_frame_to_read = rank_start_frame[my_mpi_rank];
  unsigned long int num_frames_to_read = rank_num_frames[my_mpi_rank];

  ///////////////////////////////////////////////////////////////////////
  // Read the first frame (reference frame); all cameras
  first_frames_raw.resize(n_cams);
  first_frames_32f.resize(n_cams);
  for (unsigned int c = 0; c < n_cams; ++c) {
    // OpenCV uses 1-based frame indices
    first_frames_raw[c] = ((*ptr_cams)[c])->get_frame(1);
    // TODO
    upsp::fix_hot_pixels(first_frames_raw[c], my_mpi_rank == 0);
    first_frames_raw[c].convertTo(first_frames_32f[c], CV_32F);
  }
  __sync_synchronize();
  first_frames_ready = true;

  ///////////////////////////////////////////////////////////////////////
  // Read my slice of the input frames.  We flag each frame as it
  // becomes ready, so that we can overlap processing of prior frames
  // with the input of subsequent frames.
  input_frames.resize(n_cams);
  for (unsigned int c = 0; c < n_cams; ++c) {
    input_frames[c].resize(num_frames_to_read);
  }
  for (unsigned int f = 0; f < num_frames_to_read; ++f) {
    for (unsigned int c = 0; c < n_cams; ++c) {
      // OpenCV uses 1-based frame indices
      input_frames[c][f] =
          ((*ptr_cams)[c])->get_frame(first_frame_to_read + f + 1);
    }
    __sync_synchronize();
    input_frame_offset_ready = f;
  }

  return NULL;
}

// Write out the local part of a distributed array, to a collective file.
// Write the rows as we see them being computed.
// As written, this code only works for arrays of shape
//  (float (*)[msize])
// i.e. 'intensity' and 'pressure'.
void write_behind(void* data, volatile bool* ready_flags, int fd) {
  int start_write_row = 0;
  int num_write_rows = 0;
  int num_remaining = rank_num_frames[my_mpi_rank];
  long int file_offset = rank_start_frame[my_mpi_rank] * msize * sizeof(float);
  unsigned char* start_addr = (unsigned char*)data;

  while (num_remaining > 0) {
    __sync_synchronize();
    wait_for((void*)&(ready_flags[start_write_row]), 1000000);

    // Write the next block of contiguous rows that are ready
    do {
      if (++num_write_rows >= num_remaining) break;
      __sync_synchronize();
    } while (ready_flags[start_write_row + num_write_rows]);
    assert(num_write_rows <= num_remaining);

    long int nbytes = num_write_rows * msize * sizeof(float);
    pwrite_full(fd, start_addr, nbytes, file_offset);

    file_offset += nbytes;
    start_addr += nbytes;
    start_write_row += num_write_rows;
    num_remaining -= num_write_rows;
    num_write_rows = 0;
  }
}

void* __async_write_behind(void* arg) {
  write_behind((void*)ptr_intensity_data, intensity_row_ready,
               output_files["intensity"].fd);
  if (0 == my_mpi_rank)
    std::cerr << "## Async I/O :  'intensity' written" << std::endl;

  return NULL;
}

// Write out the local part of a distributed array, to a collective file.
// Write the entire local part as a single piece.
// As written, this code only works for arrays of shape
//  (float (*)[number_frames])
// i.e. 'intensity_transpose' and 'pressure_transpose'.
void write_block(void* data, int fd) {
  long int nbytes = number_frames * rank_num_nodes[my_mpi_rank] * sizeof(float);
  long int file_offset =
      number_frames * rank_start_node[my_mpi_rank] * sizeof(float);
  pwrite_full(fd, data, nbytes, file_offset);
}

// Procedure args for __asynch_thread
typedef struct {
  std::vector<upsp::PSPVideo*>* ptr_cams;
  std::string add_out_dir;
} _asynch_info_t;
_asynch_info_t _asynch_info;

// The main async thread.  Certain bits of work (esp. reading in the
// frame data, and writing the results) are assigned to extra threads,
// so this work can proceed in parallel with the critical-path computations.
// This is custom ad-hoc stuff; there are no particular rules for assigning
// work to threads.
void* __asynch_thread(void* arg) {
  int status;
  pthread_t tid;
  std::vector<pthread_t> thread_ids;

  _asynch_info_t* p = (_asynch_info_t*)arg;

  status = pthread_create(&tid, NULL, __async_read_ahead, p->ptr_cams);
  assert(0 == status);
  thread_ids.push_back(tid);

  // status = pthread_create (&tid, NULL, __async_write_behind, NULL);
  // assert (0 == status);
  // thread_ids.push_back(tid);

  // Rather than create yet-another thread for this, and then
  // pthread_join that thread, we just do it ourselves.
  wait_for((void*)&intensity_transpose_ready, 100000);
  write_block((void*)ptr_intensity_transpose_data,
              output_files["intensity_transpose"].fd);
  if (0 == my_mpi_rank)
    std::cerr << "## Async I/O :  'intensity_transpose' written" << std::endl;

  // Make sure all the other threads have finished, and clean-up
  for (int thrd = 0; thrd < thread_ids.size(); ++thrd) {
    int status = pthread_join(thread_ids[thrd], NULL);
    assert(0 == status);
  }

  return NULL;
}

////////////////////

struct Phase1Settings {
  Phase1Settings(upsp::FileInputs i_ifile)
      : code_version(""), add_out_dir(""), ifile(i_ifile) {
    // Prevent calibration from modifying higher order terms
    // also assume focal length is already well characterized
    int cal_options = cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 |
                      cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6 |
                      cv::CALIB_FIX_FOCAL_LENGTH;
  }

  // Just perform the checkout (phase 0)
  bool checkout;

  // Input file object
  upsp::FileInputs ifile;

  // fiducial patching
  unsigned int bound_thickness;
  unsigned int buffer_thickness;
  float target_diam_sf;

  // camera calibration
  int cal_options;

  // data storage
  unsigned int trans_nodes;

  // parameters for inclusion in H5
  std::string code_version;

  // directory for additional outputs
  std::string add_out_dir;

  // don't project to anything beyond this x-value (model coordinates)
  bool has_x_max;
  float x_max;
};

/* Store objects used in phase 1 process */
template <typename M, typename Patcher>
struct Phase1Elements {
  typedef M Model;

  Phase1Elements() : model(nullptr) {}

  ~Phase1Elements() {
    delete model;
    for (unsigned int i = 0; i < cams.size(); ++i) {
      delete cams[i];
    }
    for (unsigned int i = 0; i < patchers.size(); ++i) {
      delete patchers[i];
    }
  }

  // Grid to project the images onto
  Model* model;

  // Bounding Volume Hiearchy (BVH) containing the model
  // (for camera ray-tracing)
  std::shared_ptr<rt::BVH> scene;
  std::vector<int> triNodes;

  // Opened video file streams
  std::vector<upsp::PSPVideo*> cams;

  // Camera calibrations
  std::vector<upsp::CameraCal> cals;

  // Camera settings
  upsp::CameraSettings camera_settings;

  // Fiducial patchers
  std::vector<Patcher*> patchers;

  // The first frame from each camera
  std::vector<cv::Mat> first_frames;

  // Projection matrices from camera to model
  std::vector<Eigen::SparseMatrix<float, Eigen::RowMajor>> projs;
};

struct Phase2Settings {
  Phase2Settings(upsp::FileInputs i_ifile)
      : code_version(""),
        r(0.896),
        gamma(1.4),
        F_to_R(459.67),
        degree(6),
        grid_tol(1e-3),
        grid_units("-"),
        k(10),
        p(2.0),
        ifile(i_ifile) {}

  // data storage
  unsigned int trans_nodes;

  // Input file object
  upsp::FileInputs ifile;

  // wind off data does not have steady psp data
  bool wind_off;

  // read model_temp_p3d 
  bool read_model_temp;

  // parameters for inclusion in H5
  std::string code_version;

  // parameters for psp gain calculation
  const float r;       // turbulent boundary-layer recovery factor
  const float gamma;   // ratio of specific heats
  const float F_to_R;  // offset between deg F and deg R

  // degree of polynomial fit
  unsigned int degree;

  // structured grid tolerance for overlapping nodes
  float grid_tol;

  // grid units (in) or (ft)
  std::string grid_units;

  // settings for unstructured grid steady psp interpolation
  unsigned int k;  // number of points for inverse distance
  unsigned int p;  // exponent for inverse distance
};

// Files (both input and output) used during Phase 2
struct Phase2Files {
  std::string add_out_dir;
  std::string h5_out;
  std::string h5_out_extra;
  std::string steady_p3d;
  std::string steady_grid;
  std::string paint_cal;
  std::string model_temp_p3d;
};

/* Store objects used in phase 2 process */
template <typename M>
struct Phase2Elements {
  typedef M Model;
  Phase2Elements() : model(nullptr) {}
  ~Phase2Elements() {}
  Model* model;
  upsp::CameraSettings camera_settings;
  upsp::TunnelConditions tunnel_conditions;
};

struct Settings {
  Settings(upsp::FileInputs i_ifile):
    phase1(i_ifile), phase2(i_ifile) {}
  Phase1Settings phase1;
  Phase2Settings phase2;
  Phase2Files phase2files;
};

void imwrite(
  const cv::Mat& img, const Phase1Settings& sett,
  int cam, const std::string& name
) {
  const auto fn = sett.add_out_dir + "/" + FilenameWithCameraPrefix(cam + 1, name);
  cv::imwrite(fn, img);
};

/* Perform pre-processing steps for each camera
 * 1. update camera calibration
 * 2. create patchers for fiducial marks
 */
template <typename P1Elems>
int phase0(Phase1Settings& sett, P1Elems& elems);

template <typename P1Elems>
int phase1(Phase1Settings& sett, P1Elems& elems);

template <typename P2Elems>
int phase2(Phase2Settings& sett, Phase2Files& p2_files, P2Elems& elems);

// Returns valid settings on success, else nullptr
std::shared_ptr<Settings> ParseOpts(int argc, char **argv) {
  const cv::String keys =
      "{help h usage ? |     | print this message   }"
      "{input_file     |     | full input deck }"
      "{frames         |     | override input_file number of frames }"
      "{code_version   |  XYZ | version of the repo (deprecated)}"
      // todo-mshawlec: trans_nodes effectively unused (used to control
      // writing of large pressure-time history solutions to HDF5; now,
      // the pressure-time histories are written to separate binary flat
      // files to enable multi-process I/O, since we don't currently build
      // HDF5 for MPI parallelization). Preserving for backwards compatibility.
      "{trans_nodes    |  250   | number of nodes per chunk in transposed solution }"
      "{add_out_dir    |     | output directory for any additional debugging files "
      "(default=input deck-specified output directory) }"
      // Phase 1-specific settings
      "{checkout       |  F  | perform calibration update checkout }"
      "{bound_pts      |  2  | thickness of cluster boundary}"
      "{buffer_pts     |  1  | thickness of buffer between targets and cluster boundary}"
      "{target_diam_sf |  1.2  | scale factor to apply onto target diameter}"
      "{cutoff_x_max   |     | ignore nodes beyond this value in projection }"
      // Phase 2-specific settings
      "{h5_out             |     | output hdf5 file }"
      "{steady_p3d         |     | steady state p3d function file (for wind-on); units of Cp }"
      "{steady_grid        |     | steady state p3d grid (needed for wind-on unstructured) }"
      "{paint_cal          |     | unsteady gain paint calibration file }"
      "{model_temp_p3d     |     | temperature p3d function file; units of Temperature degrees F }"
      ;
  cv::CommandLineParser parser(argc, argv, keys);
  parser.about(
    "Process Unsteady Pressure-Sensitive Paint (uPSP) video files "
    "into pressure-time history on model surface grid"
  );

  if (parser.has("help")) {
    if (0 == my_mpi_rank) parser.printMessage();
    return nullptr;
  }

  if (!parser.has("input_file")) {
    LOG_ERROR("Must specify -input_file");
    return nullptr;
  }

  if (!parser.has("h5_out")) {
    LOG_ERROR("Must specify -h5_out");
    return nullptr;
  }

  if (!parser.has("paint_cal")) {
    LOG_ERROR("Must specify -paint_cal");
    return nullptr;
  }

  auto input_filename = parser.get<std::string>("input_file");
  upsp::FileInputs input_file;
  if (!input_file.Load(input_filename)) {
    return nullptr;
  }
  auto settings = std::make_shared<Settings>(input_file);

  auto& p1s = settings->phase1;
  p1s.bound_thickness = parser.get<unsigned int>("bound_pts");
  p1s.buffer_thickness = parser.get<unsigned int>("buffer_pts");
  p1s.target_diam_sf = parser.get<float>("target_diam_sf");
  p1s.trans_nodes = parser.get<unsigned int>("trans_nodes");
  p1s.checkout = parser.get<bool>("checkout");
  p1s.code_version = parser.get<std::string>("code_version");
  p1s.add_out_dir = parser.has("add_out_dir") ? parser.get<std::string>("add_out_dir"): p1s.ifile.out_dir;
  p1s.has_x_max = parser.has("cutoff_x_max");
  p1s.x_max = p1s.has_x_max ? parser.get<float>("cutoff_x_max"): 0.;
  p1s.ifile.number_frames = parser.has("frames") ? parser.get<int>("frames") : p1s.ifile.number_frames;

  auto& p2s = settings->phase2;
  auto& p2f = settings->phase2files;
  p2f.h5_out = parser.get<std::string>("h5_out");
  p2f.paint_cal = parser.get<std::string>("paint_cal");
  p2f.steady_p3d = parser.get<std::string>("steady_p3d");
  p2f.model_temp_p3d = parser.get<std::string>("model_temp_p3d");
  p2f.steady_grid = parser.get<std::string>("steady_grid");
  p2f.add_out_dir = parser.has("add_out_dir") ? parser.get<std::string>("add_out_dir"): p2s.ifile.out_dir;
  p2f.h5_out_extra = p2f.add_out_dir + "/" + "extras.h5";
  p2s.trans_nodes = parser.get<unsigned int>("trans_nodes");
  p2s.code_version = parser.get<std::string>("code_version");
  p2s.wind_off = p2f.steady_p3d.empty() ? true: false;
  p2s.grid_units = p2s.ifile.grid_units;
  p2s.read_model_temp = p2f.model_temp_p3d.empty() ? false: true;

  if (!parser.check()) {
    parser.printErrors();
    return nullptr;
  }

  if (!p1s.ifile.check_all()) {
    return nullptr;
  }
  if (p1s.ifile.tunnel != "ames_unitary") {
    LOG_ERROR("Unrecognized tunnel name '%s'", p1s.ifile.tunnel);
    return nullptr;
  }
  if ((p1s.ifile.registration != upsp::RegistrationType::None) &&
      (p1s.ifile.registration != upsp::RegistrationType::Pixel)) {
    LOG_ERROR("Unsupported registration type");
    return nullptr;
  }
  if (p1s.ifile.filter_size % 2 == 0) {
    LOG_ERROR("Filter size must be odd (currently '%d')", p1s.ifile.filter_size);
    return nullptr;
  }

  bool grid_is_structured;
  switch(p1s.ifile.grid_type) {
    case upsp::GridType::None:
      LOG_ERROR("Unsupported grid type");
      return nullptr;
    case upsp::GridType::P3D:
      grid_is_structured = true;
      break;
    case upsp::GridType::Tri:
      grid_is_structured = false;
      break;
  }

  // steady_p3d files is only required for wind_on data, wind_off data
  // uses tunnel static pressure as reference
  if (!grid_is_structured && p2f.steady_grid.empty()) {
    LOG_ERROR("Must specify steady psp grid for wind-on unstructured grid data");
    return nullptr;
  }

  if (0 == my_mpi_rank) std::cout << p1s.ifile << std::endl;
  timedBarrierPoint(0 == my_mpi_rank, "Load and validate input file and command line options");

  return settings;
}

template <typename Model>
int RunAllPhases(std::shared_ptr<Settings>, Model*);

int main(int argc, char* argv[]) {
  // Note that my_mpi_rank and num_mpi_ranks are global
  MPI_Init(&argc, &argv);
  MPI_Comm_rank(MPI_COMM_WORLD, &my_mpi_rank);
  MPI_Comm_size(MPI_COMM_WORLD, &num_mpi_ranks);
  // todo-mshawlec: by default, enable all log messages, including
  // debugs. If logs get too spammy, consider using environment variable
  // to allow user to control it.
  if (my_mpi_rank == 0) LogSetLevel(LOG_LEVEL_DEBUG);
  else LogSetLevel(LOG_LEVEL_SUPPRESS);

  // If we have the run script issue a similar "date" command prior
  // to invoking this executable, we can get a reasonably accurate
  // handle on how long the MPI startup takes
  if (0 == my_mpi_rank) system("date --rfc-3339=ns");

  timedBarrierPoint(0 == my_mpi_rank, "Begin");

  Eigen::initParallel();
  cv::setNumThreads(4);
  omp_set_nested(false);
  num_openmp_threads = omp_get_max_threads();

  if (0 == my_mpi_rank)
    std::cout << num_mpi_ranks << " MPI ranks, each with " << num_openmp_threads
              << " OpenMP threads" << std::endl;

  rank_start_frame.resize(num_mpi_ranks);
  rank_num_frames.resize(num_mpi_ranks);
  rank_start_node.resize(num_mpi_ranks);
  rank_num_nodes.resize(num_mpi_ranks);

  auto psett = ParseOpts(argc, argv);
  if (!psett) {
    LOG_ERROR("Failed to validate command line options; aborting");
    MPI_Finalize();
    return 1;
  }

  // Call the processing steps.
  // The entire processing algorithm is templated
  // based on the model grid type.
  int res;
  const auto& grid_type = psett->phase1.ifile.grid_type;
  const auto& grid_filename = psett->phase1.ifile.grid;
  switch(grid_type) {
    case upsp::GridType::P3D:
    {
      auto model = new upsp::P3DModel_<float>(grid_filename, 1e-3);
      timedBarrierPoint(0 == my_mpi_rank, "set up P3DModel");
      return RunAllPhases(psett, model);
    }
    case upsp::GridType::Tri:
    {
      auto model = new upsp::TriModel_<float>(grid_filename);
      timedBarrierPoint(0 == my_mpi_rank, "set up TriModel");
      return RunAllPhases(psett, model);
    }
  }
}

template <typename Model>
int RunAllPhases(std::shared_ptr<Settings> psett, Model* model) {
  typedef upsp::PatchClusters<float> Patcher;
  Phase1Elements<Model, Patcher> p1elems;
  p1elems.model = model;
  int res = phase1(psett->phase1, p1elems);
  if (res) {
    std::cerr << "Error during phase 1 processing, aborting" << std::endl;
    MPI_Finalize();
    return res;
  }
  if (psett->phase1.checkout) {
    MPI_Finalize();
    return 0;
  }
  MPI_Barrier(MPI_COMM_WORLD);
  Phase2Elements<Model> p2elems;
  p2elems.model = model;
  p2elems.camera_settings = p1elems.camera_settings;
  res = phase2(psett->phase2, psett->phase2files, p2elems);
  if (res) {
    std::cerr << "Error during phase 2 processing, aborting" << std::endl;
    MPI_Finalize();
    return res;
  }

  // Wait for all the async threads to complete
  pthread_join(async_thread_id, NULL);

  close_output_files();

  free((void*)ptr_intensity_transpose_data);
  free((void*)ptr_pressure_transpose_data);
  free((void*)pressure_row_ready);
  free((void*)intensity_row_ready);
  free((void*)ptr_temp_workspace);

  if (0 == my_mpi_rank) {
    std::cout << "Wait for all ranks to complete" << std::endl;
  }
  timedBarrierPoint(0 == my_mpi_rank, "All done");

  MPI_Finalize();
  return 0;
}

/*****************************************************************************/
template <typename P1Elems>
int phase1(Phase1Settings& sett, P1Elems& elems) {
  typedef typename P1Elems::Model Model;

  // For brevity, create references
  upsp::FileInputs& ifile = sett.ifile;
  Model& model = *elems.model;
  auto& cams = elems.cams;

  // Mark non-data points (points with x > x_max if requested)
  if (!sett.checkout) {
    if (sett.has_x_max) {
      std::vector<float> xs = model.get_x();
      for (unsigned int i = 0; i < xs.size(); ++i) {
        if (xs[i] > sett.x_max) {
          model.set_node_nondata(i);
        }
      }
    }
  }

  // Also mark non-data points according to component numbers
  if (!ifile.active_comps.empty()) {
    // load in the active components
    std::unordered_map<int, bool> active_comps =
        upsp_files::read_active_comp_file(ifile.active_comps);

    // Check that a valid number of components are in the file
    if (active_comps.size() > model.number_of_components()) {
      std::cerr << "Error: Number of components in active component file";
      std::cerr << " cannot be greater than the number of components in";
      std::cerr << " the grid" << std::endl;
      return 1;
    }

    // for each node, if its primary component is inactive, the
    // node is a non-data node
    for (auto n_it = model.cnode_begin(); n_it != model.cnode_end(); ++n_it) {
      auto n = *n_it;
      if (n.has_primary_component()) {
        int comp = n.get_primary_component();

        if (active_comps.find(comp) != active_comps.end()) {
          if (!active_comps[comp]) {
            model.set_node_nondata(n.get_nidx());
          }
        }
      }
    }
  }

  // Load cameras,
  // N.B.: "number_frames" is a global
  unsigned int number_frames_to_process;
  if (!InitializeVideoStreams(ifile, cams, number_frames_to_process, my_mpi_rank == 0)) {
    std::cerr << "Failed to initialize video streams, exiting" << std::endl;
    return 1;
  }
  number_frames = number_frames_to_process;
  timedBarrierPoint(0 == my_mpi_rank, "Load cameras");

  //////////////////////////////////////////////////////////////////////
  // At this point, we know the number of frames (number_frames), and
  // the number of model nodes (phase1Model.size()).  So we take a slight
  // pause from the processing to figure out exactly what the work
  // assignment is for this MPI rank.  This allows us to immediately
  // launch an asynchronous read of this rank's frames from the camera
  // frame input files, to overlap that i/o with the remaining setup
  // that follows.
  //
  // We also allocate the large data arrays.
  //
  // We assume that we are able to hold all these pieces in memory.
  // If you run out of memory, re-launch the job with more MPI ranks
  // spread across more compute nodes.
  // ToDo - check /proc/meminfo to verify that we really do have enough
  // memory, rather than OOM'ing the compute node.
  //

  msize = model.size();  // 'msize' is a global

  // Calculate the frames assigned to each mpi rank
  apportion(number_frames, num_mpi_ranks, &(rank_start_frame[0]),
            &(rank_num_frames[0]));
  int my_first_frame = rank_start_frame[my_mpi_rank];
  int my_num_frames = rank_num_frames[my_mpi_rank];

  // Calculate the node points assigned to each mpi rank
  // (i.e. rows in the distributed "transpose" array)
  apportion(msize, num_mpi_ranks, &(rank_start_node[0]), &(rank_num_nodes[0]));
  int my_first_node = rank_start_node[my_mpi_rank];
  int my_num_nodes = rank_num_nodes[my_mpi_rank];

  // Set up things the async threads will need
  allocate_global_data();
  open_output_files(sett.add_out_dir);
  timedBarrierPoint(0 == my_mpi_rank, "opened output files");

  // Launch the async thread(s).  This performs a variety of work in
  // the background. In particular, doing read-ahead of "input_frames".
  _asynch_info.ptr_cams = &cams;
  _asynch_info.add_out_dir = sett.add_out_dir;
  int status =
      pthread_create(&async_thread_id, NULL, __asynch_thread, &_asynch_info);
  assert(0 == status);

  if (!sett.checkout && 0 == my_mpi_rank) {
    std::cout << "opening hdf5" << std::endl;

    std::cout << "writing X, Y, Z flat files" << std::endl;
    const std::vector<float>& xpos = model.get_x();
    pwrite_full(output_files["X"].fd, &(xpos[0]), xpos.size() * sizeof(float),
                0);
    const std::vector<float>& ypos = model.get_y();
    pwrite_full(output_files["Y"].fd, &(ypos[0]), ypos.size() * sizeof(float),
                0);
    const std::vector<float>& zpos = model.get_z();
    pwrite_full(output_files["Z"].fd, &(zpos[0]), zpos.size() * sizeof(float),
                0);
  }

  // Wait until the first_frames_* data are available
  // (read in by an asynchronous thread).
  __sync_synchronize();
  if (not first_frames_ready) {
    std::cout << "Rank " << my_mpi_rank
              << " waiting for first_frames_* data ..." << std::endl;
    while (not first_frames_ready) __sync_synchronize();
    std::cout << "Rank " << my_mpi_rank << " GOT first_frames_* data ..."
              << std::endl;
  }

  // Setup Camera Calibrations and Target Patching for each camera
  if (phase0(sett, elems)) {
    std::cerr << "ERROR DURING PHASE 0" << std::endl;
    return 1;
  }

  // If checkout, processing is complete
  if (sett.checkout) {
    return 0;
  }

  auto& camsets = elems.camera_settings;
  for (unsigned int c = 0; c < ifile.cameras; ++c) {
    camsets.focal_lengths.push_back(elems.cals[c].get_focal_length());
    camsets.cam_nums.push_back(ifile.cam_nums[c]);
  }
  camsets.framerate = elems.cams[0]->get_frame_rate();
  camsets.fstop = elems.cams[0]->get_aperture();
  camsets.exposure = elems.cams[0]->get_exposure();

  // Generate projection matrices
  elems.projs.resize(ifile.cameras);

  if (0 == my_mpi_rank) {
    std::cout << "Creating projection matricies, " << num_openmp_threads
              << " OpenMP threads" << std::endl;
  }
  for (unsigned int c = 0; c < ifile.cameras; ++c) {
    cv::Size sz = first_frames_raw[c].size();
    cv::Mat_<uint8_t> pixel_node_counts(sz, 0);
    psp::BlockTimer bt(0 == my_mpi_rank, "Calculating projection matrix");
    // TODO refactor this function to projection.ipp
    const float obliqueThresh = deg2_rad(180. - ifile.oblique_angle);
    std::vector<float> uv;
    create_projection_mat(model, elems.cals[c], elems.scene, elems.triNodes,
                          obliqueThresh, elems.projs[c], uv, pixel_node_counts);

    if (0 == my_mpi_rank) {
      { // Dump pixel-to-grid-node-counts diagnostic image to file
        const auto filename = sett.add_out_dir + "/" +
                              FilenameWithCameraPrefix(c + 1, "nodecount.png");
        cv::Mat out_img;
        upsp::nodes_per_pixel_colormap(pixel_node_counts, out_img);
        cv::imwrite(filename, out_img);
      }
      { // Dump UV coordinates to flat file
        const auto filename = sett.add_out_dir + "/" +
                              FilenameWithCameraPrefix(c + 1, "uv");
        FILE *fp = fopen(filename.c_str(), "wb");
        fwrite((void*) &uv[0], sizeof(float), uv.size(), fp);
      }
    }
  }

  // Get camera centers
  std::vector<cv::Point3d> centers(ifile.cameras);
  for (unsigned int c = 0; c < ifile.cameras; ++c) {
    centers[c] = elems.cals[c].get_cam_center();
  }

  // Modify the projection matrices by the weights of each camera on each node
  // Select weighting operator based on user input
  if (ifile.overlap == upsp::OverlapType::BestView) {
    upsp::BestView<float> bv;
    upsp::adjust_projection_for_weights<Model, float>(model, centers,
                                                      elems.projs, bv);
  } else {
    upsp::AverageViews<float> av;
    upsp::adjust_projection_for_weights<Model, float>(model, centers,
                                                      elems.projs, av);
  }

  // Identify nodes without solutions in the combined solution
  std::vector<unsigned int> skipped;
  upsp::identify_skipped_nodes(elems.projs, skipped);

  // Process frames

  ////////////////////////////////////////////////////////////
  // Process the *first* frame (only), to get the reference.
  // Put the solution into "sol1"
  // (Note that we've already checked for first_frames_ready.)
  // All MPI ranks do this (rather than have one do it and
  // broadcast the result).
  if (0 == my_mpi_rank) std::cout << "Processing first frame" << std::endl;
  std::vector<float> sol1;
  {
    for (unsigned int c = 0; c < ifile.cameras; ++c) {
      // cv::Mat img = cams[c]->get_frame(1); // 1-based indices
      cv::Mat img = first_frames_raw[c].clone();

      // register the image to the first frame
      if (ifile.registration == upsp::RegistrationType::Pixel) {
        cv::Mat warp_matrix;
        const int max_iters = 50;
        const double epsilon = 0.001;
        int interpolation_flags = 0;
        switch(ifile.pixel_interpolation) {
          case upsp::PixelInterpolationType::Linear:
            interpolation_flags = cv::INTER_LINEAR;
            break;
          case upsp::PixelInterpolationType::Nearest:
            interpolation_flags = cv::INTER_NEAREST;
            break;
        }
        img = upsp::register_pixel(
          first_frames_32f[c], img, warp_matrix,
          max_iters, epsilon, interpolation_flags
        );
      }

      // patch frame
      if (ifile.target_patcher == upsp::TargetPatchType::Polynomial) {
        img = elems.patchers[c]->operator()(img);
      }

      // apply filter
      if (ifile.filter == upsp::FilterType::Gaussian) {
        GaussianBlur(img, img, cv::Size(ifile.filter_size, ifile.filter_size),
                     0);
      } else if (ifile.filter == upsp::FilterType::Box) {
        cv::blur(img, img, cv::Size(ifile.filter_size, ifile.filter_size));
      }

      // project onto the model
      std::vector<float> c_sols;
      upsp::project_frame(elems.projs[c], img, c_sols);

      // combine solutions
      if (c == 0) {
        sol1.assign(c_sols.begin(), c_sols.end());
      } else {
        std::transform(sol1.begin(), sol1.end(), c_sols.begin(), sol1.begin(),
                       std::plus<float>());
      }
    }

    // add NaNs to missing points in the solution
    for (unsigned int i = 0; i < skipped.size(); ++i) {
      sol1[skipped[i]] = qNAN;
    }

    model.adjust_solution(sol1);
  }
  ////////////////////////////////////////////////////////////

  // The original version computes rms and avg in single precision.
  // Since we are now doing things in parallel, there is a lot more
  // opportunity for round-off artifacts.  Further, we now delay
  // normalizing the result (i.e. dividing by number_frames) until
  // the end, rather than doing it to each term as we go along.
  // So we compute the intermediate results in double, and truncate
  // (and normalize) the result to float at the last minute.
  // We can of course still get numerical artifacts, but hopefully fewer.
  std::vector<double> sol_rms_partial(msize, 0.0);
  std::vector<double> sol_avg_partial(msize, 0.0);

  // Not needed for correctness; just makes things look better
  MPI_Barrier(MPI_COMM_WORLD);

  if (0 == my_mpi_rank) {
    std::cout << "\nNum frames: " << number_frames << ", model size: " << msize
              << std::endl;
    std::cout << "\nProcessing frames" << std::endl;
    std::cout << "  Rank 0:: first frame: " << my_first_frame
              << ",  num frames: " << my_num_frames << std::endl;
  }

  // A convenient alias
  float(*intensity_buf)[msize] = (float(*)[msize])ptr_intensity_data;

  timedBarrierPoint(0 == my_mpi_rank, "phase1: ready to process frames");

#pragma omp parallel
  {
    std::vector<double> local_sol_rms(msize, 0.0);
    std::vector<double> local_sol_avg(msize, 0.0);
    cv::Mat img;

// We use a chunk-size of 1 in the "schedule" clause because the input
// frames are being read (asynchronously) in order from the begining,
// and we want to start processing asap (the input of the later frames
// can overlap with the processing of the earlier frames).
#pragma omp for schedule(dynamic, 1) nowait
    for (unsigned int offset = 0; offset < my_num_frames; ++offset) {
      unsigned int f = my_first_frame + offset;

      // Wait until the frame data are available
      // (read in by an asynchronous thread).
      __sync_synchronize();
      if (input_frame_offset_ready < offset) {
        std::cout << " waiting for input frame " << f << std::endl;
        while (input_frame_offset_ready < offset) __sync_synchronize();
        std::cout << " frame " << f << " ready" << std::endl;
      }

      if ((0 == my_mpi_rank) && ((offset % 100) == 0)) {
        std::cout << "  Rank 0:: processing frame " << f << std::endl;
      }

      std::vector<float> sol;
      for (unsigned int c = 0; c < ifile.cameras; ++c) {
        upsp::fix_hot_pixels((input_frames[c])[offset]);
        (input_frames[c])[offset].copyTo(img);
        assert(!img.empty());

        // register the image to the first frame
        if ((f > 0) && (ifile.registration == upsp::RegistrationType::Pixel)) {
          cv::Mat warp_matrix;
          const int max_iters = 50;
          const double epsilon = 0.001;
          int interpolation_flags = 0;
          switch(ifile.pixel_interpolation) {
            case upsp::PixelInterpolationType::Linear:
              interpolation_flags = cv::INTER_LINEAR;
              break;
            case upsp::PixelInterpolationType::Nearest:
              interpolation_flags = cv::INTER_NEAREST;
              break;
          }
          img = upsp::register_pixel(
            elems.first_frames[c], img, warp_matrix,
            max_iters, epsilon, interpolation_flags
          );
        }

        // patch frame
        if (ifile.target_patcher == upsp::TargetPatchType::Polynomial) {
          img = elems.patchers[c]->operator()(img);
        }

        // apply filter
        if (ifile.filter == upsp::FilterType::Gaussian) {
          GaussianBlur(img, img, cv::Size(ifile.filter_size, ifile.filter_size),
                       0);
        } else if (ifile.filter == upsp::FilterType::Box) {
          cv::blur(img, img, cv::Size(ifile.filter_size, ifile.filter_size));
        }

        // project onto the model
        std::vector<float> c_sols;
        upsp::project_frame(elems.projs[c], img, c_sols);

        // combine solutions
        if (c == 0) {
          sol.assign(c_sols.begin(), c_sols.end());
        } else {
          std::transform(sol.begin(), sol.end(), c_sols.begin(), sol.begin(),
                         std::plus<float>());
        }
      }

      // add NaNs to missing points in the solution
      for (unsigned int i = 0; i < skipped.size(); ++i) {
        sol[skipped[i]] = std::numeric_limits<float>::quiet_NaN();
      }

      // add to rms and avg
      for (unsigned int i = 0; i < model.size(); ++i) {
        local_sol_rms[i] += (sol[i] * sol[i]);
        local_sol_avg[i] += sol[i];
      }

      if (ifile.grid_type == upsp::GridType::P3D) {
        model.adjust_solution(sol);
      }

      for (unsigned int ii = 0; ii < msize; ++ii) {
        intensity_buf[offset][ii] = sol[ii];
      }

      __sync_synchronize();
      intensity_row_ready[offset] = true;
    }

// Combine the thread partials
#pragma omp critical
    for (unsigned int i = 0; i < msize; ++i) {
      sol_rms_partial[i] += local_sol_rms[i];
      sol_avg_partial[i] += local_sol_avg[i];
    }
  }

  if (0 == my_mpi_rank) {
    std::cout << "Wait for all ranks to complete frame processing" << std::endl;
  }

  timedBarrierPoint(0 == my_mpi_rank, "phase1: process frames complete");

  MPI_Barrier(MPI_COMM_WORLD);
  if (0 == my_mpi_rank) {
    std::cout << "Frame processing complete" << std::endl;
  }

  if (0 == my_mpi_rank)
    std::cout << "Global reduction of rms and avg .." << std::endl;
  void* sendbuf =
      (void*)((0 == my_mpi_rank) ? MPI_IN_PLACE : &(sol_avg_partial[0]));
  MPI_Reduce(sendbuf, &(sol_avg_partial[0]), sol_avg_partial.size(), MPI_DOUBLE,
             MPI_SUM, 0, MPI_COMM_WORLD);
  sendbuf = (void*)((0 == my_mpi_rank) ? MPI_IN_PLACE : &(sol_rms_partial[0]));
  MPI_Reduce(sendbuf, &(sol_rms_partial[0]), sol_rms_partial.size(), MPI_DOUBLE,
             MPI_SUM, 0, MPI_COMM_WORLD);
  if (0 == my_mpi_rank) std::cout << "Global reduction complete" << std::endl;

  // TODO remove, or conditionally enable, writing of
  // intensity transpose to HDF5.
  // bool write_intensity_transpose_hdf5 = true;
  // if (0 == my_mpi_rank && write_intensity_transpose_hdf5) {
  //    std::cout << "WRITING INTENSITY TRANSPOSE TO HDF5" << std::endl;

  //    unsigned int sol_per_block = h5t->get_chunk_length();
  //    std::cout << "  " << sol_per_block << " frames per transpose block" <<
  //    std::endl; std::vector<float> trans_sol(model.size()*sol_per_block);
  //    unsigned int trans_adjust_frame = number_frames;
  //    if ( number_frames % sol_per_block != 0) {
  //        trans_adjust_frame = (number_frames / sol_per_block ) *
  //        sol_per_block;
  //    }
  //    std::cout << "  " << "adjust block at " << trans_adjust_frame <<
  //    std::endl; unsigned int block_frame = 0; for (unsigned int offset = 0;
  //    offset < my_num_frames; ++offset) {
  //        unsigned int f = my_first_frame + offset;

  //        // adjust transpose block for last batch of frames
  //        if ( f == trans_adjust_frame) {
  //            sol_per_block = number_frames % sol_per_block;
  //            trans_sol.resize(model.size()*sol_per_block);
  //            block_frame = 0;
  //        }

  //        // handle transposed hdf5 file (write in larger blocks)
  //        for (unsigned int i=0; i < model.size(); ++i) {
  //            trans_sol[i*sol_per_block + block_frame] =
  //            intensity_buf[offset][i];
  //        }

  //        if (f < trans_adjust_frame) {
  //            if ( (f+1) % sol_per_block == 0) {
  //                std::cout << "Writing transpose solution at frame " << (f+1)
  //                << std::endl; h5t->write_solution(trans_sol);
  //            }
  //        } else {
  //            if ( (f-trans_adjust_frame+1) % sol_per_block == 0 ) {
  //                std::cout << "Writing transpose solution at frame " << (f+1)
  //                << std::endl; h5t->write_solution(trans_sol);
  //            }
  //        }

  //        ++block_frame;
  //        block_frame = block_frame % sol_per_block;
  //    }
  //
  //    //h5t->add_units("frames", "intensity");
  //}

  coverage.resize(msize, 0.);
  sol_avg_final.resize(msize, 0.);
  sol_rms_final.resize(msize, 0.);

  if (0 == my_mpi_rank) {
    // Write additional datasets for avg and rms
    // (note: convert from double (the partials) to single (the finals)
    for (unsigned int i = 0; i < model.size(); ++i) {
      sol_avg_final[i] = sol_avg_partial[i] / number_frames;
      sol_rms_final[i] = sqrt(sol_rms_partial[i] / number_frames);
    }
    if (ifile.grid_type == upsp::GridType::P3D) {
      model.adjust_solution(sol_rms_final);
      model.adjust_solution(sol_avg_final);
    }
    pwrite_full(output_files["intensity_rms"].fd, &(sol_rms_final[0]),
                sol_rms_final.size() * sizeof(float), 0);
    pwrite_full(output_files["intensity_avg"].fd, &(sol_avg_final[0]),
                sol_avg_final.size() * sizeof(float), 0);

    // Create a sample Iref/I for frame 1
    for (unsigned int i = 0; i < sol1.size(); ++i) {
      sol1[i] = sol_avg_final[i] / sol1[i] - 1.0;
    }
    pwrite_full(output_files["intensity_ratio_0"].fd, &(sol1[0]),
                sol1.size() * sizeof(float), 0);
    
    // Create coverage datasets (project 1s)
    std::vector<float> ind_sol;
    for (unsigned int c = 0; c < ifile.cameras; ++c) {
      cv::Mat one_frame = cv::Mat::ones(elems.cals[c].size().width,
                                        elems.cals[c].size().height, CV_32F);
      upsp::project_frame(elems.projs[c], one_frame, ind_sol);
      if (c == 0) {
        coverage.assign(ind_sol.begin(), ind_sol.end());
      } else {
        std::transform(coverage.begin(), coverage.end(), ind_sol.begin(),
                       coverage.begin(), std::plus<float>());
      }
      // todo-mshawlec: re-enable writing per-camera coverage.
      // Write out to new flat files (like rms, avg, etc.)
      // std::string data_name =
      //     "cam" + std::to_string(ifile.cam_nums[c]) + "_contribution";
      // if (ifile.grid_type == upsp::GridType::P3D) {
      //   model.adjust_solution(ind_sol);
      // }
      // h5t->write_new_dataset(data_name, ind_sol);
    }

    if (ifile.grid_type == upsp::GridType::P3D) {
      model.adjust_solution(coverage);
    }
    pwrite_full(output_files["coverage"].fd, &(coverage[0]),
                coverage.size() * sizeof(float), 0);

    // Write regression test data
    {
      // Helper function to dump "maxels" of vector "v" to file "fname"
      auto dump = [](const std::string& fname, const std::vector<float> v,
                     const int maxels) {
        if (v.empty()) return false;
        float* outp = (float*)malloc(sizeof(float) * maxels);
        if (!outp) return false;
        int step = v.size() < maxels ? 1 : v.size() / maxels;
        int ii = 0, jj = 0;
        while (ii < maxels && jj < v.size()) {
          outp[ii] = v[jj];
          ii = ii + 1;
          jj = jj + step;
        }
        FILE* fp = fopen(fname.c_str(), "wb");
        if (!fp) {
          free(outp);
          return false;
        }
        fwrite((void*)outp, sizeof(float), ii, fp);
        fclose(fp);
        free(outp);
        return true;
      };
      // Data sets are prohibitively large, so we write a limited number
      // of data points from each data set to file. These files can then
      // be checked for floating-point equality between versions of the
      // processing code (for non-functional changes, data should not
      // change significantly between code versions)
      dump(ifile.out_dir + "/" + "vv-int-rms.dat", sol_rms_final, 1000);
      dump(ifile.out_dir + "/" + "vv-int-avg.dat", sol_avg_final, 1000);
      dump(ifile.out_dir + "/" + "vv-int-coverage.dat", coverage, 1000);
      dump(ifile.out_dir + "/" + "vv-int-sample1.dat", sol1, 1000);
      std::cout << "Wrote regression data" << std::endl;
    }
  }

  MPI_Bcast(&(sol_avg_final[0]), (int)sol_avg_final.size(), MPI_FLOAT, 0,
            MPI_COMM_WORLD);
  MPI_Bcast(&(sol_rms_final[0]), (int)sol_rms_final.size(), MPI_FLOAT, 0,
            MPI_COMM_WORLD);
  MPI_Bcast(&(coverage[0]), (int)coverage.size(), MPI_FLOAT, 0, MPI_COMM_WORLD);

  // Construct the transpose
  timedBarrierPoint(0 == my_mpi_rank,
                    "phase1: begin to construct the transpose");
  if (0 == my_mpi_rank) std::cout << "Construct the transpose" << std::endl;

  float(*intensity_transpose_buf)[number_frames] =
      (float(*)[number_frames])ptr_intensity_transpose_data;
  global_transpose(&(intensity_buf[0][0]), &(intensity_transpose_buf[0][0]));

  timedBarrierPoint(0 == my_mpi_rank, "phase1: transpose complete");
  MPI_Barrier(MPI_COMM_WORLD);
  if (0 == my_mpi_rank) std::cout << "transpose complete" << std::endl;

  // Tell the async i/o thread that the intensity transpose is available
  __sync_synchronize();
  intensity_transpose_ready = true;

  return 0;
}

template <typename P1Elems>
int InitializeCameraCalibration(Phase1Settings& sett, P1Elems& elems) {
  // For brevity
  typedef typename P1Elems::Model Model;
  upsp::FileInputs& ifile = sett.ifile;
  Model& model = *elems.model;

  elems.cals.resize(ifile.cameras);
  elems.first_frames.resize(ifile.cameras);
  for (unsigned int c = 0; c < ifile.cameras; ++c) {
    // Diagnostic data to write to disk:
    // - "Raw" first frame, both lossless EXR and 8-bit PNG
    cv::Mat img = elems.cams[c]->get_frame(1);
    img.convertTo(elems.first_frames[c], CV_32F);
    cv::Mat img8U = upsp::convert2_8U(img);

    if (0 == my_mpi_rank) imwrite(img8U, sett, c, "8bit-raw.png");
    if (0 == my_mpi_rank) imwrite(elems.first_frames[c], sett, c, "raw.exr");

    // Load camera calibration. The camera calibration contains:
    // - Intrinsic parameters (lens distortion, focal length, sensor offset)
    // - Extrinsic parameters to project from 3D (model coordinate system) to
    //   2D (camera image plane pixel coordinates). The calibration must be
    //   provided for the first frame in the video file, accounting for
    //   any wind-on deflection of the model away from static info provided in
    //   the WTD file for the given test condition.
    upsp::read_json_camera_calibration(elems.cals[c], ifile.cals[c]);
  }

  return 0;
}

// Initialize Phase1Elements members for fiducial patching (for
// example, polynomial patch objects).
//
// In addition, images are written to disk for diagnostic purposes.
// For each camera, the first frame is read, scaled to 8-bit PNG,
// and then overlayed with:
// - projected locations of each fiducial visible to the camera
// - results of clustering neighboring fiducials in the image plane
// - boundary of "patch" areas to cover each cluster
// - final result of "patching" the image data
//   over fiducials overlayed
template <typename P1Elems>
int InitializeImagePatches(Phase1Settings& sett, P1Elems& elems) {
  const auto& model = *(elems.model);
  elems.patchers.resize(sett.ifile.cameras);
  for (unsigned int c = 0; c < sett.ifile.cameras; ++c) {
    cv::Mat img = elems.cams[c]->get_frame(1);
    cv::Mat img8U = upsp::convert2_8U(img);

    // Get visible targets + fiducials
    std::vector<upsp::Target> targs;
    {
      const auto fname = sett.ifile.targets[c];
      std::vector<upsp::Target> orig_targs;
      std::vector<upsp::Target> fiducials;
      if (!upsp_files::read_psp_target_file(fname, orig_targs)) return 1;
      if (!upsp_files::read_psp_target_file(fname, fiducials, false, "*Fiducials")) return 1;
      std::vector<upsp::Target> tmptargs = orig_targs;
      tmptargs.insert(tmptargs.end(), fiducials.begin(), fiducials.end());
      const auto oblique_angle = sett.ifile.oblique_angle;
      const float thresh = deg2_rad(180. - std::min(oblique_angle + 5.0, 90.0));
      getTargets(model, elems.cals[c], tmptargs, elems.scene, thresh, targs);
      elems.cals[c].map_points_to_image(targs);
    }

    // Diagnostic - projected fiducial positions
    if (0 == my_mpi_rank) {
      const auto img_out = upsp::add_targets(targs, img8U, cv::Scalar(0, 255, 0), true);
      imwrite(img_out, sett, c, "8bit-projected-fiducials.png");
    }

    // Compute target diameters
    std::vector<float> diams;
    get_target_diameters(model, elems.cals[c], targs, elems.scene, diams);
    for (unsigned int i = 0; i < targs.size(); ++i) {
      targs[i].diameter = diams[i] * sett.target_diam_sf;
    }

    // Cluster the targets
    std::vector<std::vector<upsp::Target>> clusters;
    const int bound_pts = sett.bound_thickness + sett.buffer_thickness;
    upsp::cluster_points(targs, clusters, bound_pts);

    if (0 == my_mpi_rank) {
      std::cout << "Sorted " << targs.size() << " targets into ";
      std::cout << clusters.size() << " clusters" << std::endl;
    }

    // Diagnostic - clustered fiducials
    if (0 == my_mpi_rank) {
      auto img_out = img8U.clone();
      std::vector<cv::Scalar> cluster_colors;
      upsp::get_colors(clusters.size(), cluster_colors);
      for (unsigned int i = 0; i < clusters.size(); ++i) {
        const auto color = cluster_colors[i];
        img_out = upsp::add_targets(clusters[i], img_out, color, false);
      }
      imwrite(img_out, sett, c, "8bit-fiducial-clusters.png");
    }

    // Initialize fiducial patching

    // Compute threshold using histogram of image.
    // Threshold is used to size each patch region
    // intelligently to capture all "darkened" pixels
    // due to the fiducial marks on the model.
    cv::Mat_<uint16_t> img16 = img.clone();
    const auto bit_depth = elems.cams[c]->get_bit_depth();
    std::vector<int> edges;
    std::vector<int> counts;
    upsp::intensity_histc(img16, edges, counts, bit_depth, TWO_POW(8));
    unsigned int thresh = edges[upsp::first_min_threshold(counts, 5)] + 5;

    elems.patchers[c] = new upsp::PatchClusters<float>(
        clusters, img.size(), sett.bound_thickness, sett.buffer_thickness
    );
    elems.patchers[c]->threshold_bounds(img16, thresh, 2);

    // Diagnostic - all cluster boundaries
    if (0 == my_mpi_rank) {
      auto img_out = img8U.clone();
      cv::cvtColor(img_out, img_out, cv::COLOR_GRAY2RGB);
      for (int i = 0; i < elems.patchers[c]->bounds_x.size(); ++i) {
        for (int j = 0; j < elems.patchers[c]->bounds_x[i].size(); ++j) {
          img_out.at<cv::Vec3b>(
            elems.patchers[c]->bounds_y[i][j],
            elems.patchers[c]->bounds_x[i][j]
          ) = cv::Vec3b(255, 0, 0);
        }
      }
      imwrite(img_out, sett, c, "8bit-cluster-boundaries.png");
    }
  }

  return 0;
}

template <typename P1Elems>
int InitializeModel(Phase1Settings& sett, P1Elems& elems) {
  if (sett.ifile.has_normals()) {
    upsp_files::set_surface_normals(sett.ifile.normals, *(elems.model));
    timedBarrierPoint(0 == my_mpi_rank, ("Overwrote select model surface "
      "normals (using '" + sett.ifile.normals + "')").c_str());
  }
  {
    psp::BlockTimer bt(0 == my_mpi_rank, "Creating BVH");
    elems.scene = createBVH(*(elems.model), elems.triNodes);
  }
  return 0;
}

/*****************************************************************************/
template <typename P1Elems>
int phase0(Phase1Settings& sett, P1Elems& elems) {
  if (InitializeModel(sett, elems)) return 1;
  if (InitializeCameraCalibration(sett, elems)) return 1;
  if (InitializeImagePatches(sett, elems)) return 1;
  timedBarrierPoint(0 == my_mpi_rank, "Initialized camera cals and image patching");
  return 0;
}

/** Store data for overlapped nodes, to transfer forward
 *  primary node is always the lowest node index, so will
 *  have the value when a superceded node comes up */
template <typename Model>
struct OverlapNodeStorage {
  typedef typename Model::data_type FP;

  OverlapNodeStorage(Model* model, unsigned int n_frames)
      : model_(model), n_frames_(n_frames), stored_nodes_(0) {}

  /** Will store the data for future access if the
   *  node is overlapping and will be called for in the future
   */
  void store_data(unsigned int nidx, FP* data) {
    if (model_->is_overlapping(nidx) && !model_->is_superceded(nidx)) {
      data_.resize(data.size() + n_frames_);
      log_[nidx] = &data[stored_nodes_ * n_frames_];

      std::copy(data, data + n_frames_, &data_[stored_nodes_ * n_frames_]);

      ++stored_nodes_;
    }
  }

  /** Access stored node data
   *
   * @param[in] nidx  node index of interest
   * @return          pointer to the data (every frame)
   *
   * @pre model_->is_superceded(nidx)
   */
  FP* access_data(unsigned int nidx) {
    assert(model_->is_superceded(nidx));

    assert(log_.find(nidx) != log_.end());

    return log_[nidx];
  }

  /***************/

  unsigned int n_frames_;

  unsigned int stored_nodes_;

  std::vector<FP> data_;
  std::unordered_map<unsigned int, FP*> log_;  // points to the start of data
                                               // for each node

  Model* model_;
};

/*****************************************************************************/

template <typename P2Elems>
int phase2(Phase2Settings& sett, Phase2Files& p2_files, P2Elems& elems) {
  typedef typename P2Elems::Model Model;
  auto& model = *(elems.model);
  const auto& ifile = sett.ifile;

  /*******************************************/
  /* Load in Reference Values                */
  /*******************************************/

  // Load paint calibration
  upsp::PaintCalibration pcal(p2_files.paint_cal);

  if (0 == my_mpi_rank) {
    std::cout << "Paint Calibration = " << std::endl;
    std::cout << pcal << std::endl;
  }

  // Load the wind tunnel conditions
  elems.tunnel_conditions = upsp::read_tunnel_conditions(ifile.sds);
  auto& tcond = elems.tunnel_conditions;
  tcond.test_id = ifile.test_id;
  tcond.run = ifile.run;
  tcond.seq = ifile.sequence;

  tcond.ttot += sett.F_to_R;  // convert to Rankine
  float t_inf =
      tcond.ttot / (1.0 + (sett.gamma - 1.0) * 0.5 * tcond.mach * tcond.mach);
  tcond.ttot -= sett.F_to_R;  // convert to Fahrenheit
  t_inf -= sett.F_to_R;       // convert to Fahrenheit

  float wall_temp = sett.r * (tcond.ttot - t_inf) + t_inf;  // in Fahrenheit

  float model_temp = wall_temp;
  if (!std::isnan(tcond.tcavg)) {
    model_temp = tcond.tcavg;
    LOG_INFO(
      "*** Using thermocouple average (%4.1fF) "
      "for model temp, supersedes estimated temperature "
      "based on boundary layer recovery factor (%4.1fF)",
      tcond.tcavg, wall_temp
    );
  } else {
    LOG_INFO(
      "*** Using estimated temperature based on"
      " boundary layer recovery factor (%4.1fF)",
      wall_temp
    );
  }

  MPI_Barrier(MPI_COMM_WORLD);

  /*******************************************/
  /* Load the temperature psp data                */
  /*******************************************/

  std::vector<float> model_temp_input;
  // defined tmeprature
  model_temp_input = std::vector<float>(msize, model_temp);

  // read model_temp_input from p2_files.model_temp_p3d if defined
  if (sett.read_model_temp) {

    LOG_INFO("*** Using temperature file: %s",p2_files.model_temp_p3d.c_str());

    // Load model temperature solution
    if (upsp::is_structured<Model>()) {
      model_temp_input = upsp::read_plot3d_scalar_function_file(p2_files.model_temp_p3d);  //temperature 
      if (model_temp_input.size() != msize) {
        LOG_ERROR(
          "Mode-temperature function file inconsistent with grid "
          "(expect %d values, got %d)",
          msize, model_temp_input.size()
        );
        return 1;
      }
    } else {
      std::vector<float> in_model_temp =
          upsp::read_plot3d_scalar_function_file(p2_files.model_temp_p3d);  // temperature 
      // TODO: crashes without sett.grid_tol
      upsp::P3DModel_<float> steady_grid(p2_files.steady_grid, sett.grid_tol);

      model_temp_input = upsp::interpolate(steady_grid, in_model_temp, model, sett.k, sett.p);
    }
  }
  MPI_Barrier(MPI_COMM_WORLD);

  /*******************************************/
  /* Load the steady psp data                */
  /*******************************************/

  std::vector<float> steady;
  if (sett.wind_off) {
    steady = std::vector<float>(msize, 0.0);
  } else {
    // Load steady state solution
    if (upsp::is_structured<Model>()) {

      LOG_INFO("*** Using steady state file: %s",p2_files.steady_p3d.c_str());

      steady = upsp::read_plot3d_scalar_function_file(p2_files.steady_p3d);  // Cp
      if (steady.size() != msize) {
        LOG_ERROR(
          "Steady-state function file inconsistent with grid "
          "(expect %d values, got %d)",
          msize, steady.size()
        );
        return 1;
      }
    } else {
      std::vector<float> in_steady =
          upsp::read_plot3d_scalar_function_file(p2_files.steady_p3d);  // Cp
      // TODO: crashes without sett.grid_tol
      upsp::P3DModel_<float> steady_grid(p2_files.steady_grid, sett.grid_tol);

      steady = upsp::interpolate(steady_grid, in_steady, model, sett.k, sett.p);
    }
  }
  MPI_Barrier(MPI_COMM_WORLD);

  /*******************************************/
  /* Prepare for polynomial fit             */
  /*******************************************/

  // Initialize the polynomial fitter
  upsp::TransPolyFitter<float> pfitter;
  pfitter = upsp::TransPolyFitter<float>(number_frames, sett.degree, msize);

  /*******************************************/
  /* Initialize output h5 files              */
  /*******************************************/

  // Get this rank's chunk of nodes
  apportion(msize, num_mpi_ranks, &(rank_start_node[0]), &(rank_num_nodes[0]));

  // Create HDF5 file and fill with grid and attributes
  upsp::PSPWriter<Model>* h5t;
  upsp::PSPWriter<Model>* h5_extra;
  if (0 == my_mpi_rank) {
    std::cout << "Initializing new hdf5 files:" << std::endl;
    std::cout << "    " << (p2_files.h5_out) << std::endl;
    h5t = new upsp::PSPWriter<Model>(p2_files.h5_out, &model, number_frames,
                                     1.0, true, sett.trans_nodes);
    std::cout << "    " << (p2_files.h5_out_extra) << std::endl;
    h5_extra = new upsp::PSPWriter<Model>(p2_files.h5_out_extra, &model, 1);

    std::cout << "Writing header information to hdf5 files" << std::endl;
    h5t->write_grid(sett.grid_units);
    h5_extra->write_grid(sett.grid_units);

    h5t->write_tunnel_conditions(elems.tunnel_conditions);
    h5_extra->write_tunnel_conditions(elems.tunnel_conditions);

    h5t->write_camera_settings(elems.camera_settings);
    h5_extra->write_camera_settings(elems.camera_settings);

    h5t->write_string_attribute("code_version", sett.code_version);
    h5_extra->write_string_attribute("code_version", sett.code_version);
  }
  MPI_Barrier(MPI_COMM_WORLD);

  /*******************************************/
  /* Process Data                            */
  /*******************************************/

  // Set up storage for solutions
  rms.resize(msize, 0.);
  avg.resize(msize, 0.);
  gain.resize(msize, 0.);

  float(*intensity_transpose_buf)[number_frames] =
      (float(*)[number_frames])ptr_intensity_transpose_data;

  // Process all frames
  MPI_Barrier(MPI_COMM_WORLD);

  // read in intensity for all frames, this chunk of nodes
  unsigned int node_start = rank_start_node[my_mpi_rank];
  unsigned int my_num_nodes = rank_num_nodes[my_mpi_rank];
  unsigned int node_end = node_start + my_num_nodes - 1;

  std::cout << "Rank " << my_mpi_rank << " will process nodes "
            << "[" << node_start << "," << node_end << "]" << std::endl;

  MPI_Barrier(MPI_COMM_WORLD);
  if (0 == my_mpi_rank) std::cout << "Beginning node processing" << std::endl;

  float(*pressure_transpose_buf)[number_frames] =
      (float(*)[number_frames])ptr_pressure_transpose_data;

#pragma omp parallel
  {
    std::vector<double> local_rms(msize, 0.);
    std::vector<double> local_avg(msize, 0.);
    std::vector<double> local_gain(msize, 0.);

// process the current chunk of data (some nodes, all frames)
#pragma omp for
    for (unsigned int idx = 0; idx < my_num_nodes; ++idx) {
      unsigned int i = node_start + idx;
      std::vector<float> node_sol(number_frames, 0.);

      // skip if there is no coverage for this node
      // will keep nans in sol
      if (coverage[i] == 0) {
        pfitter.skip_fit(idx);
        local_rms[i] = std::numeric_limits<float>::quiet_NaN();
        local_avg[i] = std::numeric_limits<float>::quiet_NaN();
        local_gain[i] = std::numeric_limits<float>::quiet_NaN();
        continue;
      }

      // compute gain
      float Pss = tcond.qbar * steady[i] + tcond.ps;
      local_gain[i] = pcal.get_gain(model_temp_input[i], Pss);

      // convert to Iref/I
      for (unsigned int f = 0; f < number_frames; ++f) {
        node_sol[f] = sol_avg_final[i] / intensity_transpose_buf[idx][f];
      }

      // create curve fits and evaluate for this node
      std::vector<float> sol_fit = pfitter.eval_fit(&node_sol[0], 1, idx);

      for (unsigned int f = 0; f < number_frames; ++f) {
        // Compute delta pressure (psi)
        float pressure = (node_sol[f] - sol_fit[f]) * local_gain[i];

        // Convert to delta Cp
        node_sol[f] = pressure * 12.0 * 12.0 / tcond.qbar;
        pressure_transpose_buf[idx][f] = node_sol[f];

        // add to rms and avg
        local_rms[i] += (node_sol[f] * node_sol[f]);
        local_avg[i] += node_sol[f];
      }
    }

// Combine the thread partials
#pragma omp critical
    for (unsigned int i = 0; i < msize; ++i) {
      rms[i] += local_rms[i];
      avg[i] += local_avg[i];
      gain[i] += local_gain[i];
    }
  }

  // TODO - write solution flat file
  // Write nodes to h5 files
  // h5t->write_solution(sol, true);

  std::cout << "Rank " << my_mpi_rank << " finished nodes "
            << "[" << node_start << "," << node_end << "]" << std::endl;
  MPI_Barrier(MPI_COMM_WORLD);

  // TODO
  // ofs_log << node_end << std::endl;

  // TODO
  // ofs_log.close();

  // Combine the rank-totals to get the overall totals (result in rank 0)
  if (0 == my_mpi_rank)
    std::cout << "Global reduction of avg, rms and gain ..." << std::endl;
  void* sendbuf = (void*)((0 == my_mpi_rank) ? MPI_IN_PLACE : &(gain[0]));
  MPI_Reduce(sendbuf, &(gain[0]), msize, MPI_DOUBLE, MPI_SUM, 0,
             MPI_COMM_WORLD);
  sendbuf = (void*)((0 == my_mpi_rank) ? MPI_IN_PLACE : &(avg[0]));
  MPI_Reduce(sendbuf, &(avg[0]), msize, MPI_DOUBLE, MPI_SUM, 0, MPI_COMM_WORLD);
  sendbuf = (void*)((0 == my_mpi_rank) ? MPI_IN_PLACE : &(rms[0]));
  MPI_Reduce(sendbuf, &(rms[0]), msize, MPI_DOUBLE, MPI_SUM, 0, MPI_COMM_WORLD);
  if (0 == my_mpi_rank) std::cout << "Global reduction complete" << std::endl;

  if (0 == my_mpi_rank) {
    // TODO Add units to frames dataset
    // h5t->add_units("frames", "delta Cp");

    // Write additional datasets for avg, rms and gain
    std::vector<float> avg_final(msize, 0.0);
    std::vector<float> rms_final(msize, 0.0);
    std::vector<float> gain_final(msize, 0.0);
    for (unsigned int i = 0; i < msize; ++i) {
      avg_final[i] = avg[i] / number_frames;
      rms_final[i] = sqrt(rms[i] / number_frames);
      gain_final[i] = gain[i];
    }

    h5_extra->write_new_dataset("rms", rms_final, "delta Cp");
    h5_extra->write_new_dataset("average", avg_final, "delta Cp");

    h5t->write_new_dataset("rms", rms_final, "delta Cp");

    std::cout << "Writing pressure rms, average, gain flat files" << std::endl;
    pwrite_full(output_files["rms"].fd, &(rms_final[0]),
                rms_final.size() * sizeof(float), 0);
    pwrite_full(output_files["avg"].fd, &(avg_final[0]),
                avg_final.size() * sizeof(float), 0);
    pwrite_full(output_files["gain"].fd, &(gain_final[0]),
                gain_final.size() * sizeof(float), 0);

    // Write out the coverage dataset
    h5_extra->write_new_dataset("coverage", coverage);
    h5t->write_new_dataset("coverage", coverage);

    // Write out the steady state data
    for (unsigned int i = 0; i < msize; ++i) {
      if (steady[i] > 3.0) {
        steady[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }

    // Write steady state flat file
    std::cout << "Writing steady state flat file" << std::endl;
    h5t->write_new_dataset("steady_state", steady, "Cp");
    h5_extra->write_new_dataset("steady_state", steady, "Cp");
    pwrite_full(output_files["steady_state"].fd, &(steady[0]),
                steady.size() * sizeof(float), 0);

    // Write model temperature flat file
    std::cout << "Writing model temperature flat file" << std::endl;
    h5t->write_new_dataset("model_temp", model_temp_input, "F");
    h5_extra->write_new_dataset("model_temp", model_temp_input, "F");
    pwrite_full(output_files["model_temp"].fd, &(model_temp_input[0]),
                model_temp_input.size() * sizeof(float), 0);

    // Write regression data
    {
      // Data sets are prohibitively large, so we write a limited number
      // of data points from each data set to file. These files can then
      // be checked for floating-point equality between versions of the
      // processing code (for non-functional changes, data should not
      // change significantly between code versions)
      upsp::fwrite(p2_files.add_out_dir + "/" + "vv-cp-rms.dat", rms_final,
                   1000);
      upsp::fwrite(p2_files.add_out_dir + "/" + "vv-cp-avg.dat", avg_final,
                   1000);
      std::cout << "Wrote regression data" << std::endl;
    }

    // delete phase2Model;
    delete h5t;
    delete h5_extra;
  }

  if (0 == my_mpi_rank)
    std::cout << "Write pressure transpose ..." << std::endl;

  // This is the last thing we do.  Rather than flag an async thread
  // to write out the pressure transpose, we just do it ourselves.
  write_block((void*)ptr_pressure_transpose_data,
              output_files["pressure_transpose"].fd);
  if (0 == my_mpi_rank)
    std::cerr << "## 'pressure_transpose' written" << std::endl;

  if (0 == my_mpi_rank) {
    std::cout << "Wait for all ranks to complete" << std::endl;
  }
  timedBarrierPoint(0 == my_mpi_rank, "Phase 2 done");

  return 0;
}
