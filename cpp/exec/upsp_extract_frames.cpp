#include <boost/format.hpp>
#include <iostream>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <vector>

#include "PSPVideo.h"
#include "CineReader.h"
#include "MrawReader.h"

struct ExtractToVideoFileConfiguration {
  int start_frame;
  int number_frames;
  std::string filename;
  double fps;
  int fourcc;
};

struct ExtractToImageFilesConfiguration {
  int start_frame;
  int number_frames;
  int bit_depth;
  std::vector<int> cv_params;
  std::string filename_prefix;
  std::string filename_ext;
};

void GetFrame(upsp::PSPVideo* stream, int ii, cv::Mat& frame, int bit_depth) {
  cv::Mat frame16 = stream->get_frame(ii);
  const auto frame_bit_depth = stream->get_bit_depth();
  if (bit_depth == 8) {
    // scale 12-bit to 8-bit
    frame16.convertTo(frame, CV_8U, (double)TWO_POW(8) / (double)TWO_POW(frame_bit_depth));
  }
  else {
    frame = frame16;
  }
}

int ParseFourcc(cv::CommandLineParser& parser) {
  std::string s = parser.get<std::string>("fourcc");
  if (s.size() == 4) {
    return cv::VideoWriter::fourcc(s[0], s[1], s[2], s[3]);
  } else {
    return std::stoi(s);
  }
}

int ExtractToVideoFile(upsp::PSPVideo* stream,
                       const ExtractToVideoFileConfiguration& cfg) {
  cv::Size frame_size = stream->get_frame_size();
  const bool is_color = false;
  cv::VideoWriter output_stream(cfg.filename, cfg.fourcc, cfg.fps, frame_size,
                                is_color);
  std::cout << "Initialized output stream:" << std::endl;
  std::cout << "  Filename          : " << cfg.filename << std::endl;
  std::cout << "  Frames per second : " << cfg.fps << std::endl;
  std::cout << "  Frame size        : " << frame_size << std::endl;
  std::cout << "  Bit depth         : " << "8" << std::endl;
  std::cout << "  Encoding          : ["
    << char(cfg.fourcc & 255)
    << char((cfg.fourcc >> 8) & 255)
    << char((cfg.fourcc >> 16) & 255)
    << char((cfg.fourcc >> 24) & 255)
    << "]" << std::endl;
  
  cv::Mat frame(frame_size, CV_8U);
  for (int ii = cfg.start_frame; ii < cfg.start_frame + cfg.number_frames; ii++) {
    GetFrame(stream, ii, frame, 8);
    output_stream << frame;
    std::printf("Wrote frame %d / %d\r", ii - cfg.start_frame + 1, cfg.number_frames);
    std::fflush(stdout);
  }
  std::printf("\n");
  return 0;
}

int ExtractToImageFiles(upsp::PSPVideo* stream,
                        const ExtractToImageFilesConfiguration& cfg) {
  boost::format fmter("%s.%05d.%s");  
  cv::Mat frame(stream->get_frame_size(), (cfg.bit_depth == 8) ? CV_8U : CV_16U);
  for (int ii = cfg.start_frame; ii < cfg.start_frame + cfg.number_frames;
       ii++) {
    GetFrame(stream, ii, frame, cfg.bit_depth);
    fmter % cfg.filename_prefix % ii % cfg.filename_ext;
    cv::String name = fmter.str();
    std::printf("%s\n", name.c_str());
    cv::imwrite(name, frame, cfg.cv_params);
  }
  return 0;
}

bool LoadInputStream(cv::CommandLineParser& parser, upsp::PSPVideo** stream) {
  cv::String input_filename = parser.get<cv::String>("input");
  const auto ext = input_filename.substr(input_filename.rfind('.'));
  std::unique_ptr<upsp::VideoReader> reader;
  if (ext.compare(".cine") == 0) {
    reader = std::unique_ptr<upsp::VideoReader>(new upsp::CineReader(input_filename));
    std::cout << "Initialized CineVideo input stream:" << std::endl;
  } else if (ext.compare(".mraw") == 0) {
    reader = std::unique_ptr<upsp::VideoReader>(new upsp::MrawReader(input_filename));
    std::cout << "Initialized MrawVideo input stream:" << std::endl;
  } else {
    std::cerr << "Unknown video file extension '" << ext << "'"
              << " for '" << input_filename << "'."
              << " Valid extensions: {'.cine', '.mraw'}" << std::endl;
    return false;
  }
  *stream = new upsp::PSPVideo(std::move(reader));
  std::cout << "  Filename          : " << input_filename << std::endl;
  std::cout << "  Frames per second : " << (*stream)->get_frame_rate() << std::endl;
  std::cout << "  Frame size        : " << (*stream)->get_frame_size() << std::endl;
  std::cout << "  Bit depth         : " << (*stream)->get_bit_depth() << std::endl;
  return true;
}

int main(int argc, char* argv[]) {
  const cv::String keys =
      "{help h usage ? |        | print this message}"
      "{i input        |        | input video file (*.mraw or *.cine)}"
      "{o output       |        | output <prefix>.<ext> (e.g., 'name.png')."
      " If <ext> is an image format, output will be images named "
      "<prefix>.<frame number>.<ext>;"
      " if <ext> is a video format, output will be a video file named "
      "<prefix>.<ext>}"
      "{s start        |  1     | starting frame}"
      "{c count        |  1     | number of frames}"
      "{r rate         | 100    | (video output-only) output frames per second "
      "(e.g., for 10K fps input video, r=100 ~ 1/100 slow-mo)}"
      "{b bit_depth    |  8     | bit depth of extracted images (8 or 16)}"
      "{f fourcc       | DIVX   | (video output-only) output encoding fourcc (integer or 4-char string)}"
      "{d dumpheader   |        | (optional, *.cine only) dump header}"
      "{compression    |        | (optional) output file OpenCV compression "
      "parameters}";
  cv::CommandLineParser parser(argc, argv, keys);

  if (parser.has("help")) {
    parser.printMessage();
    return 0;
  }

  if (!parser.has("input")) {
    std::cerr << "Must specify 'input'" << std::endl;
    return 1;
  }

  if (!parser.has("output")) {
    std::cerr << "Must specify 'output'" << std::endl;
    return 1;
  }

  if (!parser.check()) {
    parser.printErrors();
    return 1;
  }

  upsp::PSPVideo* stream;
  if (!LoadInputStream(parser, &stream)) {
    return 1;
  }

  const std::string output(parser.get<cv::String>("output"));
  const auto output_prefix = output.substr(0, output.find_last_of("."));
  const auto output_ext = output.substr(output.find_last_of(".") + 1);
  int start_frame = parser.get<int>("s");
  int number_frames = parser.get<int>("c");
  int bit_depth = parser.get<int>("b");

  if (stream->get_number_frames() < start_frame + number_frames - 1) {
    number_frames = stream->get_number_frames() - start_frame;
  }

  if ((bit_depth != 8) && (bit_depth != 16)){
    std::cerr << "Unexpected bit depth '" << bit_depth << "'" << std::endl;
    return 1;
  }

  if ((bit_depth != 8) && (output_ext == "avi")){
    std::cerr << "Video format only supports 8 bit video'";
    return 1;
  }

  if (output_ext == "jpg" || output_ext == "tiff" || output_ext == "png" ||
      output_ext == "pbm" || output_ext == "pgm" || output_ext == "ppm") {
    ExtractToImageFilesConfiguration cfg;
    cfg.filename_prefix = output_prefix;
    cfg.filename_ext = output_ext;
    cfg.start_frame = start_frame;
    cfg.number_frames = number_frames;
    cfg.bit_depth = bit_depth;
    if (parser.has("compression")) {
      int img_qual = parser.get<int>("compression");
      if (output_ext == "jpg") {
        cfg.cv_params.push_back(cv::IMWRITE_JPEG_QUALITY);
      } else if (output_ext == "png") {
        cfg.cv_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
      } else if ((output_ext == "pbm") || (output_ext == "pgm") ||
                 (output_ext == "ppm")) {
        cfg.cv_params.push_back(cv::IMWRITE_PXM_BINARY);
      }
      cfg.cv_params.push_back(img_qual);
    }
    return ExtractToImageFiles(stream, cfg);
  } else if (output_ext == "avi") {
    ExtractToVideoFileConfiguration cfg;
    cfg.start_frame = start_frame;
    cfg.number_frames = number_frames;
    cfg.filename = output;
    cfg.fps = parser.get<double>("rate");
    cfg.fourcc = ParseFourcc(parser);
    return ExtractToVideoFile(stream, cfg);
  } else {
    std::cerr << "unexpected output format '" << output_ext << "'" << std::endl;
    return 1;
  }

  return 0;
}
