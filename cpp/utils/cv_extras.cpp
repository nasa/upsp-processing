#include "utils/cv_extras.h"
#include <cmath>

namespace upsp {

/********************************************************************
* TextSizeIterator
********************************************************************/

/*****************************************************************************/
TextSizeIterator::TextSizeIterator() {
    position = -1;
}

/*****************************************************************************/
TextSizeIterator::TextSizeIterator(std::string text, int font_face, 
        double font_scale, int thickness, unsigned int position) : text(text), 
        font_face(font_face), font_scale(font_scale), thickness(thickness), 
        position(position) {}

/*****************************************************************************/
unsigned int TextSizeIterator::operator*() const { 
    int buf=0;
    cv::Size text_sz = cv::getTextSize(text.substr(0,position), font_face, 
            font_scale, thickness, &buf);
    return text_sz.width;
}

/*****************************************************************************/
int TextSizeIterator::get_position() const { return position; }

/*****************************************************************************/
TextSizeIterator& TextSizeIterator::operator++() {
    ++position;
    return *this;
}

/*****************************************************************************/
bool TextSizeIterator::operator==(const TextSizeIterator& tsi) const {
    if ((position < 0) || (position >= text.length())) {
        return ( (tsi.position < 0) || (tsi.position >= text.length()) );
    } else {
        return (text == tsi.text) && (position == tsi.position);
    }
}

/*****************************************************************************/
bool TextSizeIterator::operator<(const TextSizeIterator& tsi) const {
    if ((position < 0) || (position >= text.length())) {
        return (!( (tsi.position < 0) || (tsi.position >= text.length()) ));
    } else {
        if (text == tsi.text) {
            return position < tsi.position;
        } else {
            if (text.length() == tsi.text.length()) {
                for (int i=0; i < text.length(); ++i) {
                    if (text[i] < tsi.text[i]) {
                        return true;
                    }
                }
                return false;
            } else {
                return text.length() < tsi.text.length();
            }
        }
    }
}   

/********************************************************************
* Functions
********************************************************************/

std::string convert_type_string(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


/*****************************************************************************/
std::string convert_depth_string(int cv_depth) {

    int img_type = cv_depth%8;
    std::string type_str;

    switch (img_type) {
        case CV_8U:
            type_str = "CV_8U";
            break;
        case CV_8S:
            type_str = "CV_8S";
            break;
        case CV_16U:
            type_str = "CV_16U";
            break;
        case CV_16S:
            type_str = "CV_16S";
            break;
        case CV_32S:
            type_str = "CV_32S";
            break;
        case CV_32F:
            type_str = "CV_32F";
            break;
        case CV_64F:
            type_str = "CV_64F";
            break;
        default:
            break;
    }

    int channel = cv_depth/8 + 1;

    std::stringstream ss;
    ss << type_str << "C" << channel;

    return ss.str();
}

/*****************************************************************************/
cv::Mat convert2_8U(const cv::Mat& inp) {

    // Get maximum and minimum values in input file
    double min, max;
    cv::minMaxLoc(inp, &min, &max);

    // Put image in range 0.0 - 255.0 to convert
    cv::Mat_<double> conv = inp - min;
    cv::Mat frame8;
    conv.convertTo(frame8, CV_8U, 255.0/(max-min)); 

    return frame8;
}

/*****************************************************************************/
void get_ac_submat(const cv::Mat& input, cv::Mat& output, const cv::Rect roi) {
    output = input(roi).clone();
    output.convertTo(output, CV_32F);
    output = output - cv::mean(output)[0];
}

/*****************************************************************************/
void get_dc_submat(upsp::PSPVideo* cam, cv::Mat& output, const cv::Rect roi, 
        int num_frames, int start_frame) {

    // Process num_frames
    cv::Mat inter;
    output = cv::Mat(roi.size(),CV_32F,0.0);
    for (unsigned int i=start_frame; i <= num_frames; ++i) {
        get_ac_submat(cam->get_frame(i), inter, roi);
        output += (1.0/num_frames)*inter;
    }

}

/*****************************************************************************/
void split_text(std::string input, unsigned int max_len, int font_face, 
        double font_scale, int thickness, std::vector<std::string>& splt) {

    // method finds lower bound, so text < max_len, increase by 1 to get <=
    max_len += 1;

    int buf = 0;
    cv::Size text_sz = cv::getTextSize(input, font_face, font_scale, thickness, &buf);

    splt.clear();
    if (text_sz.width < max_len) {
        splt.push_back(input);
        return;
    }

    std::string text = input;
    TextSizeIterator tsi_end;
    while (text.length() > 0) {
        // get the next component that just fits inside max_len, tsi_comp
        // will actually point 1 past the one that just fits max_len or to the end of
        // the string
        TextSizeIterator tsi_begin(text, font_face, font_scale, thickness, 0);
        TextSizeIterator tsi_comp = std::lower_bound(tsi_begin, tsi_end, max_len);

        if (tsi_comp.get_position() == 0) {
            std::cerr << "Cannot split text, individual characters exceed limit" << std::endl;
            std::abort();
        }

        // handle case where iterator is the the last character in the string
        if ( (text.length() == tsi_comp.get_position() ) && ( (*tsi_comp) < max_len) ) {
            splt.push_back(text);
            break;
        } else {
            splt.push_back(text.substr(0, tsi_comp.get_position()-1 ));
        }

        // update text
        text = text.substr(tsi_comp.get_position()-1);
    }

}

/*****************************************************************************/
/*
 * algorithm:
 * 1. look for pixels with value above thresh, save their locations
 *    if we find too many, assume that pixels aren't hot but instead the
 *    image is overexposed, so do nothing
 * 2. for each found pixel, replace it with the median of the neighbors. 
 *    don't replace the pixel if the median is too close to the old value
 *    as we only want to replace isolated hot pixels
 *
 * Any replaced pixels may be logged (printed) if logit is true.
 */

void fix_hot_pixels(cv::Mat& inp, bool logit, int thresh, int min_change, int max_hot) {

  cv::MatIterator_<uint16_t> inp_pix = inp.begin<uint16_t>();
  int n_pix = inp.rows * inp.cols;
  int hot_pix_locs[max_hot];
  int n_hot = 0;
  // find locations of hot pixels and save them
  for (int pix = 0; pix < n_pix; ++pix, ++inp_pix) {
    if (*inp_pix >= thresh) {
      if (n_hot >= max_hot) {
	if (logit)
	    std::cout << "fix_hot_pixels: too many pixels look hot" << std::endl;
	return;
      }
      hot_pix_locs[n_hot++] = pix;
    }
  }

  for (int hot_pix = 0; hot_pix < n_hot; ++hot_pix) {
    // calculate median of 4 neighbor pixels, skipping pixels that are outside image
    uint16_t vals[4];
    uint32_t n_vals = 0;
    int row = hot_pix_locs[hot_pix] / inp.cols;
    int col = hot_pix_locs[hot_pix] % inp.cols;
    if (row > 0)          { vals[n_vals] = inp.at<uint16_t>(row-1, col); ++n_vals; }
    if (col > 0)          { vals[n_vals] = inp.at<uint16_t>(row, col-1); ++n_vals; }
    if (row < inp.rows-1) { vals[n_vals] = inp.at<uint16_t>(row+1, col); ++n_vals; }
    if (col < inp.cols-1) { vals[n_vals] = inp.at<uint16_t>(row, col+1); ++n_vals; }
    std::sort(vals, vals+n_vals);
    uint16_t old_val = inp.at<uint16_t>(row, col);
    uint16_t new_val = vals[n_vals/2];
    if (old_val - new_val > min_change) {
      if (logit)
	std::cout << "replacing hot pix @ " << row << ',' << col << " old " <<
	  old_val << " new " << new_val << std::endl;
      inp.at<uint16_t>(row, col) = new_val;
    }
    else if (logit)
      std::cout << "NOT replacing hot pix @ " << row << ',' << col << " old " <<
        old_val << " new " << new_val << std::endl;

  }
}

/* ripped + adapted from
 * https://stackoverflow.com/questions/28825520/is-there-something-like-matlabs-colorbar-for-opencv
 * */
void nodes_per_pixel_colormap(cv::Mat& nodecounts, cv::Mat& dst)
{
  // Build color map (OpenCV convention is (B,G,R))
  cv::Mat colors(256, 1, CV_8UC3);
  const int number_colors = 6;
  colors.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 0, 0); // Black
  colors.at<cv::Vec3b>(1, 0) = cv::Vec3b(0, 255, 0); // Green
  colors.at<cv::Vec3b>(2, 0) = cv::Vec3b(0, 255, 255); // Yellow
  colors.at<cv::Vec3b>(3, 0) = cv::Vec3b(51, 153, 255); // Orange
  colors.at<cv::Vec3b>(4, 0) = cv::Vec3b(153, 204, 255); // Light Orange
  for (int ii = 5; ii < 256; ii++) {
    colors.at<cv::Vec3b>(ii, 0) = cv::Vec3b(255, 255, 255);  // White
  }

  // Apply color map to input image
  cv::Mat M;
  cv::applyColorMap(nodecounts, M, colors);

  // Color bar + labels size in output window
  const int width_labels = 30;
  const int width_colorbar = 10;
  const int vline = 10;

  // Allocate full output window (colorbar is vertical on right)
  cv::Mat img_window(
    cv::Size(
        nodecounts.cols + vline + width_labels + width_colorbar,
        nodecounts.rows
    ),
    CV_8UC3, cv::Scalar(123, 123, 123)
  );

  // Image sub-area containing color bar
  cv::Mat img_colorbar(
    cv::Size(width_colorbar, nodecounts.rows),
    CV_8UC3, cv::Scalar(123, 123, 123)
  );

  // Image sub-area containing color bar labels
  cv::Mat img_labels(
    cv::Size(width_labels, nodecounts.rows),
    CV_8UC3, cv::Scalar(123, 123, 123)
  );

  // Populate color bar and color bar labels sub-images
  for (int ii = 0; ii < number_colors; ii++) {
    const int colorbar_segment_height = nodecounts.rows / number_colors;
    // vertical coordinates of this colorbar segment
    const int y_upper = ii * colorbar_segment_height;
    const int y_lower = (ii + 1) * colorbar_segment_height;
    // Draw colored patch in color bar
    cv::rectangle(
      img_colorbar,
      cv::Point(0, y_upper), cv::Point(img_colorbar.cols, y_lower),
      colors.at<cv::Vec3b>(ii, 0), cv::FILLED
    );
    // Draw text in labels. Color map saturates on last color,
    // so the label should be ">[last value]"
    std::string label(std::to_string(ii));
    if (ii > 0 && ii == (number_colors - 1)) {
        label = std::string(">") + std::to_string(ii - 1);
    }
    cv::putText(
      img_labels, label,
      cv::Point(5, y_lower - 5),
      cv::FONT_HERSHEY_DUPLEX, 0.6,
      cv::Scalar(0, 0, 0), 1 , 2 , false
    );
  }

  M.copyTo(img_window(cv::Rect(0, 0, nodecounts.cols, nodecounts.rows)));

  img_labels.copyTo(
    img_window(
      cv::Rect(
        nodecounts.cols + vline + width_colorbar, 0,
        width_labels, nodecounts.rows
      )
    )
  );

  img_colorbar.copyTo(
    img_window(
      cv::Rect(
        nodecounts.cols + vline, 0,
        width_colorbar, nodecounts.rows
      )
    )
  );

  dst = img_window.clone();
}

} /* end namespace upsp */
