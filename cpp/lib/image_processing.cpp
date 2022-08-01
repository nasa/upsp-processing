/** @file 
 *  @brief  Basic Image Statistics and Writing
 *  @date   March 21, 2018
 *  @author jmpowel2
 */

#include "image_processing.h"

namespace upsp {

/********************************************************************
 * SubtractDC
********************************************************************/

/*****************************************************************************/
SubtractDC::SubtractDC() {}

/*****************************************************************************/
SubtractDC::SubtractDC(cv::Rect roi): roi_(roi) {}

/*****************************************************************************/
cv::Mat SubtractDC::operator()(cv::Mat inp) const {
    subtract_dc(inp); 
    return inp;
}

/********************************************************************
 * TimeStamp
********************************************************************/

/*****************************************************************************/
TimeStamp::TimeStamp(const cv::Mat& sample) {

    // Find rectangles in sample image
    std::vector<cv::Rect> rects;
    upsp::find_rectangles(sample, rects);

    assert(rects.size() > 0);

    // Choose the one closest to the bottom left corner (large u, low v)
    cv::Point2i bl(sample.rows,0);
    double dist = std::sqrt( std::pow(sample.rows,2) + std::pow(sample.cols,2));
    unsigned int best = 0;
    for (int i=0; i < rects.size(); ++i) {
        cv::Point2i curr(rects[i].x,rects[i].y);
        if (cv::norm(bl - curr) < dist) {
            best = i;
            dist = cv::norm(bl - curr);
        }
    } 
    ts_ = rects[best];
}

/*****************************************************************************/
cv::Mat TimeStamp::operator()(cv::Mat inp) const {
    cv::Mat roi = inp(ts_);
    roi.setTo(0);
    return inp;
}

/*****************************************************************************/
cv::Mat TimeStamp::fill_avg(cv::Mat inp) const {
    cv::Mat roi = inp(ts_);
    roi.setTo(cv::mean(inp)[0]);
    return inp;
}

/********************************************************************
 * Stamp
********************************************************************/

/*****************************************************************************/
Stamp::Stamp(std::string stamp_in, unsigned int max_length, cv::Size img_size) : 
        stamp(stamp_in), max_length(max_length), img_size(img_size), 
        font_face(cv::FONT_HERSHEY_PLAIN), font_scale(1.0), color(cv::Scalar(255)), 
        thickness(1) {

    int buf;

    const int min_offset = 5;

    // remove whitespace in string
    auto f_isspace = [](unsigned char const c) { return std::isspace(c);};
    stamp_in.erase(std::remove_if(stamp_in.begin(), stamp_in.end(), f_isspace),
            stamp_in.end());

    // parse the items in the string ";" delimited
    std::vector<std::string> terms1;
    split_string(stamp_in, ';', terms1);

    // adjust max_length for long terms if needed
    // and split term if it is too long
    cv::Size text_sz;
    std::vector<std::string> splt_terms;
    std::vector<std::string> terms2;
    for (int i=0; i < terms1.size(); ++i) {
        text_sz = cv::getTextSize(terms1[i], font_face, font_scale, thickness, &buf);
        if (text_sz.width > (img_size.width-min_offset)) {
            // split term into multiple lines
            split_text(terms1[i], (img_size.width-min_offset), font_face, font_scale, 
                    thickness, splt_terms);
        
            // add split terms to terms2
            unsigned int intermediate_max = 0;
            for (int j=0; j < splt_terms.size(); ++j) {
                terms2.push_back(splt_terms[j]);
                text_sz = cv::getTextSize(splt_terms[j], font_face, font_scale, thickness,
                        &buf);
                if (text_sz.width > intermediate_max) {
                    intermediate_max = text_sz.width;
                }
            }

            if (intermediate_max > max_length) {
                max_length = intermediate_max;
            }

        } else {
            terms2.push_back(terms1[i]);
        
            if (text_sz.width > max_length) {
                max_length = text_sz.width;
            }
        }
    }

    // organize terms to fit in corner, maintain order
    unsigned int curr_len = 0;
    std::string term, term_space;
    for (int i=0; i < terms2.size(); ++i) {
        if (curr_len > 0) {
            term_space = " " + terms2[i];
        } else {
            term_space = terms2[i];
        }
        text_sz = cv::getTextSize(term_space, font_face, font_scale, thickness, &buf);
        if ( (curr_len + text_sz.width) <= max_length) {
            if (curr_len > 0) {
                term += " " + terms2[i];
            } else {
                term = terms2[i];
            }
            curr_len += text_sz.width;
        } else {
            terms.push_back(term);
            text_sz = cv::getTextSize(terms2[i], font_face, font_scale, thickness, &buf);
            curr_len = text_sz.width;
            term = terms2[i];
        }
    }
    terms.push_back(term);

    // create vector of initial term placements
    int start_u = img_size.width - max_length;
    int start_v = 0;
    for (int i=0; i < terms.size(); ++i) {
        text_sz = cv::getTextSize(terms[i], font_face, font_scale, thickness, &buf);
        start_v += text_sz.height + 1;
        start_pts.push_back(cv::Point2i(start_u, start_v));
    } 

}

/*****************************************************************************/
cv::Mat Stamp::operator()(const cv::Mat& inp) const {

    // convert to 8 bit if needed
    cv::Mat out;
    if (inp.depth() != CV_8U) {
        out = convert2_8U(inp);
    } else {
        out = inp.clone();
    }

    for (int i=0; i < terms.size(); ++i) {
        cv::putText(out, terms[i], start_pts[i], font_face, font_scale, color, thickness);
    }

    return out;
}

/********************************************************************
 * Functions
********************************************************************/

/*****************************************************************************/
void subtract_dc(cv::Mat& inp, cv::Rect roi) {
    if (inp.depth() != CV_64F) {
        inp.convertTo(inp, CV_64F);
    }

    if (roi.height == 0) {
        inp -= cv::mean(inp)[0];
    } else {
        inp -= cv::mean(inp(roi))[0];
    }
}

/*****************************************************************************/
void find_rectangles(const cv::Mat& inp, std::vector<cv::Rect>& rects) {

    // Convert the image to CV_8U
    cv::Mat img;
    if (inp.depth() != CV_8U) {
        img = convert2_8U(inp);
    } else {    
        img = inp.clone();
    }

    // Filter the image
    cv::GaussianBlur(img, img, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);

    // Binarize the image using an adaptive threshold
    cv::adaptiveThreshold(img, img, 255, cv::ADAPTIVE_THRESH_MEAN_C, 
            cv::THRESH_BINARY_INV, 3, 2); 

    /* Visualize Binarized Image 
    cv::String winname1 = "Binarized Image";
    cv::namedWindow(winname1);
    cv::imshow(winname1,img);
    cv::waitKey(0);
    */

    // Find Contours in the image
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Remove contours smaller than 100 pixels in area
    auto cit = contours.begin();
    while (cit != contours.end()) {
        if (cv::contourArea(*cit) < 100.0) {
            cit = contours.erase(cit);
        } else {
            ++cit;
        }
    }

    /* Visualize the contours
    std::cout << "Displaying " << contours.size() << " contours" << std::endl;
    cv::Mat img_color;
    cv::cvtColor(img, img_color, CV_GRAY2BGR);
    cv::drawContours(img_color, contours, -1, cv::Scalar(0,0,255), CV_FILLED, 8);
    cv::String winname2 = "Contours";
    cv::namedWindow(winname2);
    cv::imshow(winname2, img_color);
    cv::waitKey(0);
    */

    // Bound each contour with a rectangle
    rects.resize(0);
    for (int i=0; i < contours.size(); ++i) {
        rects.push_back(cv::boundingRect(contours[i]));
    }
    
    // Remove rectangles with large aspect ratio
    auto rit = rects.begin();
    cv::Rect r_tmp;
    while (rit != rects.end()) {
        r_tmp = *rit;
        if ( ( r_tmp.height / r_tmp.width > 40) || (r_tmp.width/r_tmp.height > 40) ) {
            rit = rects.erase(rit);
        } else {
            ++rit;
        }
    }

    /* Visualize the rectangles
    std::cout << "Displaying " << rects.size() << " rectangles" << std::endl;
    cv::Mat img_color2;
    cv::cvtColor(img, img_color2, CV_GRAY2BGR);
    for (int i=0; i < rects.size(); ++i) {
        cv::rectangle(img_color2, {rects[i].x,rects[i].y}, {rects[i].x+rects[i].width,
                rects[i].y+rects[i].height}, cv::Scalar(0,0,255), 2);
    }
    cv::String winname3 = "Rectangles";
    cv::namedWindow(winname3);
    cv::imshow(winname3, img_color2);
    cv::waitKey(0);
    */

}

/*****************************************************************************/
void colorize_image(const cv::Mat& input, cv::Mat& output, 
        int colormap /*=cv::COLORMAP_JET*/) {

    unsigned int colorbar_width = std::round(input.cols * 0.05);
    unsigned int gaps = std::round(colorbar_width * 0.25);

    int fontFace = cv::FONT_HERSHEY_PLAIN;
    int thickness = 2;
    double fontScale = 2.0;
    cv::Scalar font_color = cv::Scalar(0,0,0);

    // Get metrics
    double min, max;
    cv::minMaxLoc(input, &min, &max);

    // Convert to CV_8U if needed
    cv::Mat src;
    if (input.depth() != CV_8U) {
        src = convert2_8U(input);
    } else {
        src = input.clone();
    }

    // Get metrics
    double min_post, max_post;
    cv::minMaxLoc(src, &min_post, &max_post);

    // Create colorbar
    cv::Mat colorbar = cv::Mat::zeros(input.rows, colorbar_width, CV_8U);

    uchar* p_color;
    double val;
    for (int i=0; i < colorbar.rows; ++i) {
        p_color = colorbar.ptr<uchar>(i);
        val = (max_post - min_post) * (colorbar.rows-1 - i) / (colorbar.rows-1) + min_post ;
        for (int j= 0; j < colorbar.cols; ++j, ++p_color) {
            *p_color = (uchar) std::round(val);
        }
    }

    // Create colorized image
    cv::applyColorMap(src, src, colormap);
    cv::applyColorMap(colorbar, colorbar, colormap);

    // Define colorbar limit text sizes
    std::vector<double> limits = {max, (min + max)*0.5, min};
    std::vector<cv::Size> txt_size(limits.size());
    std::vector<std::string> txt(limits.size());
    std::ostringstream oss;
    int max_height=0, max_width=0;
    int baseLine;
    for (int i=0; i < limits.size(); ++i) {
        oss.str("");
        oss.clear();
        oss << std::setw(5) << std::right << std::setprecision(2) << std::fixed << limits[i];
        txt[i] = oss.str();
        txt_size[i] = cv::getTextSize(txt[i], fontFace, fontScale, thickness, &baseLine);
        max_height = std::max(max_height, txt_size[i].height);
        max_width = std::max(max_width, txt_size[i].width);
    }

    // Create final frame
    output = cv::Mat( input.rows, input.cols + gaps + 2 + colorbar_width + max_width, CV_8UC3, 
            cv::Scalar(255,255,255));

    cv::Mat frame = output(cv::Rect(0, 0, input.cols, input.rows));
    src.copyTo(frame);

    cv::Mat bar = output(cv::Rect(input.cols + gaps, 0, colorbar_width, input.rows));
    colorbar.copyTo(bar);

    // Add text
    cv::putText(output, txt[0], cv::Point(src.cols + gaps + colorbar_width + 1, 
            txt_size[0].height * 1.25), fontFace, fontScale, font_color, thickness);
    cv::putText(output, txt[1], cv::Point(src.cols + gaps + colorbar_width +1, 
            src.rows / 2 + txt_size[1].height / 2), fontFace, fontScale, font_color, thickness);
    cv::putText(output, txt[2], cv::Point(src.cols + gaps + colorbar_width +1, 
            src.rows - txt_size[2].height * 0.25), fontFace, fontScale, font_color, thickness);
        
}

/*****************************************************************************/
void get_colors(unsigned int count, std::vector<cv::Scalar>& colors, 
        int colormap /*=cv::COLORMAP_JET*/) {
    colors.resize(count);

    cv::Mat_<uchar> inp(count, 1);
    for (int i=0; i < count; ++i) {
        inp(i) = i * 256 / count;
    }

    cv::Mat output(count,3,CV_8U);
    cv::applyColorMap(inp, output, colormap);

    //std::cout << output << std::endl;

    for (unsigned int i=0; i < count; ++i) {
        colors[i] = cv::Scalar(output.at<uchar>(i,0),output.at<uchar>(i,1),output.at<uchar>(i,2));
    }

} 

} /* end namespace upsp */
