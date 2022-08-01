/** @file
 *  @brief  Patches over Image Targets
 *  @date   August 22, 2017
 *  @author jmpowel2
 */

#include "patches.h"

namespace upsp {

/********************************************************************
 * PatchTargets
********************************************************************/

/*****************************************************************************/
PatchTargets::PatchTargets(const std::vector<Target>& targs): targs_(targs) {}

/*****************************************************************************/
cv::Mat PatchTargets::operator()(cv::Mat inp) const {
    fill_data_gaps(inp, targs_);
    return inp;
}

/********************************************************************
 * Functions
********************************************************************/

/*****************************************************************************/
void fill_data_gaps(cv::Mat& img, const std::vector<Target>& targs, 
        int bound_pts/*=2*/) {

    // convert img to type double if needed
    if (img.depth() != CV_64F) {
        img.convertTo(img, CV_64F);
    }

    // get image properties
    unsigned int im_height = img.rows;
    unsigned int im_width = img.cols;

    unsigned int height, width, num_pts, count, count2;
    std::vector<double> x1, y1, x2, y2, z, poly;
    cv::Point2i min_pt, max_pt;
    for (int i=0; i < targs.size(); ++i) {

        // Bound the target with a rectangle
        min_pt.x = std::floor(targs[i].uv.x - 0.5*targs[i].diameter);
        min_pt.y = std::floor(targs[i].uv.y - 0.5*targs[i].diameter);
        max_pt.x = std::ceil(targs[i].uv.x + 0.5*targs[i].diameter);
        max_pt.y = std::ceil(targs[i].uv.y + 0.5*targs[i].diameter);

        // determine number of points that will be included in fit and initialize
        // vectors
        height = max_pt.y - min_pt.y + 1;
        width = max_pt.x - min_pt.x + 1;
        num_pts = (height+2*bound_pts)*(width+2*bound_pts) - height*width;
        x1.resize(num_pts);     
        y1.resize(num_pts);     
        z.resize(num_pts);      
        x2.resize(height*width);
        y2.resize(height*width);

        // check that target +/- bounds is within image
        assert( (min_pt.x-bound_pts >= 0) && (min_pt.y-bound_pts >= 0) );
        assert( (max_pt.x+bound_pts < im_width) && (max_pt.y+bound_pts <  im_height) );
    
        // Fill vectors with points around the square
        count = 0;  
        count2 = 0;
        for (int j=min_pt.x-bound_pts; j <= max_pt.x+bound_pts; ++j) {
            for (int k=min_pt.y-bound_pts; k <= max_pt.y+bound_pts; ++k) {
                if ( (j < min_pt.x) || (j > max_pt.x) || 
                        (k < min_pt.y) || (k > max_pt.y)) {
                    assert(count < num_pts);
                    x1[count] = (double) j;
                    y1[count] = (double) k;
                    z[count] = img.at<double>(k,j);
                    ++count;
                } else {
                    assert(count2 < height*width);
                    x2[count2] = (double) j;
                    y2[count2] = (double) k;
                    ++count2;
                }
            }
        }

        // Get the 2D polynomial that fits this data
        polyfit2D(x1, y1, z, poly, 3);

        // Evaluate the polynomial inside the target rectangle
        polyval2D(x2, y2, poly, z);

        // Update the img with patched data
        count2 = 0;
        for (int j=min_pt.x; j <= max_pt.x; ++j) {
            for (int k=min_pt.y; k <= max_pt.y; ++k) {
                img.at<double>(k,j) = z[count2];
                ++count2;
            }
        }
    }

}

} /* end namespace upsp */  
