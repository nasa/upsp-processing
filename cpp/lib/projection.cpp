/** @file 
 *  @brief  Project between Image Frame and Model
 *  @date   March 22, 2018
 *  @author jmpowel2
 */

#include "projection.h"

/*****************************************************************************/
bool upsp::contains(const cv::Size& sz, const cv::Point2i& pt) {
    if ( (pt.x >= 0) && (pt.y >= 0) && (pt.x < sz.width) && (pt.y < sz.height) ) {
        return true;
    } else {
        return false;
    }
}
