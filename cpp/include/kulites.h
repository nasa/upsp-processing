/** @file
 *  @brief  Process Virtual Kulite Data 
 *  @date   October 18, 2017
 *  @author jmpowel2
 */

#ifndef UFML_KULITES_H_
#define UFML_KULITES_H_

#include <opencv2/opencv.hpp>
#include <vector>

#include "data_structs.h"

namespace upsp {

/** @brief Location of Virtual Kulite 
 *
 * Location of virtual kulite relative to kulite in image frame 
 */
enum VKulDirection { TOP, BOTTOM, LEFT, RIGHT, UPSTREAM, DOWNSTREAM };

/** Convert a string with a direction into a VKulDirection
 *
 * @param[in] dir   direction string
 * @return          direction in integral form
 */
VKulDirection parse_virtual_kulite_direction(std::string dir);

/** Define the location of the virtual kulites relative to the kulite
 *
 * @param[in,out] kuls      kulites to find virtual kulites around
 * @param[in] dir       direction relative to the image frame to place the virtual kulite
 * @parm[in] stand_off  distance in kulite diameters away from center of kulite to place
 *                      virtual kulite
 * @param[in] set_size  if > 0, will set every kulite to the given size in pixels, else
 *                      will use kuls[i].size
 *
 * @post for all i, 0 <= i < kuls.size(), kuls[i].top_left will be defined
 */ 
void locate_virtual_kulites(std::vector<Kulite>& kuls, VKulDirection dir = LEFT, 
        double stand_off = 1.5, int set_size = -1);

/** Define the location of the virtual kulites relative to the kulite
 *
 * @param[in,out] kuls      kulites to find virtual kulites around
 * @param[in] dir       direction relative to the image frame to place the virtual kulite
 * @parm[in] stand_off  distance in kulite diameters away from center of kulite to place
 *                      virtual kulite
 * @param[in] set_size  if > 0, will set every kulite to the given size in pixels, else
 *                      will use kuls[i].size
 *
 * @post for all i, 0 <= i < kuls.size(), kuls[i].top_left will be defined
 */ 
void locate_virtual_kulites(std::vector<Kulite>& kuls, cv::Point2d dir, 
        double stand_off = 1.5, int set_size = -1);

/** Take the mean of all pixels within the virtual kulite
 *
 * @param[in] src       frame from which to extract the virtual kulites
 * @param[in,out] kuls  kulites to process
 *
 * @pre for all i, 0 <= i < kuls.size(), 
 *          0 <= kuls[i].top_left.x < src.width - kuls[i].size and
 *          0 <= kuls[i].top_left.y < src.height - kuls[i].size
 * @post for all i, 0 <= i < kuls.size(), kuls[i].value is the mean of the 
 *      virtual kulite pixels
 */ 
void extract_virtual_kulites(const cv::Mat& src, std::vector<Kulite>& kuls);

/** Check if a virtual kulite is entirely within a frame
 *
 * @param[in] kul       kulite with vritual kulite location
 * @param[in] sz        Size of the frame
 * @return              true if virtual kulite is entirely in frame, else false
 */
bool in_frame(Kulite& kul, cv::Size sz);


} /* end namespace upsp */


#endif /* UFML_KULITES_H_ */
