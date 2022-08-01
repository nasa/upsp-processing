/** @file
 *  @brief  1D Clustering
 *  @date   April 26, 2018
 *  @author jmpowel2
 */

#ifndef UFML_UTILS_CLUSTERING_H_
#define UFML_UTILS_CLUSTERING_H_

#include <algorithm>
#include <boost/iterator/counting_iterator.hpp>
#include <cmath>
#include <numeric>
#include <vector>

#include <iostream>

namespace upsp {

/** Find peaks in data
 *
 * @param[in] data          vector of 1D data
 * @param[out] peaks        vector of indices in data where there are peaks
 * @param[in] separation    minimum number of indices between adjacent peaks
 */
template<typename T>
void find_peaks(std::vector<T> data, std::vector<unsigned int>& peaks, 
        unsigned int separation = 0);

/** Determine threshold based on 1 main black peak
 *
 * @param[in] counts        histogram counts
 * @param[in] separation    separation distance in bins of adjacent peaks
 * @return                  break point index, 0 if no valid point
 */
template<typename T>
unsigned int first_min_threshold(const std::vector<T>& counts, 
        unsigned int separation = 1);

/** Get the Otsu threshold from histogram data
 *
 * @param[in] counts    histogram counts
 * @return              break point index
 */
template<typename T>
unsigned int otsu_threshold(const std::vector<T>& counts);

} /* end namespace upsp */

#include "../utils/clustering.ipp"

#endif /* UFML_UTILS_CLUSTERING_H_ */
