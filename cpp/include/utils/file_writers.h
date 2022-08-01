/** @file
 *  @brief  Write uPSP Files
 *  @date   January 9, 2020
 *  @author mshawlec
 */

#ifndef UFML_UTILS_FILE_WRITERS_H_
#define UFML_UTILS_FILE_WRITERS_H_

#include <string>
#include <vector>

namespace upsp {

/** Write a std::vector<float> to a flat file
 *
 * @parma[in] fname  filename
 * @param[in] v      vector to write
 * @param[in] maxels if >0, then max number of vector
 *                   elements to write. Otherwise, entire
 *                   vector is written.
 */
int fwrite(const std::string&, const std::vector<float>&, const int);

}

#endif
