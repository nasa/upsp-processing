/** @file
 *  @brief  Help Reading Binary Data
 *  @date   December 18, 2019
 *  @author jmpowel2
 */

#include "utils/file_io.h"

/*****************************************************************************/
void swap_bytes(char *p, unsigned int n_bytes) {
    char tmp[n_bytes];
    int i;
    int ii;

    ii = n_bytes - 1;
    for (i = 0; i < n_bytes; ++i) {
        tmp[i] = p[ii];
        --ii;
    }

    for (i = 0; i < n_bytes; ++i) {
        p[i] = tmp[i];
    }
}



