/** @file
 *  @brief  Help Reading Binary Data
 *  @date   December 18, 2019
 *  @author jmpowel2
 */

#ifndef UFML_UTILS_FILE_IO_H_
#define UFML_UTILS_FILE_IO_H_

/* Swap bytes, switch endianness 
 *
 * @param[in,out] p     swap bytes for each element in this array
 * @param[in] n_bytes   number of bytes in each element
 *
 */
void swap_bytes(char *p, unsigned int n_bytes);


#endif /* UFML_UTILS_FILE_IO_H_ */


