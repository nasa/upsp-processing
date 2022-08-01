/** @file
 *
 * @brief   General Utilities
 */

#ifndef UTILS_GENERAL_UTILS_H_
#define UTILS_GENERAL_UTILS_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#define TWO_POW(p) (1 << (p))
#ifndef PI
#define PI 3.141592653589793
#endif
#define MM_TO_INCHES 0.03937007874

/** Define operator!= from a class's operator==
 *
 * Usage:
 *  class MyClass : private equality_comparable<MyClass> {
 *    bool operator==(const MyClass& c) { ... }
 *  };
 */
template<typename T>
struct equality_comparable {
    friend bool operator!=(const T& a, const T& b) {return !(a == b); }
};

/** Derive operator>, operator<=, and operator>= form a class's operator<
 *
 * Usage:
 *  class MyClass : private less_than_comparable<MyClass> {
 *      bool operator<(const MyClass& c) { ... }
 *  };
 */
template <typename T>
struct less_than_comparable {
    friend bool operator> (const T& a, const T& b) { return   b < a; }
    friend bool operator<=(const T& a, const T& b) { return !(b < a); }
    friend bool operator>=(const T& a, const T& b) { return !(a < b); }
};

/** Derive operator!=, operator>, operator<=, and operator>= from a class's
 * operator< and operator==
 *
 * Usage:
 *  class MyClass : private totally_ordered<MyClass> {
 *      bool operator< (const MyClass& c) { ... }
 *      bool operator==(const MyClass& c) { ... }
 *  };
 */
template<typename T>
struct totally_ordered : less_than_comparable<T>, equality_comparable<T> {
};

/** Circular Array, where the array is a cycle 
 *  elements should be unique
 */
template<typename T, size_t Length>
class CircularArray : private equality_comparable<CircularArray<T,Length>> {
public:
    
    /** Create an empty CircularArray */
    CircularArray() {}

    /** Create an initialized CircularArray */
    CircularArray(std::initializer_list<T> il) {
        assert(il.size() == Length);
        std::copy(il.begin(), il.end(), arr_.begin());
    }

    /** Copy CircularArray */
    CircularArray& operator=(const CircularArray& ca) {
        arr_ = ca.arr_;
        return *this;
    }

    /** Return the element at a given index */
    T& operator[](size_t n) {
        return arr_[n % Length];
    }
    
    /** Return the element at a given index */
    const T& operator[](size_t n) const {
        return arr_[n % Length];
    }

    /** Return the size of the array */
    size_t size() const {
        return Length;
    }

    /** Check for equality 
     *
     * @pre     for all i,j 0 <= i,j < Length and i != j, this->[](i) != this->[](j)
     * @return  True if this->size() == ca.size() and there exists i,j s.t.
     *          this->[](i+n) == ca[j+n] for 0 <= n < this->size()   
     */
    bool operator==(const CircularArray& ca) const;

private:

    std::array<T,Length> arr_;

};

/** Add circular array to stream */
template<typename T, size_t Length>
std::ostream& operator<<(std::ostream& os, const CircularArray<T,Length>& carr);

/** Compute the factorial */
unsigned int factorial(unsigned int n);

/** Split a string by a delimiter into a vector
 *
 * @param[in]       str     input string
 * @param[in]       delim   delimiter
 * @param[in,out]   terms   output segments of the string
 */
void split_string(std::string str, char delim, std::vector<std::string>& terms);

/** Split a string on whitespace
 *
 * @param[in]       str     input string
 * @param[in,out]   terms   output segments of the string
 */
void split_whitespace(std::string str, std::vector<std::string>& terms);

/** Converts radians to degrees
 *
 * @param[in] angle an angle in radians
 * @return          @a angle in degrees
 */
template<typename T>
T rad2_deg(T angle);

/** Converts degrees to radians
 *
 * @param[in] angle an angle in degrees
 * @return          @a angle in radians
 */
template<typename T>
T deg2_rad(T angle);

/** Determine which of the 3 doubles is the minimum
 *
 * @param[in] p0    first value
 * @param[in] p1    second value
 * @param[in] p2    third value
 * @return          index of min value 0 if p0, 1 if p1, 2 if p2
 */
template<typename T>
unsigned int min_ind(T p0, T p1, T p2);

/** Determine which of the 3 doubles is the maximum
 *
 * @param[in] p0    first value
 * @param[in] p1    second value
 * @param[in] p2    third value
 * @return          index of max value 0 if p0, 1 if p1, 2 if p2
 */
template<typename T>
unsigned int max_ind(T p0, T p1, T p2);

/** Find the minimum of 3 doubles
 *
 * @param[in] p1    first value
 * @param[in] p2    second value
 * @param[in] p3    third value
 * @return          minimum of (p1,p2,p3)
 */
template<typename T>
T min(T p1, T p2, T p3);

/** Find the maximum of 3 doubles
 *
 * @param[in] p1    first value
 * @param[in] p2    second value
 * @param[in] p3    third value
 * @return          maximum of (p1,p2,p3)
 */
template<typename T>
T max(T p1, T p2, T p3);

/** Erase a vector of indices from another vector
 *
 * @param[in] data      vector to delete from
 * @param[in] idcs      sorted (small to large) indices to delete
 * @return data without idcs
 */
template<typename T>
std::vector<T> erase_indices(const std::vector<T>& data, const std::vector<unsigned int>& idcs);

/** Round up to nearest multiple
 *
 * @param[in] input     number to round
 * @param[in] multiple  
 */
int round_up(int input, int multiple);

/** Round down to nearest multiple
 *
 * @param[in] input     number to round
 * @param[in] multiple  
 */
int round_down(int input, int multiple);

/** Round down to nearest mutliple
 *
 * @param[in] input     number to round
 * @param[in] multiple  
 */
unsigned int round_down(unsigned int input, unsigned int multiple);

/* Transpose a block of a matrix 
 *
 * @param[in] A             input matrix
 * @param[out] B            output matrix 
 * @param[in] rows          number of rows in A
 * @param[in] cols          number of columns in A
 * @param[in] block_size    size of the block to transpose
 *
 * @post will transpose block_sizexblock_size of A into B, starting at 
 *       A,B pointers
 */
template<typename T>
inline void transpose_scalar_block(T* A, T* B, unsigned int rows,
        unsigned int cols, unsigned int block_size);

/* Transpose matrix 
 *
 * @param[in] A             input matrix
 * @param[out] B            output matrix 
 * @param[in] rows          number of rows in A
 * @param[in] cols          number of columns in A
 * @param[in] block_size    size of block increments (transpose 1 block
 *                          at a time)
 *
 * @post will transpose A into B
 */
template<typename T>
inline void transpose_block(T* A, T* B, unsigned int rows,
        unsigned int cols, unsigned int block_size);

#include "./general_utils.ipp"

#endif /* UTILS_GENERAL_UTILS_H_ */
