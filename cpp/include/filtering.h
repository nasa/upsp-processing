/** @file
 *  @brief  Filtering and Curve Fitting
 *  @date   Nov 20, 2018
 *  @author jmpowel2
 */

#ifndef UTILS_FILTERING_H__
#define UTILS_FILTERING_H__

#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <iterator>
#include <vector>

namespace upsp {

/** Create and apply polynomial fits to chunks of nodes
 *  intended to be used when polynomial is created and immediately
 *  evaluated for some number of data points
 */
template<typename T>
struct TransPolyFitter {

    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> PolyMat;

    /** Create invalid transpose polyfitter object */
    TransPolyFitter() {} 

    /** Initialize transpose polyfitter object
     *
     * @param[in] n_frames  number of frames to process for each data point
     * @param[in] degree    order of the polynomial
     * @param[in] n_pts     number of data points that will be evaluated
     *
     */
    TransPolyFitter(unsigned int n_frames, unsigned int degree, 
            unsigned int n_pts);

    /** Initialize a transpose polyfitter object
     *
      * @param[in] filename     file with the polynomial fit coefficients
      * @param[in] n_frames     number of frames to process for each data point
      *
      */
    TransPolyFitter(std::string filename, unsigned int n_frames); 

    /** Create polynomial fit and evaluate for some chunk of nodes
     *
     * @param[in] data          Chunk data point values to be fit (n_pts x n_frames)
     * @param[in] n_pts         Chunk number of points to process
     * @param[in] curr_pt       index (into total number of points) of ths first data
     *                          point in this chunk (0 := first point)
     *
     * @return                  polynomial value for each point, for each frame
     *                          (n_pts x n_frames)
     *
     * @pre @a data.size() % n_frames_ == 0
     */
    std::vector<T> eval_fit(T* data, unsigned int n_pts, unsigned int curr_pt);  

    /** Skip node at index 'curr_pt', store 0s for poly fit */
    void skip_fit(unsigned int curr_pt) { 
        for (unsigned int c=0; c < coeffs_; ++c) {
            poly_(c,curr_pt) = 0.0;
        }
    }

    /** Write out the fit coefficients */
    void write_coeffs(std::string filename) const;

    /** Read in the fit coefficients */
    void read_coeffs(std::string filename);

    /*******************************************/

    PolyMat A_;
    PolyMat poly_;

    unsigned int coeffs_;
    unsigned int n_frames_;
    unsigned int n_pts_;

    bool read_fits_; // if loaded fits from file, do not need to recreate
};

/** Create polynomial fits many times with the same x values and degree
 *  intended to be used when polynomial must be evaluated on a frame-by-frame
 *  basis
 */
template<typename T>
struct PolyFitter {

    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> PolyMat;

    /** Create invalid polyfitter object */
    PolyFitter();

    /** Initialize a polyfitter object
      * @param[in] x_begin  iterator to the beginning of the independent parameters
      * @param[in] length   length of @a x_begin
      * @param[in] degree   order of the polynomial
      * @param[in] nodes    number of terms that will receive a fit
      * @param[in] incr     step between elements of @a x_begin to include in the fit
      *
      * @pre std::advance(@a x_begin, @a incr) @a length times produces a valid T
      */
    template<typename InputIterator>
    PolyFitter(InputIterator x_begin, unsigned int length, unsigned int degree, 
        unsigned int nodes, unsigned int incr=1);

    /** Initialize a polyfitter object
      * @param[in] filename file with the polynomial fit coefficients
      * @param[in] x_begin  iterator to the beginning of the independent parameters
      * @param[in] length   length of @a x_begin
      * @param[in] incr     step between elements of @a x_begin to include in the fit
      *
      * @pre std::advance(@a x_begin, @a incr) @a length times produces a valid T
      */
    template<typename InputIterator>
    PolyFitter(std::string filename, InputIterator x_begin, unsigned int length, 
        unsigned int incr=1);
    
    /** Fit a polynomial to the data and return R^2 */
    template<typename InputIterator>
    T fit_data(InputIterator y_begin, unsigned int node);

    /** Evaluate each polynomial for each node for a single frame */
    void polyval(float x, std::vector<T>& out);

    /** Write out the fit coefficients */
    void write_coeffs(std::string filename) const;

    /** Read in the fit coefficients */
    void read_coeffs(std::string filename);

    /** Get the polynomial coefficients for a node */
    std::vector<T> get_coeffs(unsigned int node);

    PolyMat A_;
    Eigen::ColPivHouseholderQR<PolyMat> dec_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> fits_; // nodes_ X coeffs_
    unsigned int length_;
    unsigned int coeffs_;
    unsigned int nodes_;
    unsigned int incr_;

};

/** Create a polynomial fit
 * 
 * @param[in] x_begin   iterator to the beginning of x coordinates
 * @param[in] y_begin   iterator to the beginning of y coordinates
 * @param[in] length    number of elements in x and y
 * @param[in] degree    degree of polynomial 
 * @return              polynomial coefficients
 *
 * @pre @a x_begin can be incremented @a length times
 * @pre @a y_begin can be incremented @a length times
 * @pre @a length >= degree + 1
 */
template<typename InputIterator, typename T>
std::vector<T> polyfit(InputIterator x_begin, InputIterator y_begin, unsigned int length, 
        unsigned int degree);

/** Create a polynomial fit
 * 
 * @param[in] x         vector of x coordinates
 * @param[in] y         vector of y coordinates
 * @param[in] degree    degree of polynomial 
 * @return              polynomial coefficients
 *
 * @pre @a x.size() == @a y.size()
 * @pre @a x.size() >= degree + 1
 */
template<typename T>
std::vector<T> polyfit(const std::vector<T>& x, const std::vector<T>& y, 
        unsigned int degree);

/** Evaluate a polynomial at an x coordinate
 *
 * @param[in] x     x position
 * @param[in] poly  polynomial coefficients
 * @return          polynomial value at x
 */
template<typename T>
T polyval(T x, const std::vector<T>& poly);

template<typename T>
std::vector<T> polyval(const std::vector<T>& x, const std::vector<T>& poly);

} /* end namespace upsp */

#include "../lib/filtering.ipp"

#endif /* UTILS_FILTERING_H_ */
