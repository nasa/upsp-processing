/** @file
 *  @brief  Test the filtering.h functions
 *  @date   September 9, 2019
 *  @author jmpowel2
 */
#include "gtest/gtest.h"

#include "../include/filtering.h"


/* TODO: Still need tests for:
 *
 * polyfit 
 * polyval
 * 
 */

/** Test the transpose polyfitter */
TEST(FilteringHTest, TransPolyfitter) {

    float epsilon = 1e-4;

    const std::string out_file = "outputs/trans_polyfits.dat";

    unsigned int degree = 6;
    unsigned int n_frames = 25;
    unsigned int n_pts = 13;

    unsigned int coeffs = degree + 1;

    // create sample normalized frame vector
    std::vector<float> x(n_frames);
    for (unsigned int f=0; f < n_frames; ++f) {
        x[f] = (float) f / (float) n_frames;
    }

    // define reference coefficients unique for each pt
    // [pts x coeffs]
    std::vector<float> true_coeffs(coeffs*n_pts);
    
    for (unsigned int c=0; c < coeffs; ++c) {
        for (unsigned int p=0; p < n_pts; ++p) {
            true_coeffs[p*coeffs+c] = 2.5/(c+1) + p/(c+1);
        }
    }

    // define data (defined by true_coeffs)
    // transpose data is [pts x frames]
    std::vector<float> y(n_frames*n_pts, 0);
    for (unsigned int p=0; p < n_pts; ++p) {
        for (unsigned int f=0; f < n_frames; ++f) {
            for (unsigned int c=0; c < coeffs; ++c) {
                y[p*n_frames+f] += std::pow(x[f], c) * true_coeffs[p*coeffs+c];
            }
        }
    }

    // initialize polynomial fitter
    upsp::TransPolyFitter<float> pfitter(n_frames, degree, n_pts);

    // create fit and evaluate for blocks of data points
    unsigned int c_pts = 3;
    for (unsigned int p=0; p < n_pts; p+=c_pts) {
    
        unsigned int p_start = p;
        unsigned int p_end   = std::min(n_pts-1, p+c_pts-1);

        // copy over this chunk of data
        std::vector<float> c_data(y.begin() + n_frames*p_start, 
                                  y.begin() + n_frames*(p_end+1));

        // create fit and evaluate
        std::vector<float> y2 = pfitter.eval_fit(&c_data[0], p_end - p_start + 1, p_start);

        // compare results
        unsigned int count=0;
        for (unsigned int p=p_start; p <= p_end; ++p, ++count) {
            for (unsigned int f=0; f < n_frames; ++f) {
                ASSERT_GT(std::abs(y[p*n_frames+f] - y2[count*n_frames+f]), epsilon);
            }
        }
    }

    // write out polynomial fit
    pfitter.write_coeffs(out_file);

    // read in polynomial fit
    upsp::TransPolyFitter<float> pfitter2(out_file, n_frames);

    // create fit and evaluate for blocks of data points
    // with read in polynomial fits
    for (unsigned int p=0; p < n_pts; p+=c_pts) {
    
        unsigned int p_start = p;
        unsigned int p_end   = std::min(n_pts-1, p+c_pts-1);

        // copy over this chunk of data
        std::vector<float> c_data(y.begin() + n_frames*p_start, 
                                  y.begin() + n_frames*(p_end+1));

        // create fit and evaluate
        std::vector<float> y2 = pfitter.eval_fit(&c_data[0], p_end - p_start + 1, p_start);

        // compare results
        unsigned int count=0;
        for (unsigned int p=p_start; p <= p_end; ++p, ++count) {
            for (unsigned int f=0; f < n_frames; ++f) {
                ASSERT_LT(std::abs(y[p*n_frames+f] - y2[count*n_frames+f]), epsilon);
            }
        }
    }

}

/** Test polyfitter */
TEST(FilteringHTest, Polyfitter) {

    float epsilon = 1e-2;
    float epsilon2 = 1e-4;

    int degree = 6;
    unsigned int len = 25;
    unsigned int nodes = 13;

    // create sample data
    std::vector<float> x(len);
    for (unsigned int i=0; i < len; ++i) {
        x[i] = (float) i / (float) len;
    }

    std::vector<float> true_coeffs(degree+1);
    for (unsigned int i=0; i < degree+1; ++i) {
        true_coeffs[i] = 2.5/(i+1);
    }

    std::vector<float> y(len*nodes, 0);
    for (unsigned int i=0; i < nodes; ++i) {
        for (unsigned int j=0; j < len; ++j) {
            y[i*len+j] = i;
            for (unsigned int c=0; c < degree+1; ++c) {
                y[i*len+j] += std::pow(x[j], c) * true_coeffs[c];
            }
        }
    }

    // initialize polynomial fitter
    upsp::PolyFitter<float> pfitter;
    pfitter = upsp::PolyFitter<float>(x.begin(), len, degree, nodes, 1);

    // solve for the coefficients for each node
    std::vector<float> coeffs;
    for (unsigned int i=0; i < nodes; ++i) {
        // perform the fit and get the coefficients
        pfitter.fit_data(y.begin() + i*len, i);
        coeffs = pfitter.get_coeffs(i);

        // compare to true coeffs
        for (unsigned int c=0; c < degree+1; ++c) {
            ASSERT_LT(std::abs(true_coeffs[c] - coeffs[c]),  epsilon);
        }

        // test fit evaluation
        std::vector<float> y2 = upsp::polyval(x, true_coeffs);
        ASSERT_EQ(y2.size(), len);
        for (unsigned int j=0; j < len; ++j) {
            //ASSERT_LT(std::abs(y2[j] - y[i*len+j]), epsilon2);
            ASSERT_FLOAT_EQ(y2[j], y[i*len+j]);
        }

        true_coeffs[0] += 1; // true_coeffs[0] = true_coeffs[0] + i 
    }

    // evaluate fits using polyfitter
    std::vector<float> y3(nodes);
    for (unsigned int i=0; i < len; ++i) {
        // evaluate all nodes
        pfitter.polyval(x[i], y3);

        // compare to true y
        for (unsigned int j=0; j < nodes; ++j) {
            ASSERT_LT(std::abs(y[j*len+i] - y3[j]), epsilon2);
        }
    }
}
