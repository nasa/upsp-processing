/** @file
 *  @brief Simple Data Structure Definitions (no openCV)
 *  @date May 29, 2018
 *  @author jmpowel2
 */

#include "non_cv_upsp.h"

#include <stdexcept>

namespace upsp {

/********************************************************************
 * PaintCalibration
********************************************************************/

/*****************************************************************************/
PaintCalibration::PaintCalibration(std::string filename) : a(0.0), b(0.0), c(0.0),
        d(0.0), e(0.0), f(0.0) {

    std::ifstream ifs(filename);
    if (!ifs) {
        throw(std::invalid_argument("Cannot read paint calibration file"));
    }

    // read in calibration coefficients
    std::string buf;
    std::vector<std::string> tokens;
    while (getline(ifs, buf)) {
        // get the variable/value pairs
        buf.erase(std::remove_if(buf.begin(), buf.end(), isspace), buf.end() );
        split_string(buf, '=', tokens);

        // parse the variable
        if (tokens.size() == 2) {
            float var = 0;
            try {
                var = stod(tokens[1]);
            } catch(...) {
                std::cerr << "Error: Could not parse coefficient " << tokens[0];
                std::cerr << ".  Expected float" << std::endl;
                ifs.close();
                std::abort();
            }
        
            if (tokens[0] == "a") {
                a = var;
            } else if (tokens[0] == "b") {
                b = var;
            } else if (tokens[0] == "c") {
                c = var;
            } else if (tokens[0] == "d") {
                d = var;
            } else if (tokens[0] == "e") {
                e = var;
            } else if (tokens[0] == "f") {
                f = var;
            }
        }
    }

    ifs.close();
}

/*****************************************************************************/
float PaintCalibration::get_gain(float T, float Pss) {
    return a + b*T + c*T*T + (d + e*T + f*T*T)*Pss;
}

/*****************************************************************************/
std::ostream& operator<<(std::ostream& os, const PaintCalibration& pcal) {
    os << "a + b*T + c*T*T + (d + e*T + f*T*T)*Pss" << std::endl << std::endl;
    os << "a = " << pcal.a << std::endl;
    os << "b = " << pcal.b << std::endl;
    os << "c = " << pcal.c << std::endl;
    os << "d = " << pcal.d << std::endl;
    os << "e = " << pcal.e << std::endl;
    os << "f = " << pcal.f << std::endl;
    return os;
}

/********************************************************************
  * TunnelConditions
 ********************************************************************/

/*****************************************************************************/
TunnelConditions::TunnelConditions() : 
        alpha(std::numeric_limits<float>::quiet_NaN()),
        beta(std::numeric_limits<float>::quiet_NaN()),
        phi(std::numeric_limits<float>::quiet_NaN()),
        mach(std::numeric_limits<float>::quiet_NaN()),
        rey(std::numeric_limits<float>::quiet_NaN()),
        ptot(std::numeric_limits<float>::quiet_NaN()),
        qbar(std::numeric_limits<float>::quiet_NaN()),
        ttot(std::numeric_limits<float>::quiet_NaN()),
        ps(std::numeric_limits<float>::quiet_NaN()),
        tcavg(std::numeric_limits<float>::quiet_NaN()),
        test_id(""), run(0), seq(0) {}

/*****************************************************************************/
bool TunnelConditions::operator==(const TunnelConditions& tc) {
    return ( (alpha == tc.alpha) && (beta == tc.beta) && (phi == tc.phi) &&
            (mach == tc.mach) && (rey == tc.rey) && (ptot == tc.ptot) &&
            (qbar == tc.qbar) && (ttot == tc.ttot) && (ps == tc.ps) &&
            (test_id == tc.test_id) && (run == tc.run) && (seq == tc.seq) );
}

/*****************************************************************************/
TunnelConditions read_tunnel_conditions(const std::string& filename) {

    std::ifstream ifs(filename);
    if (!ifs) {
        throw(std::invalid_argument("Cannot open '" + filename + "'"));
    }

    upsp::TunnelConditions cond;

    std::string line;
    std::vector<std::string> terms;
    while (std::getline(ifs, line)) {
        // split the terms on whitespace
        split_whitespace(line, terms);

        // check if header
        if (terms.size() > 0) {
            if ( (terms[0].length() == 1) && (terms[0][0] == '#') ) {

                // get the term values
                std::vector<std::string> vals;
                std::getline(ifs, line);
                split_whitespace(line, vals);

                if (vals.size() != (terms.size()-1)) {
                    std::cerr << "Failed to parse '" << filename << "'" << std::endl;
                    std::abort();
                }

                // pull out the values of interest
                for (int i=1; i < terms.size(); ++i) {
                    try {
                        if (terms[i] == "ALPHA") {
                            cond.alpha = stod(vals[i-1]);
                        } else if (terms[i] == "BETA") {
                            cond.beta = stod(vals[i-1]);
                        } else if (terms[i] == "PHI") {
                            cond.phi = stod(vals[i-1]);
                        } else if (terms[i] == "MACH") {
                            cond.mach = stod(vals[i-1]);
                        } else if (terms[i] == "RNU") {
                            cond.rey = stod(vals[i-1]);
                        } else if (terms[i] == "PTOT") {
                            cond.ptot = stod(vals[i-1]);
                        } else if (terms[i] == "Q") {
                            cond.qbar = stod(vals[i-1]);
                        } else if (terms[i] == "TTF") {
                            cond.ttot = stod(vals[i-1]);
                        } else if (terms[i] == "PS") {
                            cond.ps = stod(vals[i-1]);
                        } else if (terms[i] == "TCAVG") {
                            cond.tcavg = stod(vals[i-1]);
                        }
                    } catch (...) {
                        std::cerr << "Unable to parse variable " << terms[i];
                        std::cerr << " in '" << filename << "'" << std::endl;
                    }
                }

                break;
            }
        }
    }

    ifs.close();

    // check if conditions are missing
    if (std::isnan(cond.alpha)) {
        std::cout << "Warning: ALPHA was not read from '" << filename << "'" << std::endl;
    }
    if (std::isnan(cond.beta)) {
        std::cout << "Warning: BETA was not read from '" << filename << "'" << std::endl;
    }
    if (std::isnan(cond.phi)) {
        std::cout << "Warning: PHI was not read from '" << filename << "'" << std::endl;
    }
    if (std::isnan(cond.mach)) {
        std::cout << "Warning: MACH was not read from '" << filename << "'" << std::endl;
    }
    if (std::isnan(cond.rey)) {
        std::cout << "Warning: REYN was not read from '" << filename << "'" << std::endl;
    }
    if (std::isnan(cond.ptot)) {
        std::cout << "Warning: PTOT was not read from '" << filename << "'" << std::endl;
    }
    if (std::isnan(cond.qbar)) {
        std::cout << "Warning: Q was not read from '" << filename << "'" << std::endl;
    }
    if (std::isnan(cond.ttot)) {
        std::cout << "Warning: TTOT was not read from '" << filename << "'" << std::endl;
    }
    if (std::isnan(cond.ps)) {
        std::cout << "Warning: PS was not read from '" << filename << "'" << std::endl;
    }

    return cond;
}

/********************************************************************
  * CameraSettings
 ********************************************************************/

/*****************************************************************************/
CameraSettings::CameraSettings() : framerate(0), fstop(0.0), exposure(0.0) {}

/*****************************************************************************/
bool CameraSettings::operator==(const CameraSettings& cs) {
    if ( (framerate != cs.framerate) || (fstop != cs.fstop) || 
            (exposure != cs.exposure) ) {
        return false;
    }
    if ( (focal_lengths.size() != cs.focal_lengths.size()) || 
            (cam_nums.size() != cs.cam_nums.size()) ) {
        return false;
    }
    for (unsigned int i=0; i < focal_lengths.size(); ++i) {
        if (focal_lengths[i] != cs.focal_lengths[i]) {
            return false;
        }
    }
    for (unsigned int i=0; i < cam_nums.size(); ++i) {
        if (cam_nums[i] != cs.cam_nums[i]) {
            return false;
        }
    }
    return true;
}

} /* end namespace upsp */
