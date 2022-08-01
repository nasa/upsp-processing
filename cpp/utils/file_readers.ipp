/** @file
 *  @brief  Read uPSP Files
 *  @date   September 15, 2017
 *  @author jmpowel2
 */
#include "logging.h"
#include "TriModel.h"

namespace upsp_files {

/*****************************************************************************/
template<typename M>
bool set_surface_normals(const std::string& normal_file, M& model) {

    std::ifstream ifs_in(normal_file);
    if (!ifs_in) {
        throw(std::invalid_argument("Cannot open surface normal csv file"));
    }

    auto f_isspace = [](unsigned char const c) { return std::isspace(c);};

    std::string line;
    std::vector<std::string> terms;
    std::getline(ifs_in, line);

    // Parse the header
    line.erase(std::remove_if(line.begin(), line.end(), f_isspace), line.end());
    split_string(line, ',', terms);
    int nidx_i = -1, x_norm_i = -1, y_norm_i = -1, z_norm_i = -1;
    for (unsigned int i=0; i < terms.size(); ++i) {
        if (terms[i] == "nidx") {
            nidx_i = i;
        } else if (terms[i] == "x_norm") {
            x_norm_i = i;
        } else if (terms[i] == "y_norm") {
            y_norm_i = i;
        } else if (terms[i] == "z_norm") {
            z_norm_i = i;
        }
    }
    if (nidx_i == -1) {
        throw(std::invalid_argument("Could not parse nidx in normal csv file"));
    }
    if (x_norm_i == -1) {
        throw(std::invalid_argument("Could not parse x_norm in normal csv file"));
    }
    if (y_norm_i == -1) {
        throw(std::invalid_argument("Could not parse y_norm in normal csv file"));
    }
    if (z_norm_i == -1) {
        throw(std::invalid_argument("Could not parse z_norm in normal csv file"));
    }

    // Parse the normal vectors
    typedef typename M::data_type FP;
    typedef typename M::node_idx node_idx;

    cv::Point3_<FP> normal;
    node_idx nidx;
    int nset = 0;
    while(std::getline(ifs_in, line)) {
        // remove whitespace
        line.erase(std::remove_if(line.begin(), line.end(), f_isspace), line.end());

        // split by ,
        split_string(line, ',', terms);

        // get values
        try {
            nidx = std::stoi(terms[nidx_i]);
            normal.x = std::stod(terms[x_norm_i]);
            normal.y = std::stod(terms[y_norm_i]);
            normal.z = std::stod(terms[z_norm_i]);
        } catch (...) {
            LOG_ERROR("Cannot parse normal csv file'%s'", normal_file.c_str());
            return false;
        }

        // update model
        model.set_normals(nidx, normal);
        nset += 1;
    }

    LOG_DEBUG(
        "upsp::set_surface_normals('%s'): Overwrote %d/%d normals",
        normal_file.c_str(), nset, model.size()
    );

    return true;
}

template<typename FP>
bool set_surface_normals(const std::string& normal_file, upsp::TriModel_<FP>& model) {
    LOG_ERROR(
        "Refusing to read '%s'; can not specify normals CSV for a TriModel_",
        normal_file.c_str()
    );
    return false;
}

/*****************************************************************************/
template<typename T>
bool read_target_file(cv::String target_file, std::vector<T>& targs) {

    std::ifstream ifs_in(target_file);
    if (!ifs_in) {
        LOG_ERROR("Cannot open '%s'", target_file.c_str());
        return false;
    }
    
    targs.clear();

    auto f_isspace = [](unsigned char const c) { return std::isspace(c);};

    bool has_diam = false;
    int extra_terms = 1;
    std::vector<std::string> terms;
    int targ_id;
    double x, y, z, u, v;
    std::string line;
    int count = 0, pt_size;
    while(std::getline(ifs_in, line)) {
        // remove whitespace
        line.erase(std::remove_if(line.begin(), line.end(), f_isspace), line.end());

        // split the different terms
        split_string(line, ',', terms); 

        // check for header
        if (line[0] == '#') {
            // parse the header to determine if diameter is included
            for (int i=0; i < terms.size(); ++i) {
                if (terms[i] == "diameter") {
                    has_diam = true;
                    ++extra_terms;
                    break;
                }
            }
            continue;
        }

        if (count == 0) {
            if (terms.size() == 3) {
                pt_size = 2;
                if (has_diam) {
                    LOG_ERROR("Target reader does not support this file type");
                    return false;
                }
            } else if (terms.size() == 4) {
                if (has_diam) {
                    pt_size = 2;
                } else {
                    pt_size = 3;
                }
            } else if (terms.size() == 5) {
                pt_size = 3;
                if (!has_diam) {
                    LOG_ERROR("Target reader does not support this file type");
                    return false;
                }
            } else {
                LOG_ERROR("Target reader does not support this file type");
                return false;
            }
        }

        if (terms.size() != (pt_size+extra_terms)) {
            LOG_ERROR("Target data file has inconsistent number of columns");
            return false;
        }

        // convert to terms to appropriate type
        T tmp_targ;
        try {
            tmp_targ.num = std::stoi(terms[0]);
            if (pt_size == 2) {
                tmp_targ.uv.x = std::stod(terms[1]);
                tmp_targ.uv.y = std::stod(terms[2]);
                if (has_diam) {
                    tmp_targ.diameter = std::stod(terms[3]);
                }
            }
            if (pt_size == 3) {
                tmp_targ.xyz.x = std::stod(terms[1]);
                tmp_targ.xyz.y = std::stod(terms[2]);
                tmp_targ.xyz.z = std::stod(terms[3]);
                if (has_diam) {
                    tmp_targ.diameter = std::stod(terms[4]);
                }
            }
        } catch (...) {
            LOG_ERROR("Cannot parse target data file '%s'", target_file.c_str());
            return false;
        }
        
        // add point to targs
        targs.push_back(tmp_targ);
        
        ++count;
    } 
    
    ifs_in.close();
    return true;
}

/*****************************************************************************/
template<typename T>
bool read_psp_target_file(cv::String target_file, std::vector<T>& targs, 
        bool planar/*=false*/, cv::String label/*="*Targets"*/) {

    std::ifstream ifs_in(target_file);
    if (!ifs_in.is_open()) {
        LOG_ERROR("Cannot open '%s'", target_file.c_str());
        return false;
    }

    if (targs.size() > 0) {
        targs.clear();
    }

    std::string line;
    std::stringstream ss;
    int count=0, targ_id;
    double tmp, x, y, z, diam;
    while (getline(ifs_in,line)) {
        if (line.find(label) != std::string::npos) {
            while (getline(ifs_in, line)) {
                // Check for another label (begins with *)
                if (line[0] == '*') {
                    break;
                }

                // Process Line
                ss.str(std::string());
                ss.clear();
                ss.str(line);
        
                targ_id = 0;
                ss >> targ_id;

                targs.push_back(T());
                ss >> x >> y >> z >> tmp >> tmp >> tmp >> diam;
                if (planar) z = 0.0;
                targs[count].xyz = cv::Point3d(x,y,z);
                targs[count].num = targ_id;
                targs[count].diameter = diam;

                ++count;
            }
            break;
        }
    }

    ifs_in.close();
    return true;
}

} /* end namespace upsp_files */

