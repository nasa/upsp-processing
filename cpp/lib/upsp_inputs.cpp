/** @file
 *  @brief  Handle uPSP Processing Input File
 *  @date   August 30, 2017
 *  @author jmpowel2
 */

#include <assert.h>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <numeric>
#include <stdexcept>
#include <sys/stat.h>

#include "upsp_inputs.h"
#include "logging.h"

namespace upsp {

/*******************************************************************
 * FileInputs
********************************************************************/

/*****************************************************************************/
FileInputs::FileInputs() :
        test_id(""), 
        run(0), sequence(0), tunnel(""), cameras(0), sds(""), 
        grid(""), active_comps(""), out_name(""), normals(""), out_dir(""), 
        grid_type(GridType::None), target_patcher(TargetPatchType::None), 
        registration(RegistrationType::None), filter(FilterType::None), 
        overlap(OverlapType::AverageViews), filter_size(0), 
        oblique_angle(70), number_frames(0), grid_units("-") {}

bool FileInputs::Load(const std::string& input_file) {
    filename = input_file;
    // Open file
    std::ifstream fs(input_file);
    if (!fs.is_open()) {
        LOG_ERROR("Input file '%s': %s", input_file.c_str(), std::strerror(errno));
        return false;
    }

    // Setup holders for fillers
    bool fill_extras = false;
    std::string fill_targets = "";
    std::string fill_calibration = "";

    // Parse file
    std::string buf;
    while (getline(fs, buf)) {
        // strip leading whitespace
        buf.erase(buf.begin(), std::find_if_not(buf.begin(), buf.end(), isspace ));

        // Skip Comments
        if (buf[0] == '#') {
            continue;
        }

        // Check file meta data
        if (buf[0] == '%') {
            if (buf.find("Version") != std::string::npos) {
                std::vector<std::string> terms;
                split_whitespace(buf, terms);
                version = terms[1];
            }
            continue;
        }

        // Load input blocks
        if (buf.find("@general") != std::string::npos) {
            if (!load_general(fs)) return false;
        } else if (buf.find("@vars") != std::string::npos) {
            if (!load_vars(fs)) return false;
        } else if (buf.find("@all") != std::string::npos) {
            fill_extras = load_all(fs, fill_targets, fill_calibration);
        } else if (buf.find("@camera") != std::string::npos) {
            if (!load_camera(fs)) return false;
        } else if (buf.find("@options") != std::string::npos) {
            if (!load_options(fs)) return false;
        } else if (buf.find("@output") != std::string::npos) {
            if (!load_output(fs)) return false;
        }
    }

    fs.close();

    // Sort cameras by number, ascending
    if (!is_sorted(cam_nums.begin(), cam_nums.end())) {
        std::vector<size_t> idx(cameras);
        std::iota(idx.begin(), idx.end(), 0);
        auto& tmp_cams = cam_nums;
        std::sort(idx.begin(), idx.end(), [&tmp_cams](size_t i1, size_t i2) {
                return tmp_cams[i1] < tmp_cams[i2];});

        std::vector<unsigned int> tmp_cam_nums(cam_nums);
        std::vector<std::string> tmp_camera_filenames(camera_filenames);
        std::vector<std::string> tmp_targets(targets);
        std::vector<std::string> tmp_cals(cals);
        for (unsigned int i=0; i < cameras; ++i) {
            cam_nums[i] = tmp_cam_nums[idx[i]]; 
            camera_filenames[i] = tmp_camera_filenames[idx[i]]; 
            targets[i] = tmp_targets[idx[i]]; 
            cals[i] = tmp_cals[idx[i]]; 
        }
    }

    // Replace any vars with appropriate values
    if (vars.size() != 0) {
        if (!evaluate_vars(fill_targets)) {
            LOG_ERROR("Unable to parse variables in @all:targets");
            return false;
        }
        if (!evaluate_vars(fill_calibration)) {
            LOG_ERROR("Unable to parse variables in @all:calibration");
            return false;
        }
        if (!evaluate_vars(sds)) {
            LOG_ERROR("Unable to parse variables in @all:sds");
            return false;
        }
        if (!evaluate_vars(grid)) {
            LOG_ERROR("Unable to parse variables in @all:grid");
            return false;
        }
        if (!evaluate_vars(normals)) {
            LOG_ERROR("Unable to parse variables in @all:normals");
            return false;
        }
        if (!evaluate_vars(active_comps)) {
            LOG_ERROR("Unable to parse variables in @all:active_comps");
            return false;
        }
        if (!evaluate_vars(out_dir)) {
            LOG_ERROR("Unable to parse variables in @output:dir");
            return false;
        }

        assert(targets.size() == cameras);
        assert(cals.size() == cameras);
        assert(camera_filenames.size() == cameras);
        for (unsigned int i=0; i < cameras; ++i) {
            if (!evaluate_vars(targets[i])) {
                LOG_ERROR("Unable to parse variables in @camera:%d:targets", i + 1);
                return false;
            }
            if (!evaluate_vars(cals[i])) {
                LOG_ERROR("Unable to parse variables in @camera:%d:calibration", i + 1);
                return false;
            }
            if (!evaluate_vars(camera_filenames[i])) {
                LOG_ERROR("Unable to parse variables in @camera:%d:filename", i + 1);
                return false;
            }
        }
    }

    // Fill @all data into cameras with missing params
    if (fill_extras) {
        assert(targets.size() == cameras);
        assert(cals.size() == cameras);
        for (unsigned int i=0; i < cameras; ++i) {
            if (targets[i].empty()) {
                targets[i] = fill_targets;
            }
            if (cals[i].empty()) {
                cals[i] = fill_calibration;
            }
        }
    }

    return true;
}

/*****************************************************************************/
bool FileInputs::check_all() const {

    struct stat buffer;

    const size_t SZ = 200;
    char err[SZ];
    for (unsigned int i=0; i < cameras; ++i) {
        if (stat(camera_filenames[i].c_str(), &buffer) != 0) {
            LOG_ERROR("Camera %d file '%s': %s", i+1, camera_filenames[i].c_str(), std::strerror(errno));
            return false;
        }
        if (stat(targets[i].c_str(), &buffer) != 0) {
            LOG_ERROR("Targets file '%s': %s", targets[i].c_str(), std::strerror(errno));
            return false;
        }
        if (stat(cals[i].c_str(), &buffer) != 0) {
            LOG_ERROR("Calibration file '%s': %s", cals[i].c_str(), std::strerror(errno));
            return false;
        }
    }   

    if (stat(sds.c_str(), &buffer) != 0) {
        LOG_ERROR("SDS file '%s': %s", sds.c_str(), std::strerror(errno));
        return false;
    }
    if (stat(grid.c_str(), &buffer) != 0) {
        LOG_ERROR("Grid file '%s': %s", grid.c_str(), std::strerror(errno));
        return false;
    }
    if ( grid_type == GridType::None ) {
        LOG_ERROR("Invalid GridType::None");
        return false;
    }
    if (!normals.empty()) {
        if (stat(normals.c_str(), &buffer) != 0) {
            LOG_ERROR("Normals file '%s': %s", normals.c_str(), std::strerror(errno));
            return false;
        }
    }
    if (!active_comps.empty()) {
        if (stat(active_comps.c_str(), &buffer) != 0) {
            LOG_ERROR("Active components file '%s': %s", active_comps.c_str(), std::strerror(errno));
            return false;
        }
    }
    if (stat(out_dir.c_str(), &buffer) != 0) {
        LOG_ERROR("Output dir '%s': %s", out_dir.c_str(), std::strerror(errno));
        return false;
    }

    return true;
}

/*****************************************************************************/
bool FileInputs::has_normals() const {
    return !normals.empty();
}

/*****************************************************************************/
void FileInputs::write_file(std::string filename) const {

    // open file
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        throw(std::invalid_argument("Could not open file for writing input deck"));
    }

    // Write header
    if (!version.empty()) {
        ofs << "%Version " << version << std::endl;

        std::time_t t = time(0);
        std::tm* now = std::localtime(&t);
        ofs << "%Date_Created " << (now->tm_mon + 1) << "/" << (now->tm_mday) << "/";
        ofs << (now->tm_year + 1900) << std::endl;
        ofs << std::endl;
    }

    // Write general information
    ofs << "@general" << std::endl;
    ofs << "\ttest = " << test_id << std::endl;
    ofs << "\trun = " << run << std::endl;
    ofs << "\tsequence = " << sequence << std::endl;
    if (!tunnel.empty()) {
        ofs << "\ttunnel = " << tunnel << std::endl;
    }
    if (!grid_units.empty()) {
        ofs << "\tgrid_units = " << grid_units << std::endl;
    }

    // Write out any vars
    if (vars.size() > 0) {
        ofs << "@vars" << std::endl;
        assert(vars.size() == vars_map.size());
        for (unsigned int i=0; i < vars.size(); ++i) {
            ofs << "\t" << vars[i] << " = " << vars_map[i] << std::endl;
        }
    }

    // Write out common terms
    ofs << "@all" << std::endl;
    ofs << "\tgrid = " << refill_vars(grid) << std::endl;
    ofs << "\tsds = " << refill_vars(sds) << std::endl;
    if (!normals.empty()) {
        ofs << "\tnormals = " << refill_vars(normals) << std::endl;
    }
    if (!active_comps.empty()) {
        ofs << "\tactive_comps = " << refill_vars(active_comps) << std::endl;
    }

    bool targ_all = true;
    for (unsigned int i=1; i < targets.size(); ++i) {
        if (targets[i] != targets[0]) {
            targ_all = false;
            break;
        }
    }
    bool cal_all = true;
    for (unsigned int i=1; i < cals.size(); ++i) {
        if (cals[i] != cals[0]) {
            cal_all = false;
            break;
        }
    }

    if (targ_all) {
        ofs << "\ttargets = " << refill_vars(targets[0]) << std::endl;
    }
    if (cal_all) {
        ofs << "\tcalibration = " << refill_vars(cals[0]) << std::endl;
    }

    // Write out values for each camera
    assert((cameras == cam_nums.size()) && (cameras == camera_filenames.size()));

    for (unsigned int i=0; i < cameras; ++i) {
        ofs << "@camera" << std::endl;
        ofs << "\tnumber = " << cam_nums[i] << std::endl;
        ofs << "\tfilename = " << refill_vars(camera_filenames[i]) << std::endl;
        if (!targ_all) {
            ofs << "\ttargets = " << refill_vars(targets[i]) << std::endl;
        }
        if (!cal_all) {
            ofs << "\tcalibration = " << refill_vars(cals[i]) << std::endl;
        }
    }

    // Write out the options
    ofs << "@options" << std::endl;
    ofs << "\ttarget_patcher = " << target_patcher << std::endl;
    ofs << "\tregistration = " << registration << std::endl;
    ofs << "\tfilter = " << filter << std::endl;
    ofs << "\toverlap = " << overlap << std::endl;
    ofs << "\tfilter_size = " << filter_size << std::endl;
    ofs << "\toblique_angle = " << oblique_angle << std::endl;
    ofs << "\tnumber_frames = " << number_frames << std::endl;

    // Write out the output
    ofs << "@output" << std::endl;
    ofs << "\tdir = " << refill_vars(out_dir) << std::endl;
    ofs << "\tname = " << out_name << std::endl;

    ofs.close();
}

/*****************************************************************************/
bool FileInputs::load_camera(std::ifstream& fs) {
    ++cameras;

    // Set camera data with default values
    cam_nums.push_back(0);
    camera_filenames.push_back("");
    cals.push_back("");
    targets.push_back("");

    std::string buf;
    std::vector<std::string> tokens;
    int len = fs.tellg();
    while (getline(fs,buf)) {
        if (buf.find("@") != std::string::npos) {
            fs.seekg(len);
            return true;
        }

        // Get the tokens, and add input to vector
        parse_line(buf, tokens);
        if (tokens.size() == 2) {
            if (tokens[0] == "number") {
                try {
                    cam_nums[cameras-1] = std::stoi(tokens[1]);
                } catch(...) {
                    throw(std::invalid_argument("Could not parse " + 
                            std::string("camera number in input file")));
                }
            // todo-mshawlec: maintaining "cine" for backwards compatibility
            //                with legacy input file decks.
            } else if (tokens[0] == "cine" || tokens[0] == "filename") {
                camera_filenames[cameras-1] = tokens[1];
            } else if (tokens[0] == "calibration") {
                cals[cameras-1] = tokens[1];
            } else if (tokens[0] == "targets") {
                targets[cameras-1] = tokens[1];
            } else {
                LOG_WARNING("Skipping unknown token '%s'", tokens[0]);
            }
        }

        len = fs.tellg();
    }

    return true;
}

/*****************************************************************************/
bool FileInputs::load_output(std::ifstream& fs) {

    std::string buf;
    std::vector<std::string> tokens;
    int len = fs.tellg();
    while (getline(fs,buf)) {
        if (buf.find("@") != std::string::npos) {
            fs.seekg(len);
            return true;
        }

        parse_line(buf, tokens);
        if (tokens.size() == 2) {
            if (tokens[0] == "dir") {
                out_dir = tokens[1];
            } else if (tokens[0] == "name") {
                out_name = tokens[1];
            }
        }

        len = fs.tellg();
    }

    return true;
}

/*****************************************************************************/
bool FileInputs::load_all(std::ifstream&fs, std::string& fill_targets, 
        std::string& fill_cals) {
    bool fill_extras = false;

    std::string buf;
    std::vector<std::string> tokens;
    int len = fs.tellg();
    while (getline(fs,buf)) {
        if (buf.find("@") != std::string::npos) {
            fs.seekg(len);
            return fill_extras;
        }

        parse_line(buf, tokens);
        if (tokens.size() == 2) {
            if (tokens[0] == "sds") {
                sds = tokens[1];
            } else if (tokens[0] == "grid") {
                grid = tokens[1];
                // identify grid type
                std::string frmt = grid.substr(grid.find_last_of(".") + 1);
                if ( (frmt == "p3d") || (frmt == "g") || (frmt == "x") || (frmt == "grid") || (frmt == "grd") ) {
                    grid_type = GridType::P3D;
                }
                if (frmt == "tri") {
                    grid_type = GridType::Tri;
                }
            } else if (tokens[0] == "normals") {
                normals = tokens[1];
            } else if (tokens[0] == "targets") {
                fill_targets = tokens[1];
                fill_extras = true;
            } else if (tokens[0] == "calibration") {
                fill_cals = tokens[1];
                fill_extras = true;
            } else if (tokens[0] == "grid_units") {
                grid_units = tokens[1];
            } else if (tokens[0] == "active_comps") {
                active_comps = tokens[1];
            }
        }

        len = fs.tellg();
    }

    return fill_extras;
}

/*****************************************************************************/
bool FileInputs::load_general(std::ifstream& fs) {

    std::string buf;
    std::vector<std::string> tokens;
    int len = fs.tellg();
    while (getline(fs,buf)) {
        if (buf.find("@") != std::string::npos) {
            fs.seekg(len);
            return true;
        }

        parse_line(buf, tokens);
        if (tokens.size() == 2) {
            if (tokens[0] == "test") {
                test_id = tokens[1];
            } else if (tokens[0] == "run") {
                try {
                    run = stoi(tokens[1]);
                } catch(...) {
                    LOG_ERROR("Error: Could not parse @general:run. Expected integer");
                    fs.close();
                    return false;
                }
            } else if (tokens[0] == "sequence") {
                try {
                    sequence = stoi(tokens[1]);
                } catch(...) {
                    LOG_ERROR("Error: Could not parse @general:sequence. Expected integer");
                    fs.close();
                    return false;
                }
            } else if (tokens[0] == "tunnel") {
                tunnel = tokens[1];
            }
        }

        len = fs.tellg();
    }

    return true;
}

/*****************************************************************************/
bool FileInputs::load_vars(std::ifstream& fs) {
    std::string buf;
    std::vector<std::string> tokens;
    int len = fs.tellg();
    while (getline(fs,buf)) {
        if (buf.find("@") != std::string::npos) {
            fs.seekg(len);
            return true;
        }

        parse_line(buf, tokens);
        if (tokens.size() == 2) {
            vars.push_back(tokens[0]);
            vars_map.push_back(tokens[1]);
        }

        len = fs.tellg();
    }

    return true;
}

/*****************************************************************************/
bool FileInputs::load_options(std::ifstream& fs) {
    std::string buf;
    std::vector<std::string> tokens;
    int len = fs.tellg();
    while (getline(fs,buf)) {
        if (buf.find("@") != std::string::npos) {
            fs.seekg(len);
            return true;
        }

        parse_line(buf, tokens);
        if (tokens.size() == 2) {
            if (tokens[0] == "target_patcher") {
                if (tokens[1] == "polynomial") {
                    target_patcher = TargetPatchType::Polynomial;
                } else if (tokens[1] == "none") {
                    target_patcher = TargetPatchType::None;
                } else {
                    LOG_ERROR(
                        "Error: Could not parse @options:target_patcher. "
                        "Options are 'polynomial' or 'none'"
                    );
                    fs.close();
                    return false;
                }
            } else if (tokens[0] == "registration") {
                if (tokens[1] == "pixel") {
                    registration = RegistrationType::Pixel;
                } else if (tokens[1] == "point") {
                    registration = RegistrationType::Point;
                } else if (tokens[1] == "none") {
                    registration = RegistrationType::None;
                } else {
                    LOG_ERROR(
                        "Error: Could not parse @options:registration."
                        " Options are 'pixel', 'point', or 'none'"
                    );
                    fs.close();
                    return false;
                }
            } else if (tokens[0] == "filter") {
                if (tokens[1] == "gaussian") {
                    filter = FilterType::Gaussian;
                } else if (tokens[1] == "box") {
                    filter = FilterType::Box;
                } else if (tokens[1] == "none") {
                    filter = FilterType::None;
                } else {
                    LOG_ERROR(
                        "Error: Could not parse @options:filter."
                        " Options are 'gaussian', 'box', or 'none'"
                    );
                    fs.close();
                    return false;
                }
            } else if (tokens[0] == "overlap") {
                if (tokens[1] == "best_view") {
                    overlap = OverlapType::BestView;
                } else if (tokens[1] == "average_view") {
                    overlap = OverlapType::AverageViews;
                } else {
                    LOG_ERROR(
                        "Error: Could not parse @options:overlap."
                        " Options are 'best_view' or 'average_view'"
                    );
                    fs.close();
                    return false;
                }
            } else if (tokens[0] == "filter_size") {
                try {
                    filter_size = stoi(tokens[1]);
                } catch(...) {
                    LOG_ERROR(
                        "Error: Could not parse @options:filter_size."
                        " Expected integer"
                    );
                    fs.close();
                    return false;
                }
            } else if (tokens[0] == "oblique_angle") {
                try {
                    oblique_angle = stof(tokens[1]);
                } catch(...) {
                    LOG_ERROR(
                        "Error: Could not parse @options:oblique_angle."
                        " Expected float"
                    );
                    fs.close();
                    return false;
                }
            } else if (tokens[0] == "number_frames") {
                try {
                    number_frames = stoi(tokens[1]);
                } catch(...) {
                    LOG_ERROR(
                        "Error: Could not parse @options:number_frames."
                        " Expected integer"
                    );
                    fs.close();
                    return false;
                }
            }
        }

        len = fs.tellg();
    }

    return true;
}

/*****************************************************************************/
void FileInputs::parse_line(std::string input, std::vector<std::string>& tokens) {
    tokens.clear();

    // remove whitespace
    input.erase(std::remove_if(input.begin(), input.end(), isspace ), input.end() );

    // split on '='
    split_string(input, '=', tokens);
}

/*****************************************************************************/
bool FileInputs::evaluate_vars(std::string& term) {
    if (term.empty()) {
        return true;
    }

    std::string var;
    size_t pos_start;
    unsigned int num_vars = num_substr(term, "$");
    for (unsigned int j=0; j < num_vars; ++j) {
        bool replaced = false;
        for (unsigned int i=0; i < vars.size(); ++i) {
            var = "$" + vars[i];
            pos_start = term.find(var);

            if (pos_start != std::string::npos) {
                term.replace(pos_start, pos_start + var.size(), vars_map[i]);
                replaced = true;
                break;
            }
        }
        if (!replaced) {
            std::cout << "check 1" << std::endl;
            return false;
        }
    }
    if (term.find("$") != std::string::npos) {
        return false;
    } else {
        return true;
    }
}

/*****************************************************************************/
unsigned int FileInputs::num_substr(std::string str, std::string substr) {
    if (substr.size() == 0) {
        return 0;
    }

    size_t pos = str.find(substr, 0);
    unsigned int count = 0;
    while (pos != std::string::npos) {
        pos += substr.size();
        ++count;
        pos = str.find(substr, pos);
    }
    return count;
}

/*****************************************************************************/
std::string FileInputs::refill_vars(std::string term) const {

    if (vars.size() == 0) {
        return term;
    }

    assert(vars.size() == vars_map.size());

    // Find the match with the longest size string
    int best_match = -1;
    int best_size = 0;
    for (unsigned int i=0; i < vars.size(); ++i) {
        std::size_t fidx = term.find(vars_map[i]);
        if (fidx != std::string::npos) {
            if (vars_map[i].size() > best_size) {
                best_match = i;
                best_size = vars_map[i].size();
            }
        }
    }

    // If a match was found, replace with variable name
    if (best_match != -1) {
        term.replace(term.find(vars_map[best_match]),
                vars_map[best_match].length(), "$"+vars[best_match]);
    }

    return term;
}

/*******************************************************************
 * Functions
********************************************************************/

/*****************************************************************************/
std::ostream& operator<<(std::ostream& os, const FileInputs& fi) {
    os << std::endl;
    os << "Input File " << fi.filename << ":" << std::endl;
    os << fi.test_id << " Run " << fi.run << " Seq " << fi.sequence << std::endl;
    os << fi.tunnel << std::endl << std::endl;

    os << "grid         = " << fi.grid << std::endl;
    os << "    units    = " << fi.grid_units << std::endl;

    if (!fi.normals.empty()) {
        os << "normals      = " << fi.normals << std::endl;
    }
    if (!fi.active_comps.empty()) {
        os << "active_comps = " << fi.active_comps << std::endl;
    }
    os << "sds          = " << fi.sds << std::endl;
    os << std::endl;
    
    if (fi.cameras > 1) {
        os << fi.cameras << " Cameras:" << std::endl;
    } else {
        os << fi.cameras << " Camera:" << std::endl;
    }
    for (unsigned int i=0; i < fi.cameras; ++i) {
        os << " camera " << fi.cam_nums[i] << std::endl;
        os << "           filename    = " << fi.camera_filenames[i] << std::endl;
        os << "           targets     = " << fi.targets[i] << std::endl;
        os << "           calibration = " << fi.cals[i] << std::endl;
    }

    os << std::endl;
    os << "output dir  = " << fi.out_dir << std::endl;
    os << "output name = " << fi.out_name << std::endl; 

    os << std::endl;
    os << "Options:" << std::endl;
    os << "  Target Patcher   = " << fi.target_patcher << std::endl;
    os << "  Registration     = " << fi.registration << std::endl;
    os << "  Filter           = " << fi.filter << " (" << fi.filter_size << "x";
    os << fi.filter_size << ")" << std::endl;
    os << "  Overlap          = " << fi.overlap << std::endl;
    os << "  Oblique Angle    = " << fi.oblique_angle << std::endl;
    os << "  Number of Frames = " << fi.number_frames << std::endl;

    return os;
}

/*****************************************************************************/
std::ostream& operator<<(std::ostream& os, const TargetPatchType& tpt) {
    switch (tpt) {
        case (TargetPatchType::None) :
            os << "none";
            break;
        case (TargetPatchType::Polynomial) :
            os << "polynomial";
            break;
        default :
            os << "undefined target patch type";
            break;
    }
    return os;
}

/*****************************************************************************/
std::ostream& operator<<(std::ostream& os, const RegistrationType& rt) {
    switch (rt) {
        case (RegistrationType::None) :
            os << "none";
            break;
        case (RegistrationType::Point) :
            os << "point";
            break;
        case (RegistrationType::Pixel) :
            os << "pixel";
            break;
        default :
            os << "undefined registration type";
            break;
    }
    return os;
}

/*****************************************************************************/
std::ostream& operator<<(std::ostream& os, const FilterType& ft) {
    switch (ft) {
        case (FilterType::None) :
            os << "none";
            break;
        case (FilterType::Gaussian) :
            os << "gaussian";
            break;
        default :
            os << "undefined filter type";
            break;
    }
    return os;
}

/*****************************************************************************/
std::ostream& operator<<(std::ostream& os, const OverlapType& ot) {
    switch (ot) {
        case (OverlapType::BestView) :
            os << "best_view";
            break;
        case (OverlapType::AverageViews) :
            os << "average_views";
            break;
        default :
            os << "undefined overlap type";
            break;
    }
    return os;
}

} /* end namespace upsp */
