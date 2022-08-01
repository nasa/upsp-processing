/** @file
 *  @brief  Handle uPSP Processing Input File
 *  @date   August 30, 2017
 *  @author jmpowel2
 */

#ifndef UFML_UPSP_INPUTS_H_
#define UFML_UPSP_INPUTS_H_

#include <algorithm>
#include <assert.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <vector>

#include "utils/general_utils.h"

namespace upsp {

/** Method for patching over a target */
enum class TargetPatchType { None, Polynomial };

/** Registration method */
enum class RegistrationType { None, Point, Pixel };

/** Type of spatial filter */
enum class FilterType { None, Gaussian, Box };

/** Type of grid file */
enum class GridType { None, P3D, Tri };

/** Method for handling multi-view texture mapping */
enum class OverlapType { BestView, AverageViews };

/** Load and manipulate input parameters */
class FileInputs {
public:

    FileInputs(); 

    /** Load all of the input parameters */
    bool Load(const std::string& input_file); 

    /** Return true if all files and output directory exist */
    bool check_all() const;

    /** Return true if normals are included in the input file */
    bool has_normals() const;

    /** Write out the input deck to a file */
    void write_file(std::string filename) const;

    /***************************************************************/

    std::string filename;

    std::string test_id;
    unsigned int run;
    unsigned int sequence;
    std::string tunnel;

    unsigned int cameras;
    std::vector<unsigned int> cam_nums;
    std::vector<std::string> camera_filenames;
    std::vector<std::string> targets;
    std::vector<std::string> cals;
    std::string sds;
    std::string grid;
    std::string active_comps;
    std::string normals;
    std::string out_name;
    std::string out_dir;

    GridType grid_type;
    TargetPatchType target_patcher;
    RegistrationType registration;
    FilterType filter;
    OverlapType overlap;
    unsigned int filter_size;
    float oblique_angle;
    int number_frames;

    std::string grid_units;

private:

    /** Parse a "camera" block
     *
     * @param[in] fs   file stream
     */
    bool load_camera(std::ifstream& fs);

    /** Parse the "output" block
     *
     * @param[in] fs   file stream
     */
    bool load_output(std::ifstream& fs);

    /** Parse the "all" block 
     * 
     * @param[in] fs            file stream
     * @param[out] fill_targets if target file in block, return it
     * @param[out] fill_cals        if calibration file in block, return it
     * return                       true if target file or calibration file in block
     */
    bool load_all(std::ifstream&fs, std::string& fill_targets, std::string& fill_cals);

    /** Parse the "general" block
     *
     * @param[in] fs   file stream
     */
    bool load_general(std::ifstream& fs);

    /** Parse the "vars" block
     * 
     * @param[in] fs        file stream
     */
    bool load_vars(std::ifstream& fs);

    /** Parse the "options" block
     *
     * @param[in] fs        file stream
     */
    bool load_options(std::ifstream& fs);
                                                                                         
    /** Parse a line with form 'term = input' and return [term,input] in vector
     *
     * @param[in] input         string to split
     * @param[out] tokens       [term,input]
     */
    void parse_line(std::string input, std::vector<std::string>& tokens);

    /** Replace any variables with their values
     *
     * @param[in,out] term     string to fill in with variables
     * @return     true if successfully replaced all vars
     */
    bool evaluate_vars(std::string& term);

    /** Compute number of instances of a substring within a string
     *
     * @param[in] str      string to evaluate
     * @param[in] substr   substring to find in @a str
     * @return             number of times @a substr occurs in @a str
     */
    unsigned int num_substr(std::string str, std::string substr);
                                                                                         
    /** Replace portion of string with variable if applicable
     *
     * @param[in] term      string that potentially contains a variable value
     * @return      term with variable included if there is a match
     */
    std::string refill_vars(std::string term) const;

    /***************************************************************/

    std::string version;

    std::vector<std::string> vars;
    std::vector<std::string> vars_map;

};

/** Add details of the input file to stream */
std::ostream& operator<<(std::ostream& os, const upsp::FileInputs& fi);

/** Add target patch type string to stream */
std::ostream& operator<<(std::ostream& os, const upsp::TargetPatchType& tpt);

/** Add registration type string to stream */
std::ostream& operator<<(std::ostream& os, const upsp::RegistrationType& rt);

/** Add filter type string to stream */
std::ostream& operator<<(std::ostream& os, const upsp::FilterType& ft);

/** Add overlap type string to stream */
std::ostream& operator<<(std::ostream& os, const upsp::OverlapType& ot);

} /* end namespace upsp */

#endif /* UFML_UPSP_INPUTS_H_ */
