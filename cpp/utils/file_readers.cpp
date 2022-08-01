/** @file
 *  @brief  Read uPSP Files
 *  @date   August 28, 2019
 *  @author jmpowel2
 */

#include "utils/file_readers.h"

namespace upsp_files {

/*****************************************************************************/
std::unordered_map<int,bool> read_active_comp_file(std::string comp_file) {

    std::ifstream ifs_in(comp_file);
    if (!ifs_in) {
        throw(std::invalid_argument("Cannot open active component csv file"));
    }

    std::unordered_map<int,bool> active_comps;

    std::string line;
    std::getline(ifs_in, line); // get the header line

    // Parse all of the components
    std::vector<std::string> terms;
    int comp = 0;
    unsigned int active = 0;
    while (std::getline(ifs_in, line)) {
        // split by ,
        split_string(line, ',', terms);

        // get values
        try {
            comp   = std::stoi(terms[0]);
            active = std::abs(std::stoi(terms[1]));
        } catch (...) {
            throw(std::invalid_argument("Cannot parse active component csv file"));
        }

        if (active == 0) {
            active_comps[comp] = false;
        } else {
            active_comps[comp] = true;
        }
    }

    return active_comps;
}



} /* end namespace upsp_files */
