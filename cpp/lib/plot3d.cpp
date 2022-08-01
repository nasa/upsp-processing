#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "plot3d.h"
#include "grids.h"

namespace upsp {

std::string type_of(int *v) { return "int"; }
std::string type_of(float *v) { return "float32"; }

template <typename T>
bool read_record(std::ifstream& ifs, T* dst, size_t nv) {
    size_t number_bytes = sizeof(T) * nv;
    ifs.read((char*) dst, number_bytes);
    if (ifs) return true;
    else return false;
}

// Read record from an unformatted FORTRAN file.
// "with_seps": read FORTRAN record separators (leading + trailing int32's
//              that indicate the size of the record in bytes).
template <typename T>
bool read_record(std::ifstream& ifs, T* dst, size_t nv, bool with_seps) {
    if (with_seps) {
        int32_t sep;
        int32_t sep_expect = nv * sizeof(T);
        if (!read_record(ifs, &sep, 1)) return false;
        if (sep != sep_expect) return false;
        if (!read_record(ifs, dst, nv)) return false;
        if (!read_record(ifs, &sep, 1)) return false;
        if (sep != sep_expect) return false;
    } else {
        if (!read_record(ifs, dst, nv)) return false;
    }
    return true;
}

// Read scalars from an unformatted FORTRAN file.
std::string read_plot3d_scalar_function_file(std::string filename, std::vector<float>& sol, bool seps) {
    sol.clear();
    std::ifstream ifs(filename, std::ifstream::binary);
    if (!ifs) return std::string("Failed to open file");
    int number_zones = 0;
    int total_number_vertices = 0;
    // get the number of zones
    if (!read_record(ifs, &number_zones, 1, seps)) {
        return std::string("Failed to parse number of zones");
    }
    // get the size of the zones
    std::vector<int32_t> zone_sizes_flat(number_zones * 4);
    if (!read_record(ifs, (int32_t*) &zone_sizes_flat[0], number_zones * 4, seps)) {
        return std::string(
            "Failed to parse zone sizes (expected " +
            std::to_string(number_zones) + " zones)"
        );
    }
    for (int ii = 0; ii < number_zones; ii++) {
        const int i = zone_sizes_flat[ii * 4];
        const int j = zone_sizes_flat[ii * 4 + 1];
        const int k = zone_sizes_flat[ii * 4 + 2];
        const int n = zone_sizes_flat[ii * 4 + 3];
        total_number_vertices += (i * j * k);
    }
    // Read the solution for each zone
    sol.resize(total_number_vertices);
    if (!read_record(ifs, (float*) &sol[0], total_number_vertices)) {
        return std::string(
            "failed to read scalars (expected " +
            std::to_string(number_zones) + " zones, " + 
            std::to_string(total_number_vertices) + " scalars)"
        );
    }
    ifs.close();
    return "";
}

std::vector<float> read_plot3d_scalar_function_file(std::string filename, int record_seps) {
    std::vector<float> sol;
    std::string with_seps_err("");
    std::string without_seps_err("");
    std::string err("");
    if (record_seps == +1 || record_seps == -1) {
        with_seps_err = read_plot3d_scalar_function_file(filename, sol, true);
        if (with_seps_err.empty()) return std::move(sol);
        else err.append(
            std::string("\nAssuming *has* FORTRAN record separators: ") + with_seps_err
        );
    }
    if (record_seps == 0 || record_seps == -1) {
        without_seps_err = read_plot3d_scalar_function_file(filename, sol, false);
        if (without_seps_err.empty()) return std::move(sol);
        else err.append(
            std::string("\nAssuming *no* FORTRAN record separators: ") + without_seps_err
        );
    }
    throw(std::invalid_argument(
        "Failed to parse Plot3D function file '" + filename + "':" + err + "\n"
    ));
}

template<typename StructGrid>
void read_plot3d_grid_file(std::string filename, StructGrid& grid) {

    // Open file for reading (binary)
    std::ifstream ifs(filename, std::ios::in | std::ios::binary);
    if (!ifs) {
        throw std::invalid_argument("Could not open plot3d file " + filename);
    }

    // Pull out the data type
    typedef typename StructGrid::data_type FP;

    // This is fortran style (each block has int32 at beginning and end
    // for size of the block -- record marker)
    int32_t buf;

    // Read the first record marker and determine if the file is
    // big or little endian (following N. Pruitt)
    bool big_endian = false;

    ifs.read((char*) &buf, sizeof(int32_t));
    char* cp = (char*) &buf;
    if ( (cp[0] == 0) && (cp[3] != 0) ) {
        big_endian = true;
    } else if ((cp[0] == 0) && (cp[3] == 0)) {
        throw std::invalid_argument("Unable to identify plot3d endianness "
                " in file " + filename);
    }

    if (big_endian) swap_bytes( (char*) &buf, sizeof(int32_t));

    // Read the number of zones
    int32_t zones;
    bool single_p3d = false;
    if (buf == (sizeof(int32_t))) {
        ifs.read((char*) &zones, sizeof(int32_t));

        if (big_endian) swap_bytes( (char*) &zones, sizeof(int32_t));

        ifs.read((char*) &buf, sizeof(int32_t));
    } else {
        if (buf != (sizeof(int32_t)*3)) {
            ifs.close();
            throw std::invalid_argument("Unexpected plot3d file format "
                    " in file " + filename);
        }

        zones = 1;
        single_p3d = true;
    }
    
    // Read in the size of the zones
    grid.grid_size.resize(zones);
    int total_size = 0;
    if (!single_p3d) {
        ifs.read((char*) &buf, sizeof(int32_t));
    }
    for (unsigned int i=0; i < zones; ++i) {
        grid.grid_size[i].resize(3);
        ifs.read((char*) &grid.grid_size[i][0], sizeof(int32_t)*3);
        
        if (big_endian) {
            swap_bytes( (char*) &grid.grid_size[i][0], sizeof(int32_t));
            swap_bytes( (char*) &grid.grid_size[i][1], sizeof(int32_t));
            swap_bytes( (char*) &grid.grid_size[i][2], sizeof(int32_t));
        }

        total_size += grid.zone_size(i);
    }
    ifs.read((char*) &buf, sizeof(int32_t));

    // Resize vectors to hold all grid points
    grid.x.resize(total_size);
    grid.y.resize(total_size);
    grid.z.resize(total_size);

    // Read in the grid coordinates
    int32_t data_sz;
    bool has_iblank = false;
    int32_t idx = 0;
    for (unsigned int i=0; i < zones; ++i) {
        int32_t n_pts = grid.zone_size(i);
    
        // Check for single or double precision file
        // and check if IBLANK is included    
        ifs.read((char*) &buf, sizeof(int32_t));
        if (i == 0) {
            if (big_endian) swap_bytes( (char*) &buf, sizeof(int32_t));
            if (buf == (3*sizeof(float)*n_pts)) {
                data_sz = sizeof(float);
            } else if (buf == (3*sizeof(float)*n_pts + sizeof(int32_t)*n_pts)) {
                data_sz = sizeof(float);
                has_iblank = true;
            } else if (buf == (3*sizeof(double)*n_pts)) {
                data_sz = sizeof(double);
                has_iblank = false;
            } else if (buf == (3*sizeof(double)*n_pts + sizeof(int32_t)*n_pts)) {
                data_sz = sizeof(double);
                has_iblank = true;
            } else {
                throw std::invalid_argument("Unrecognized data type in plot3d" 
                        " file " + filename);
            }
        }

        if (data_sz == sizeof(FP)) {
            //std::cout << "reading " << n_pts << " data points" << std::endl;
            ifs.read((char*) &grid.x[idx], sizeof(FP)*n_pts);
            ifs.read((char*) &grid.y[idx], sizeof(FP)*n_pts);
            ifs.read((char*) &grid.z[idx], sizeof(FP)*n_pts);
        } else {
            if (data_sz == sizeof(float)) {
                std::vector<float> tmp_float(n_pts);
                ifs.read((char*) &tmp_float[0], sizeof(float)*n_pts);
                for (unsigned int j=0; j < n_pts; ++j) {
                    grid.x[idx+j] = static_cast<double>(tmp_float[j]);
                }
                ifs.read((char*) &tmp_float[0], sizeof(float)*n_pts);
                for (unsigned int j=0; j < n_pts; ++j) {
                    grid.y[idx+j] = static_cast<double>(tmp_float[j]);
                }
                ifs.read((char*) &tmp_float[0], sizeof(float)*n_pts);
                for (unsigned int j=0; j < n_pts; ++j) {
                    grid.z[idx+j] = static_cast<double>(tmp_float[j]);
                }
            } else {
                std::vector<double> tmp_double(n_pts);
                ifs.read((char*) &tmp_double[0], sizeof(double)*n_pts);
                for (unsigned int j=0; j < n_pts; ++j) {
                    grid.x[idx+j] = static_cast<float>(tmp_double[j]);
                }
                ifs.read((char*) &tmp_double[0], sizeof(double)*n_pts);
                for (unsigned int j=0; j < n_pts; ++j) {
                    grid.y[idx+j] = static_cast<float>(tmp_double[j]);
                }
                ifs.read((char*) &tmp_double[0], sizeof(double)*n_pts);
                for (unsigned int j=0; j < n_pts; ++j) {
                    grid.z[idx+j] = static_cast<float>(tmp_double[j]);
                }
            }
        }

        // if IBLANK is included, just skip it, don't need to read
        if (has_iblank) {
            ifs.seekg(sizeof(int32_t)*n_pts, std::ios_base::cur);
        }

        ifs.read((char*) &buf, sizeof(int32_t));
    
        idx += n_pts;
    }

    // Swap Bytes if needed
    if (big_endian) {
        for (unsigned int i=0; i < total_size; ++i) {
            swap_bytes((char*) &grid.x[i], sizeof(FP));
            swap_bytes((char*) &grid.y[i], sizeof(FP));
            swap_bytes((char*) &grid.z[i], sizeof(FP));
        }        
    }

    ifs.close();
}

template<typename StructGrid>
void write_plot3d_grid_file(std::string filename, const StructGrid& grid) {

    // Open file for writing
    std::ofstream ofs(filename);
    if (!ofs) {
        throw std::invalid_argument("Could not open file to write plot3d grid");
    }

    // Get data type
    typedef typename StructGrid::data_type FP;

    // Determine if the grid is single or multi format
    bool single_p3d = false;
    int32_t n_zones = grid.num_zones();
    if (n_zones == 1) {
        single_p3d = true;
    }

    // Setup Fortran record-length variable
    int32_t buf;

    // Write number of zones
    buf = sizeof(int32_t);
    if (!single_p3d) {
        ofs.write((char*) &buf, sizeof(int32_t));
        ofs.write((char*) &n_zones, sizeof(int32_t));
        ofs.write((char*) &buf, sizeof(int32_t));
    }

    // Write out the size of each zone
    buf = sizeof(int32_t)*n_zones*3;
    ofs.write((char*) &buf, sizeof(int32_t));
    for (unsigned int i=0; i < n_zones; ++i) {
        int32_t j_size = grid.grid_size[i][0];
        int32_t k_size = grid.grid_size[i][1];
        int32_t l_size = grid.grid_size[i][2];

        ofs.write((char*) &j_size, sizeof(int32_t));
        ofs.write((char*) &k_size, sizeof(int32_t));
        ofs.write((char*) &l_size, sizeof(int32_t));
    }
    ofs.write((char*) &buf, sizeof(int32_t));

    // Write out the x,y,z positions (no iblank)
    unsigned int idx = 0;
    for (unsigned int i=0; i < n_zones; ++i) {
        unsigned int n_pts = grid.zone_size(i);
        buf = sizeof(FP)*n_pts*3;
        ofs.write((char*) &buf, sizeof(int32_t));

        ofs.write((char*) &grid.x[idx], sizeof(FP)*n_pts);
        ofs.write((char*) &grid.y[idx], sizeof(FP)*n_pts);
        ofs.write((char*) &grid.z[idx], sizeof(FP)*n_pts);

        ofs.write((char*) &buf, sizeof(int32_t));

        idx += n_pts;
    }

    ofs.close();
}

// Explicit instantiations for concrete data types.
template void read_plot3d_grid_file(std::string, StructuredGrid<float>&);
template void read_plot3d_grid_file(std::string, StructuredGrid<double>&);
template void read_plot3d_grid_file(std::string, RefStructuredGrid<float>&);
template void read_plot3d_grid_file(std::string, RefStructuredGrid<double>&);
template void write_plot3d_grid_file(std::string, const StructuredGrid<float>&);
template void write_plot3d_grid_file(std::string, const StructuredGrid<double>&);
template void write_plot3d_grid_file(std::string, const RefStructuredGrid<float>&);
template void write_plot3d_grid_file(std::string, const RefStructuredGrid<double>&);

}  // namespace upsp
