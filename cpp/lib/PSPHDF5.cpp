/* @file
 *  @brief  HDF5 File Reader/Writer Add-ons
 *  @date   February 16, 2018
 *  @author jmpowel2
 */

#include "PSPHDF5.h"

namespace upsp {

/*****************************************************************************/
bool hdf5_is_structured(std::string filename) {

    // get file version
    unsigned int file_ver = hdf5_get_version(filename);

    // Initialize structured flag
    uint16_t structured = 2;    

    try {
        H5::H5File file(filename, H5F_ACC_RDONLY);
        H5::Group group(file.openGroup("/"));

        std::string dset_name = "structured";
        if (file_ver == 0) {
            dset_name = "Structured";
        }

        H5::Attribute attr(group.openAttribute(dset_name));
        attr.read(H5::PredType::NATIVE_UINT16, &structured);

        file.close();
        group.close();
        attr.close();

    } // end try block

    catch (H5::FileIException error) {
        std::cout << "Cannot open hdf5 file: " << error.getDetailMsg() << std::endl;
    }
    catch (H5::GroupIException error) {
        std::cout << "Cannot parse hdf5 file: " << error.getDetailMsg() << std::endl;
    }
    catch (H5::AttributeIException error) {
        std::cout << "HDF5 file missing Structured attribute: " << 
                error.getDetailMsg() << std::endl;
    }

    if (structured == 1) {
        return true;
    } else if (structured == 0) {
        return false;
    } else {
        throw(std::invalid_argument("Unknown grid format"));
    }
}

/*****************************************************************************/
bool hdf5_is_valid(std::string filename) {

    H5::Exception::dontPrint();
    try {
    
        // Check for critical information
        unsigned int file_ver = hdf5_get_version(filename);
        bool structured = hdf5_is_structured(filename);
        hdf5_is_nodal(filename);
        hdf5_num_frames(filename);
        hdf5_num_faces(filename);
        hdf5_is_transposed(filename);

        // Try to open file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // Load the number of nodes
        unsigned int n_nodes = hdf5_num_nodes(filename);

        // Check that the data really is structured/unstructured
        if (structured) {
            H5::DataSet dataset_g;
            if (file_ver == 0) {
                dataset_g = file.openDataSet("Grid_Sizes");
            } else {
                dataset_g = file.openDataSet("/Grid/grid_sizes");
            }
            dataset_g.close();
        } else {
            H5::DataSet tris  = file.openDataSet("/Grid/triangles");
            H5::DataSet comps = file.openDataSet("/Grid/components");
            tris.close();
            comps.close();
        }

        // Open Node coordinate definitions 
        H5::DataSet dataset_x;
        H5::DataSet dataset_y;
        H5::DataSet dataset_z;
        if (file_ver == 0) {
            dataset_x = file.openDataSet("X");
            dataset_y = file.openDataSet("Y");
            dataset_z = file.openDataSet("Z");
        } else {
            dataset_x = file.openDataSet("/Grid/x");
            dataset_y = file.openDataSet("/Grid/y");
            dataset_z = file.openDataSet("/Grid/z");
        }

        // Ensure X has all grid points
        H5::DataSpace xspace = dataset_x.getSpace();
        hsize_t n_dims = xspace.getSimpleExtentNdims();
        if (n_dims != 1) {
            return false;
        }
        hsize_t x_dims[n_dims];
        xspace.getSimpleExtentDims(x_dims, nullptr);
        if (x_dims[0] != n_nodes) {
            return false;
        }
        xspace.close();
        dataset_x.close();

        // Ensure Y has all grid points
        H5::DataSpace yspace = dataset_y.getSpace();
        n_dims = yspace.getSimpleExtentNdims();
        if (n_dims != 1) {
            return false;
        }
        hsize_t y_dims[n_dims];
        yspace.getSimpleExtentDims(y_dims, nullptr);
        if (y_dims[0] != n_nodes) {
            return false;
        }
        yspace.close();
        dataset_y.close();

        // Ensure Z has all grid points
        H5::DataSpace zspace = dataset_z.getSpace();
        n_dims = zspace.getSimpleExtentNdims();
        if (n_dims != 1) {
            return false;
        }
        hsize_t z_dims[n_dims];
        zspace.getSimpleExtentDims(z_dims, nullptr);
        if (z_dims[0] != n_nodes) {
            return false;
        }
        zspace.close();
        dataset_z.close();

        file.close();
    } catch (...) {
        return false;
    }

    return true;
}

/*****************************************************************************/
bool hdf5_is_nodal(std::string filename) {

    // version 0 of PSPHDF5 did not have this attribute, always nodal
    if (!hdf5_has_attribute(filename, "nodal")) {
        return true;
    }

    uint16_t nodal = 2;    
    try {
        H5::H5File file(filename, H5F_ACC_RDONLY);
        H5::Group group(file.openGroup("/"));
        H5::Attribute attr(group.openAttribute("nodal"));
        attr.read(H5::PredType::NATIVE_UINT16, &nodal);

        file.close();
        group.close();
        attr.close();

    } // end try block

    catch (H5::FileIException error) {
        std::cout << "Cannot open hdf5 file: " << error.getDetailMsg() << std::endl;
    }
    catch (H5::GroupIException error) {
        std::cout << "Cannot parse hdf5 file: " << error.getDetailMsg() << std::endl;
    }
    catch (H5::AttributeIException error) {
        std::cout << "Cannot parse hdf5 file: " << error.getDetailMsg() << std::endl;
    }
    
    if (nodal == 1) {
        return true;
    } else if (nodal == 0) {
        return false;
    } else {
        throw(std::invalid_argument("Unknown nodal setting"));
    }

}

/*****************************************************************************/
unsigned int hdf5_get_version(std::string filename) {

    // version 0 if no version number attribute
    if (!hdf5_has_attribute(filename, "psph5_version")) {
        return 0;
    }

    unsigned int ver = 0;
    try {
        H5::H5File file(filename, H5F_ACC_RDONLY);
        H5::Group group(file.openGroup("/"));
        H5::Attribute attr(group.openAttribute("psph5_version"));
        attr.read(H5::PredType::NATIVE_UINT, &ver);

        file.close();
        group.close();
        attr.close();
    } // end try block
    catch (H5::FileIException error) {
        std::cout << "Cannot open hdf5 file: " << error.getDetailMsg() << std::endl;
    }
    catch (H5::GroupIException error) {
        std::cout << "Cannot parse hdf5 file: " << error.getDetailMsg() << std::endl;
    }
    catch (H5::AttributeIException error) {
        std::cout << "Cannot parse hdf5 file: " << error.getDetailMsg() << std::endl;
    }

    return ver;
}

/*****************************************************************************/
std::string hdf5_get_code_version(std::string filename) {

    unsigned int h5ver = hdf5_get_version(filename);

    std::string code_version = "";

    try {
        H5::H5File file(filename, H5F_ACC_RDONLY);
        H5::Group group(file.openGroup("/"));
    
        std::string attr_name = "code_version";
        if (h5ver == 0) {
            attr_name = "Code Version";
        } 

        H5::StrType strdatatype(H5::PredType::C_S1, H5_STRING_LEN);
        H5::Attribute attr(group.openAttribute(attr_name));
        
        code_version.reserve(H5_STRING_LEN); // just in case it isn't allocated
        attr.read(strdatatype, code_version);

        file.close();
        group.close();
        attr.close();

    } // end try block
    catch (H5::FileIException error) {
        std::cout << "Cannot open hdf5 file: " << error.getDetailMsg() << std::endl;
    }
    catch (H5::GroupIException error) {
        std::cout << "Cannot parse hdf5 file: " << error.getDetailMsg() << std::endl;
    }
    catch (H5::AttributeIException error) {
        std::cout << "Cannot parse hdf5 file: " << error.getDetailMsg() << std::endl;
    }

    return code_version;
}

/*****************************************************************************/
unsigned int hdf5_num_nodes(std::string filename) {

    unsigned int num_nodes = 0;
    H5::Exception::dontPrint();
    try {
        // Get the file version
        unsigned int file_ver = hdf5_get_version(filename);

        // Try to open file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // Handle single grid
        std::string dset_name = "/Grid/x";
        if (file_ver == 0) {
            dset_name = "X";
        }
        H5::DataSet gset(file.openDataSet(dset_name));
        H5::DataSpace gspace = gset.getSpace();

        assert(gspace.getSimpleExtentNdims() == 1);

        hsize_t nodes;
        gspace.getSimpleExtentDims(&nodes, nullptr);
        num_nodes = nodes;

        gset.close();
        gspace.close();

    } // end try block
    catch (H5::FileIException error) {
        return 0;
    }
    catch (H5::DataSetIException error) {
        return 0;
    }
    catch (H5::DataSpaceIException error) {
        return 0;
    }

    return num_nodes;
}

/*****************************************************************************/
unsigned int hdf5_num_faces(std::string filename) {

    unsigned int num_faces = 0;
    H5::Exception::dontPrint();
    try {
        // Try to open file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // Get the file version
        unsigned int file_ver = hdf5_get_version(filename);

        // Determine if the file is structured
        bool structured = hdf5_is_structured(filename);

        // Get the dimensions of each zone
        if (structured) {
            std::string dset_name = "/Grid/grid_sizes";
            if (file_ver == 0) {
                dset_name = "Grid_Sizes";
            }
            H5::DataSet gset(file.openDataSet(dset_name));
            H5::DataSpace gspace = gset.getSpace();
            int n_dims = gspace.getSimpleExtentNdims();
            assert(n_dims == 2);
            hsize_t dims[n_dims];
            gspace.getSimpleExtentDims(dims, nullptr);
            assert(dims[1] == 3);

            hsize_t zones = dims[0];

            // use the dimensions to compute the number of 
            // faces in each zone 
            int gsize[zones*dims[1]];
            gset.read(gsize, H5::PredType::NATIVE_INT);
            
            for (unsigned int i=0; i < zones; ++i) {
                num_faces += (gsize[3*i]-1)*(gsize[3*i+1]-1); // assume l=1
            }

            gset.close();
            gspace.close();
        } else {
            std::string dset_name = "/Grid/triangles";
            H5::DataSet gset(file.openDataSet(dset_name));
            H5::DataSpace gspace = gset.getSpace();
            int n_dims = gspace.getSimpleExtentNdims();
            assert(n_dims == 2);
            hsize_t dims[n_dims];
            gspace.getSimpleExtentDims(dims, nullptr);
            assert(dims[1] == 3);

            num_faces = dims[0];

            gset.close();
            gspace.close();
        }

    } // end try block
    catch (H5::FileIException error) {
        return 0;
    }
    catch (H5::DataSetIException error) {
        return 0;
    }
    catch (H5::DataSpaceIException error) {
        return 0;
    }

    return num_faces;

}

/*****************************************************************************/
unsigned int hdf5_num_pts(std::string filename) {
    if (hdf5_is_nodal(filename)) {
        return hdf5_num_nodes(filename);
    } else {
        return hdf5_num_faces(filename);
    }
}

/*****************************************************************************/
std::string hdf5_grid_units(std::string filename) {

    std::string units = "";

    unsigned int file_ver = hdf5_get_version(filename);

    if (file_ver == 0) {
        return "";
    } 
     
    try {
        // Try to open file
        H5::H5File file(filename, H5F_ACC_RDONLY);
        
        // Try to get the grid units attribute
        H5::StrType strdatatype(H5::PredType::C_S1, H5_STRING_LEN);
        H5::Attribute attr(file.openAttribute("Grid/units"));
        units.reserve(H5_STRING_LEN); // just in case it isn't allocated
        attr.read(strdatatype, units);

    } catch (...) {
        units = "";
    }

    return units;
}

/*****************************************************************************/
unsigned int hdf5_num_frames(std::string filename) {

    // Retrieve a primary data set
    std::vector<std::string> prim_sets;
    try { 
        hdf5_primary_datasets(filename, prim_sets);
    } catch (...) {
        //std::cout << "failed to get primary datasets" << std::endl;
        return 0;
    }
    if (prim_sets.size() == 0) {
        //std::cout << "no primary datasets" << std::endl;
        return 0;
    }

    unsigned int num_frames = 0;
    H5::Exception::dontPrint();
    try {
        // Try to open file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // Check if solutions exist
        H5::DataSet dataset(file.openDataSet(prim_sets[0]));
        H5::DataSpace dspace = dataset.getSpace();
        int n_dims = dspace.getSimpleExtentNdims();
        if (n_dims == 1) {
            num_frames = 1;
        } else if (n_dims == 2) {
            // Determine which dimension the frames are in
            unsigned int num_pts = hdf5_num_pts(filename);

            hsize_t dims[n_dims];
            dspace.getSimpleExtentDims(dims, nullptr);
            if (dims[0] == num_pts) {
                num_frames = dims[1];
            } else {
                num_frames = dims[0];
            }
        } else {
            return 0;
        }
    
        dataset.close();
        dspace.close();
        file.close();

    } // end try block
    catch (H5::FileIException error) {
        return 0;
    }
    catch (H5::DataSetIException error) {
        return 0;
    }
    catch (H5::DataSpaceIException error) {
        return 0;
    }

    return num_frames;
}

/*****************************************************************************/
bool hdf5_has_additional_datasets(std::string filename) {
    std::vector<std::string> datasets;
    hdf5_additional_datasets(filename, datasets);
    if (datasets.size() > 0) {
        return true;
    } else {
        return false;
    }
}

/*****************************************************************************/
void hdf5_primary_datasets(std::string filename, 
        std::vector<std::string>& datasets) {

    datasets.clear();
    try {
        // open file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // open group
        H5::Group root_group = file.openGroup("/");

        // create function to get the names of each object
        auto getobjnames = [](hid_t loc_id, const char* member_name, 
                const H5L_info_t* linfo, void* opdata) {
            auto names = reinterpret_cast< std::vector<std::string>* >(opdata);
            names->push_back( member_name );
            return 0;
        };

        // get object names
        herr_t idx = H5Literate(root_group.getId(), H5_INDEX_NAME, H5_ITER_INC, nullptr,
                getobjnames, &datasets);

        if (idx != 0) {
            throw(std::invalid_argument("Could not check for primary datasets: "));
        }

        // remove datasets that are standard or multidimensional
        auto it = datasets.begin();
        while (it != datasets.end()) {
            if (((*it) == "Grid_Sizes") || ((*it) == "Grid") || ((*it) == "Condition")) {
                it = datasets.erase(it);
            } else {
                H5::DataSet dset = file.openDataSet(*it);
                H5::DataSpace dspace = dset.getSpace();
                int n_dims = dspace.getSimpleExtentNdims();
                if (n_dims == 1) {
                    it = datasets.erase(it);
                } else {
                    ++it;
                }
            }
        }

        root_group.close();
        file.close();

    } catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not check for primary datasets: " 
                + error.getDetailMsg()));
    } catch (H5::GroupIException error) {
        throw(std::invalid_argument("Could not check for primary datasets: "
                + error.getDetailMsg()));
    } catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not check for primary datasets: " 
                + error.getDetailMsg()));
    } catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not check for primary datasets: " 
                + error.getDetailMsg()));
    }
}

/*****************************************************************************/
void hdf5_additional_datasets(std::string filename, 
        std::vector<std::string>& datasets) {

    datasets.clear();

    try {
        // open file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // open group
        H5::Group root_group = file.openGroup("/");

        // create function to get the names of each object
        auto getobjnames = [](hid_t loc_id, const char* member_name, 
                const H5L_info_t* linfo, void* opdata) {
            auto names = reinterpret_cast< std::vector<std::string>* >(opdata);
            names->push_back( member_name );
            return 0;
        };

        // get object names
        herr_t idx = H5Literate(root_group.getId(), H5_INDEX_NAME, H5_ITER_INC, nullptr,
                getobjnames, &datasets);

        if (idx != 0) {
            throw(std::invalid_argument("Could not check for additional datasets: "));
        }

        // remove datasets that are standard or multidimensional
        auto it = datasets.begin();
        while (it != datasets.end()) {
            if ( ((*it) == "Grid_Sizes") || ((*it) == "X") || ((*it) == "Y") || 
                    ((*it) == "Z") || ((*it) == "frames") || ((*it) == "Grid") ||
                    ((*it) == "Condition") ) {
                it = datasets.erase(it);
            } else {
                H5::DataSet dset = file.openDataSet(*it);
                H5::DataSpace dspace = dset.getSpace();
                int n_dims = dspace.getSimpleExtentNdims();
                if (n_dims > 1) {
                    it = datasets.erase(it);
                } else {
                    ++it;
                }
            }
        }

        root_group.close();
        file.close();

    } catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not check for additional datasets: " 
                + error.getDetailMsg()));
    } catch (H5::GroupIException error) {
        throw(std::invalid_argument("Could not check for additional datasets: " 
                + error.getDetailMsg()));
    } catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not check for additional datasets: " 
                + error.getDetailMsg()));
    } catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not check for additional datasets: " 
                + error.getDetailMsg()));
    }
}

/*****************************************************************************/
void hdf5_attributes(std::string filename, std::vector<std::string>& attr_name) { 

    attr_name.clear();

    try {
        // open file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // open group
        H5::Group root_group = file.openGroup("/");

        // attributes are indexed from 0 to N-1 
        // where N is the number of attributes
        unsigned int N = root_group.getNumAttrs();

        // get the names and values of each attribute
        for (unsigned int i=0; i < N; ++i) {
            H5::Attribute attr = root_group.openAttribute(i);

            attr_name.push_back(attr.getName());

            attr.close();
        }

        root_group.close();
        file.close();

    } catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not check for additional datasets: " 
                + error.getDetailMsg()));
    } catch (H5::GroupIException error) {
        throw(std::invalid_argument("Could not check for additional datasets: " 
                + error.getDetailMsg()));
    } catch (H5::AttributeIException error) {
        throw(std::invalid_argument("Could not check for additional datasets: " 
                + error.getDetailMsg()));
    }
}

/*****************************************************************************/
bool hdf5_has_attribute(std::string filename, std::string attr_name) {
    std::vector<std::string> names;
    hdf5_attributes(filename, names);
    for (unsigned int i = 0; i < names.size(); ++i) {
        if (names[i] == attr_name) {
            return true;
        }
    }
    return false;
}

/*****************************************************************************/
std::string hdf5_read_attribute(std::string filename, std::string attr_name) {

    std::string out;

    try {
        H5::H5File file(filename, H5F_ACC_RDONLY);
        H5::Group root_group = file.openGroup("/");
        H5::Attribute attr = root_group.openAttribute(attr_name);
        H5::DataType dt = attr.getDataType();

        out.reserve(H5_STRING_LEN); // just in case it isn't allocated
        attr.read(dt, out);

        dt.close();
        attr.close();
        root_group.close();
        file.close();
    } catch (H5::FileIException error) {
        throw(std::invalid_argument("Unable to read attribute " + attr_name));
    } catch (H5::GroupIException error) {
        throw(std::invalid_argument("Unable to read attribute " + attr_name));
    } catch (H5::AttributeIException error) {
        throw(std::invalid_argument("Unable to read attribute " + attr_name));
    } catch (H5::DataTypeIException error) {
        throw(std::invalid_argument("Unable to read attribute " + attr_name));
    }

    return out;
}

/*****************************************************************************/
unsigned int hdf5_pts_per_chunk(std::string filename) {
    unsigned int num_pts = 0;
    try {
        // Try to open file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // Get the dataset
        std::vector<std::string> prim_dsets;
        hdf5_primary_datasets(filename, prim_dsets);

        if (prim_dsets.size() == 0) {
            return 0;
        }

        H5::DataSet dataset(file.openDataSet(prim_dsets[0]));

        // Get the size of the dataset and node dimension
        H5::DataSpace dspace = dataset.getSpace();
        int n_dims = dspace.getSimpleExtentNdims();
        unsigned int node_dim = 0;
        if (n_dims == 2) {
            // Determine which dimension the frames are in
            unsigned int num_nodes = hdf5_num_pts(filename);
    
            hsize_t dims[n_dims];
            dspace.getSimpleExtentDims(dims, nullptr);
            if (dims[0] == num_nodes) {
                node_dim = 0;
            } else {
                node_dim = 1;
            }
        } else if (n_dims > 2) {
            return 0; // invalid dataset
        }

        // Get the creation property list
        H5::DSetCreatPropList plist = dataset.getCreatePlist();

        // Get the size of the chunks 
        hsize_t dims[n_dims];
        plist.getChunk(n_dims, dims);

        // Get the number of nodes per chunk
        num_pts = dims[node_dim];
        
        plist.close();
        dataset.close();
        dspace.close();
        file.close();

    } // end try block
    catch (H5::FileIException error) {
        return 0;
    }
    catch (H5::DataSetIException error) {
        return 0;
    }
    catch (H5::DataSpaceIException error) {
        return 0;
    }
    catch (H5::PropListIException error) {
        return 0;
    }

    return num_pts;

}

/*****************************************************************************/
unsigned int hdf5_frames_per_chunk(std::string filename) {
    unsigned int num_frames = 0;
    try {
        // Try to open file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // Get a primary dataset
        std::vector<std::string> prim_dsets;
        hdf5_primary_datasets(filename, prim_dsets);

        if (prim_dsets.size() == 0) {
            return 1;
        }

        // Get the dataset
        H5::DataSet dataset(file.openDataSet(prim_dsets[0]));

        // Get the size of the dataset and node dimension
        H5::DataSpace dspace = dataset.getSpace();
        int n_dims = dspace.getSimpleExtentNdims();
        unsigned int frame_dim = 0;
        if (n_dims == 2) {
            // Determine which dimension the frames are in
            unsigned int num_nodes = hdf5_num_pts(filename);
    
            hsize_t dims[n_dims];
            dspace.getSimpleExtentDims(dims, nullptr);
            if (dims[0] == num_nodes) {
                frame_dim = 1;
            } else {
                frame_dim = 0;
            }
        } else if (n_dims > 2) {
            return 0; // invalid dataset
        }

        // Get the creation property list
        H5::DSetCreatPropList plist = dataset.getCreatePlist();

        // Get the size of the chunks 
        hsize_t dims[n_dims];
        plist.getChunk(n_dims, dims);

        // Get the number of nodes per chunk
        num_frames = dims[frame_dim];
        
        plist.close();
        dataset.close();
        dspace.close();
        file.close();

    } // end try block
    catch (H5::FileIException error) {
        return 0;
    }
    catch (H5::DataSetIException error) {
        return 0;
    }
    catch (H5::DataSpaceIException error) {
        return 0;
    }
    catch (H5::PropListIException error) {
        return 0;
    }

    return num_frames;

}

/*****************************************************************************/
bool hdf5_is_transposed(std::string filename) {

    try {
        // Try to open file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // version 0 of PSPHDF5 did not have this attribute,
        // only works if number of points != number of frames
        if (!hdf5_has_attribute(filename, "transpose")) {

            // Get a primary dataset
            std::vector<std::string> prim_sets;
            hdf5_primary_datasets(filename, prim_sets);
            if (prim_sets.size() == 0) {
                file.close();
                return true; // transpose state is irrelevant
            }

            H5::DataSet dataset(file.openDataSet(prim_sets[0]));

            // Get the size of the dataset and node dimension
            H5::DataSpace dspace = dataset.getSpace();
            int n_dims = dspace.getSimpleExtentNdims();
            //unsigned int frame_dim = 0;

            assert(n_dims == 2);

            // Determine which dimension the frames are in
            unsigned int total_nodes = hdf5_num_pts(filename);

            hsize_t dims[n_dims];
            dspace.getSimpleExtentDims(dims, nullptr);

            file.close();
            dspace.close();
            dataset.close();

            if (dims[0] == total_nodes) {
                return true; //  nodes in 1st dimension == transposed matrix
            } else if (dims[0] != dims[1]) {
                return false;  //  nodes in 2nd dimension == nominal matrix
            }

        } else {

            uint16_t transpose = 2;    
            H5::Group group(file.openGroup("/"));
            H5::Attribute attr(group.openAttribute("transpose"));
            attr.read(H5::PredType::NATIVE_UINT16, &transpose);

            file.close();
            group.close();
            attr.close();

            if (transpose == 1) {
                return true;
            } else if (transpose == 0) {
                return false;
            }
        }
    
    } 
    catch (H5::FileIException error) {
        throw(std::invalid_argument("Unable to determine if " + 
                filename + " is transposed"));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Unable to determine if " + 
                filename + " is transposed"));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Unable to determine if " + 
                filename + " is transposed"));
    }
    catch (H5::PropListIException error) {
        throw(std::invalid_argument("Unable to determine if " + 
                filename + " is transposed"));
    }

    // Should not make it here, added to prevent warning
    throw(std::invalid_argument("Unable to determine if " +
            filename + " is transposed"));
        
    return false;
}

/*****************************************************************************/
std::string hdf5_read_dataset_units(std::string filename, std::string dataset) {

    unsigned int ver = hdf5_get_version(filename);

    // Did not use units as attributes in version 0
    if (ver == 0) {
        if (dataset == "frames") {
            return upsp::hdf5_read_attribute(filename, "Frame Units");
        } else {
            return "";
        }
    }

    std::string out;
    try {
        // Try to open file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        H5::DataSet dset = file.openDataSet(dataset);
        H5::Attribute attr = dset.openAttribute("units");
        H5::DataType dt = attr.getDataType();

        out.reserve(H5_STRING_LEN); // just in case it isn't allocated
        attr.read(dt, out);

        dt.close();
        attr.close();
        dset.close();
        file.close();

    } catch (...) {
        // assume that there are no units
        return "";
        //throw(std::invalid_argument("Unable to read units for " + dataset)); 
    }

    return out;
}

/*****************************************************************************/
void hdf5_efficient_read(std::string filename, unsigned int frame_start,
        unsigned int frame_delta, unsigned int frame_end,
        float max_memory, 
        std::vector<std::pair<unsigned int,unsigned int>>& partitions) {

    partitions.clear();

    // check that at least 2 frames are desired
    if ( (frame_start + frame_delta) > frame_end) {
        partitions.push_back({frame_start,frame_start+1});
        return;
    }

    // determine the size of a single frame in MB
    float frame_mem_size = ( (float) hdf5_num_pts(filename) * sizeof(float)) / 1e6;

    // setup the partitions
    partitions.push_back({frame_start,frame_start+1});
    unsigned int l_idx = 0; // reference index
    //unsigned int l_frame = frame_start; // last checked frame
    for (unsigned int i=frame_start+frame_delta; i <= frame_end; i += frame_delta) {
        // compute the size of reading in from the last index through the current
        float curr_size = frame_mem_size * 
                                 (i - partitions[l_idx].first + 1);

        // if the new size would exceed the allowable memory, 
        // create a new partition
        if (curr_size > max_memory) {
            partitions.push_back({i,i+1});
            ++l_idx;
        } else {
            partitions[l_idx].second = i+1;
        }
    }

}


} /* end namespace upsp */
