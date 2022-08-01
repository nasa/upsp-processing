/** @file
 *  @brief  HDF5 File Reader/Writer
 *  @date   Nov 15, 2018
 *  @author jmpowel2
 */

namespace upsp {

/********************************************************************
 * PSPWriter Class
********************************************************************/

/*****************************************************************************/
template<typename Model>
PSPWriter<Model>::PSPWriter(std::string filename, const Model* model, unsigned int num_frames, 
        float chunk_size/*=1.0*/, bool transposed/* = false*/, 
        unsigned int trans_nodes/* = 250*/, bool nodal/* = true */) : 
        filename_(filename), 
        transposed_(transposed), model_(model), 
        chunk_size_(chunk_size), file_(nullptr), num_frames_(num_frames),
        trans_nodes_(trans_nodes), nodal_(nodal) {

    // check for valid mytpe
    static_assert(psph5_valid_type<mtype>() , "PSPHDF5 requires float or double grid");

    // set the number of data points per frame
    if (nodal) {
        data_pts_ = model_->size();
    } else {
        data_pts_ = model_->number_of_faces();
    }
    
    // check that if transposed, included valid number of frames
    if (transposed) {
        if (num_frames == 0) {
            throw(std::invalid_argument("Must specify number of frames for a transposed PSPHDF5 file"));
        }
        if (trans_nodes_ > data_pts_) {
            trans_nodes_ = data_pts_;
        }
    }

    // open new file
    file_ = new H5::H5File(filename_, H5F_ACC_TRUNC);

    // set properties
    set_chunk_length();

    // write the version number as attribute
    hsize_t attr_dims[] = { 1 };
    H5::DataSpace attr_dataspace(1, attr_dims);
    H5::Group root_group = file_->openGroup("/");
    H5::Attribute attr = root_group.createAttribute("psph5_version", 
            H5::PredType::NATIVE_UINT, attr_dataspace);
    attr.write(H5::PredType::NATIVE_UINT, &PSPH5_VERSION);

    // write nodal as attribute
    uint16_t attr_data[] = { 1 };
    if (!nodal) {
        attr_data[0] = 0;
    }
    H5::Attribute nodal_attr = root_group.createAttribute("nodal",
            H5::PredType::NATIVE_UINT16, attr_dataspace);
    nodal_attr.write(H5::PredType::NATIVE_UINT16, attr_data);

    // write transpose as attribute
    attr_data[0] = 1;
    if (!transposed_) {
        attr_data[0] = 0;
    }
    H5::Attribute trans_attr = root_group.createAttribute("transpose",
            H5::PredType::NATIVE_UINT16, attr_dataspace);
    trans_attr.write(H5::PredType::NATIVE_UINT16, attr_data);

    // create group to store test conditions/camera settings
    file_->createGroup("/Condition");
}

/*****************************************************************************/
template<typename Model>
PSPWriter<Model>::~PSPWriter() {
    for (unsigned int i=0; i < sols_.size(); ++i) {
        if (open_dataset_[i]) {
            dataset_[i].close();
        }
    }
    delete file_;   
}

/*****************************************************************************/
template<typename Model>
void PSPWriter<Model>::write_grid(std::string units/* = "" */) {

    // determine if the grid is structured or unstructured
    if constexpr(is_structured<Model>()) {
        write_structured_grid(units);
    } else {
        write_unstructured_grid(units);
    }
}

/*****************************************************************************/
template<typename Model>
void PSPWriter<Model>::write_structured_grid(std::string units) {

    if constexpr(!is_structured<Model>()) {
        return;
    }

    try {

    // Write descriptive attributes
    uint16_t attr_data[] = { 1 }; // 1 for structured
    hsize_t attr_dims[] = { 1 };
    H5::DataSpace attr_dataspace(1, attr_dims);
    H5::Group root_group = file_->openGroup("/");
    H5::Attribute attr = root_group.createAttribute("structured",
            H5::PredType::NATIVE_UINT16, attr_dataspace);
    attr.write(H5::PredType::NATIVE_UINT16, attr_data);

    attr.close();

    // define hdf5 data type for grid point values
    H5::DataType ptyp;
    get_data_type(ptyp);

    // define property list
    mtype fillValue = 0.0;
    H5::DSetCreatPropList plist;
    plist.setFillValue(ptyp, &fillValue);

    // get node locations
    const std::vector<mtype> x_vals = model_->get_x();
    const std::vector<mtype> y_vals = model_->get_y();
    const std::vector<mtype> z_vals = model_->get_z();

    // Create group for Grid
    H5::Group g_group = file_->createGroup("/Grid");

    // write out the x,y,z point locations
    hsize_t fdim[] = {(hsize_t) model_->size()};
    H5::DataSpace fspace(1, fdim);

    H5::DataSet dataset_x = file_->createDataSet("/Grid/x", ptyp, fspace, plist);
    dataset_x.write(&x_vals[0], ptyp);

    H5::DataSet dataset_y = file_->createDataSet("/Grid/y", ptyp, fspace, plist);
    dataset_y.write(&y_vals[0], ptyp);

    H5::DataSet dataset_z = file_->createDataSet("/Grid/z", ptyp, fspace, plist);
    dataset_z.write(&z_vals[0], ptyp);

    // add units even if empty
    H5::StrType strdatatype(H5::PredType::C_S1, H5_STRING_LEN);
    H5std_string strwritebuf(units);
    strwritebuf.resize(H5_STRING_LEN, '\0'); // pad with \0
    attr = g_group.createAttribute("units",
        strdatatype, attr_dataspace);
    attr.write(strdatatype, strwritebuf);
    attr.close();

    // write out the grid structure
    hsize_t gdim[] = { (hsize_t) model_->num_zones(), 3};
    H5::DataSpace gspace(2, gdim);

    H5::DataSet dataset_grid = file_->createDataSet("/Grid/grid_sizes", 
            H5::PredType::NATIVE_INT, gspace);

    int gsz[model_->num_zones()][3];
    for (unsigned int i=0; i < model_->num_zones(); ++i) {
        gsz[i][0] = model_->zone_size(i,0); 
        gsz[i][1] = model_->zone_size(i,1);
        gsz[i][2] = model_->zone_size(i,2);
    }

    dataset_grid.write(gsz, H5::PredType::NATIVE_INT);

    attr_dataspace.close();
    
    } // end try block 
    catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not write grid to psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::AttributeIException error) {
        throw(std::invalid_argument("Could not write grid to psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not write grid to psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not write grid to psp hdf5 file: " + error.getDetailMsg()));
    }

}

/*****************************************************************************/
template<typename Model>
void PSPWriter<Model>::write_unstructured_grid(std::string units) {

    if constexpr(is_structured<Model>()) {
        return;
    }

    try {

    // Write descriptive attributes
    uint16_t attr_data[] = { 0 }; // 0 for unstructured
    hsize_t attr_dims[] = { 1 };
    H5::DataSpace attr_dataspace(1, attr_dims);
    H5::Group root_group = file_->openGroup("/");
    H5::Attribute attr = root_group.createAttribute("structured",
            H5::PredType::NATIVE_UINT16, attr_dataspace);
    attr.write(H5::PredType::NATIVE_UINT16, attr_data);

    attr.close();

    // define hdf5 data type for grid point values
    H5::DataType ptyp;
    get_data_type(ptyp);

    // define property list
    mtype fillValue = 0.0;
    H5::DSetCreatPropList plist;
    plist.setFillValue(ptyp, &fillValue);

    // get node locations
    const std::vector<mtype> x_vals = model_->get_x();
    const std::vector<mtype> y_vals = model_->get_y();
    const std::vector<mtype> z_vals = model_->get_z();

    // Create group for Grid
    H5::Group g_group = file_->createGroup("/Grid");

    // write out the x,y,z point locations
    hsize_t fdim[] = {(hsize_t) model_->size()};
    H5::DataSpace fspace(1, fdim);

    H5::DataSet dataset_x = file_->createDataSet("/Grid/x", ptyp, fspace, plist);
    dataset_x.write(&x_vals[0], ptyp);

    H5::DataSet dataset_y = file_->createDataSet("/Grid/y", ptyp, fspace, plist);
    dataset_y.write(&y_vals[0], ptyp);

    H5::DataSet dataset_z = file_->createDataSet("/Grid/z", ptyp, fspace, plist);
    dataset_z.write(&z_vals[0], ptyp);

    dataset_x.close();
    dataset_y.close();
    dataset_z.close();
    fspace.close();

    // add units even if empty
    H5::StrType strdatatype(H5::PredType::C_S1, H5_STRING_LEN);
    H5std_string strwritebuf(units);
    strwritebuf.resize(H5_STRING_LEN, '\0'); // pad with \0
    attr = g_group.createAttribute("units",
        strdatatype, attr_dataspace);
    attr.write(strdatatype, strwritebuf);
    attr.close();

    // write out the triangles
    hsize_t face_dim[] = { (hsize_t) model_->number_of_faces(), 3 };
    H5::DataSpace tri_space(2, face_dim);

    H5::DataSet dataset_tris = file_->createDataSet("/Grid/triangles",
            H5::PredType::NATIVE_UINT, tri_space);

    std::vector<unsigned int> nodes(model_->number_of_faces()*3, 0);
    std::vector<int> comps(model_->number_of_faces(), 0);
    unsigned int count = 0;
    for (auto it=model_->cface_begin(); it != model_->cface_end(); ++it) {
        typename Model::simple_tri tri = (*it).nodes();
        nodes[3*count]   = tri[0];
        nodes[3*count+1] = tri[1];
        nodes[3*count+2] = tri[2];
        comps[count] = (*it).get_component();
        ++count;        
    }
    dataset_tris.write(&nodes[0], H5::PredType::NATIVE_UINT); 

    dataset_tris.close();
    
    // write out the components
    hsize_t comp_dim[] = { (hsize_t) model_->number_of_faces()};
    H5::DataSpace comp_space(1, comp_dim);

    H5::DataSet dset_comp = file_->createDataSet("/Grid/components",
            H5::PredType::NATIVE_INT, comp_space);

    dset_comp.write(&comps[0], H5::PredType::NATIVE_INT);
    
    dset_comp.close();

    attr_dataspace.close();
    g_group.close();

    } // end try block 
    catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not write grid to psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::AttributeIException error) {
        throw(std::invalid_argument("Could not write grid to psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not write grid to psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not write grid to psp hdf5 file: " + error.getDetailMsg()));
    }

}

/*****************************************************************************/
template<typename Model>
template<typename T>
void PSPWriter<Model>::write_solution(const std::vector<T>& sol, 
        bool all_frames/* = false*/) {
    write_solution("frames", sol, all_frames);
}

/*****************************************************************************/
template<typename Model>
template<typename T>
void PSPWriter<Model>::write_solution(std::string sol_name,
        const std::vector<T>& sol, bool all_frames/* = false */) {

    // find the solution progress, if it hasn't been created, then
    // initialize to start
    bool has_sol = false;
    unsigned int sol_id = 0;
    for (unsigned int i=0; i < sols_.size(); ++i) {
        if (sol_name == sols_[i]) {
            sol_id = i;
            has_sol = true;
            break;
        }
    }
    if (!has_sol) {
        sols_.push_back(sol_name);
        curr_pt_.push_back(0);
        curr_frame_.push_back(0);

        open_dataset_.push_back(false);
        dataset_.resize(dataset_.size() + 1);
        sol_id = dataset_.size() - 1;
    }

    if (transposed_) {
        if (all_frames) {
            transpose_write_solution2(sol_id, sol);
        } else {
            transpose_write_solution(sol_id, sol);
        }
    } else {
        nominal_write_solution(sol_id, sol);
    }

}

/*****************************************************************************/
template<typename Model>
template<typename T>
void PSPWriter<Model>::write_new_dataset(std::string name, std::vector<T>& sol,
        std::string units/*=""*/) {
    static_assert(psph5_valid_type<T>(), "Solution must be float or double");

    if (sol.size() != data_pts_) {
        throw std::invalid_argument("Must have solution at every grid point");
    }

    try {

    // define hdf5 data type for grid point values
    H5::DataType ptyp;
    if (sizeof(T) == 4) {
        ptyp = H5::PredType::NATIVE_FLOAT;
    } else if (sizeof(T) == 8) {
        ptyp = H5::PredType::NATIVE_DOUBLE;
    }

    // define property list
    T fillValue = 0.0;
    H5::DSetCreatPropList plist;
    plist.setFillValue(ptyp, &fillValue);

    // create dataset and write data
    hsize_t fdim[] = {(hsize_t) data_pts_};
    H5::DataSpace fspace(1, fdim);
    H5::DataSet dataset = file_->createDataSet(name, ptyp, fspace, plist);
    dataset.write(&sol[0], ptyp);

    dataset.close();
    fspace.close();
    
    } // end try block 
    catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not write dataset " + name + " to psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not write dataset " + name + " to psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not write dataset " + name + " to psp hdf5 file: " + error.getDetailMsg()));
    }

    if (units != "") {
        add_units(name, units);
    }

}

/*****************************************************************************/
template<typename Model>
void PSPWriter<Model>::add_units(std::string dataset, std::string units) {

    try {

        // try to load the dataset
        H5::DataSet dset(file_->openDataSet(dataset));

        // add units attribute 
        hsize_t attr_dims[] = { 1 };
        H5::DataSpace dspace(1, attr_dims);
        H5::StrType strdatatype(H5::PredType::C_S1, H5_STRING_LEN);
        H5std_string strwritebuf(units);
        strwritebuf.resize(H5_STRING_LEN, '\0'); // pad with \0
        H5::Attribute attr = dset.createAttribute("units",
                strdatatype, dspace);
        attr.write(strdatatype, strwritebuf);
        attr.close();

        dset.close();

    } // end try block
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not write units to dataset " + dataset + 
                ": " + error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not write units to dataset " + dataset + 
                ": " + error.getDetailMsg()));
    } 
    catch (H5::AttributeIException error) {
        throw(std::invalid_argument("Could not write units to dataset " + dataset +
                ": " + error.getDetailMsg()));
    }

}

/*****************************************************************************/
template<typename Model>
template<typename CamSet>
void PSPWriter<Model>::write_camera_settings(const CamSet& cs) {

    try {

    // Set up sizes
    hsize_t attr_dims[] = { 1 };
    H5::DataSpace dspace(1, attr_dims);

    /********************************************/
    /** Add Integer Datasets                    */
    /********************************************/

    // define value type
    H5::DataType ptyp = H5::PredType::NATIVE_INT;

    // define property list
    int ifillValue = 0;
    H5::DSetCreatPropList plist;
    plist.setFillValue(ptyp, &ifillValue);

    // Add frame rate and units
    H5::DataSet dset = file_->createDataSet("/Condition/frame_rate", ptyp, dspace, plist);
    dset.write(&cs.framerate, ptyp);

    H5::StrType strdatatype(H5::PredType::C_S1, H5_STRING_LEN);
    H5std_string strwritebuf("Hz");
    strwritebuf.resize(H5_STRING_LEN, '\0'); // pad with \0
    H5::Attribute attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf);
    attr.close();

    dset.close();

    /********************************************/
    /** Add Float Datasets                      */
    /********************************************/
    
    // define value type
    ptyp = H5::PredType::NATIVE_FLOAT;

    // define property list
    float ffillValue = 0.0;
    plist.setFillValue(ptyp, &ffillValue);

    // Add fstop 
    dset = file_->createDataSet("/Condition/fstop", ptyp, dspace, plist);
    dset.write(&cs.fstop, ptyp);

    H5std_string strwritebuf2("-");
    strwritebuf2.resize(H5_STRING_LEN, '\0'); // pad with \0
    attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf2);
    attr.close();

    dset.close();
    
    // Add exposure
    dset = file_->createDataSet("/Condition/exposure", ptyp, dspace, plist);
    dset.write(&cs.exposure, ptyp);

    H5std_string strwritebuf3("microseconds");
    strwritebuf3.resize(H5_STRING_LEN, '\0'); // pad with \0
    attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf3);
    attr.close();

    dset.close();

    // Add focal lengths
    hsize_t fl_dims[] = { (hsize_t) cs.focal_lengths.size() };
    H5::DataSpace flspace(1, fl_dims);

    dset = file_->createDataSet("/Condition/focal_length", ptyp, flspace, plist);
    dset.write(&cs.focal_lengths[0], ptyp);

    H5std_string strwritebuf4("mm");
    strwritebuf4.resize(H5_STRING_LEN, '\0'); // pad with \0
    attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf4);
    attr.close();

    dset.close();

    // close out
    dspace.close();
    flspace.close();

    } catch (H5::GroupIException error) {
        throw(std::invalid_argument("Could not write camera settings to psp hdf5 file: " + error.getDetailMsg()));
    } catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not write camera settings to psp hdf5 file: " + error.getDetailMsg()));
    } catch (H5::AttributeIException error) {
        throw(std::invalid_argument("Could not write camera settings to psp hdf5 file: " + error.getDetailMsg()));
    }
}

/*****************************************************************************/
template<typename Model>
template<typename Cond>
void PSPWriter<Model>::write_tunnel_conditions(const Cond& cond) {

    try {
    hsize_t attr_dims[] = { 1 };
    H5::DataSpace dspace(1, attr_dims);

    // Add the test number
    H5::StrType strdatatype(H5::PredType::C_S1, H5_STRING_LEN);
    H5std_string strwritebuf1(cond.test_id);
    strwritebuf1.resize(H5_STRING_LEN, '\0'); // pad with \0
   
    H5::DataSet dset = file_->createDataSet("/Condition/test_id", strdatatype, dspace);
    dset.write(strwritebuf1, strdatatype);
    
    dset.close();

    /********************************************/
    /*  Add Integer Datasets                    */
    /********************************************/

    // define value type
    H5::DataType ptyp = H5::PredType::NATIVE_INT;

    // define property list
    int ifillValue = 0;
    H5::DSetCreatPropList plist;
    plist.setFillValue(ptyp, &ifillValue);

    // add run number
    dset = file_->createDataSet("/Condition/run", ptyp, dspace, plist);
    dset.write(&cond.run, ptyp);

    H5std_string strwritebuf("-");
    strwritebuf.resize(H5_STRING_LEN, '\0'); // pad with \0
    H5::Attribute attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf);
    attr.close();

    dset.close();

    // add sequence number
    dset = file_->createDataSet("/Condition/sequence", ptyp, dspace, plist);
    dset.write(&cond.seq, ptyp);

    attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf);
    attr.close();

    dset.close();

    /********************************************/
    /*  Add Float Datasets                      */
    /********************************************/

    // define value type
    ptyp = H5::PredType::NATIVE_FLOAT;

    // define property list
    float ffillValue = 0.0;
    plist.setFillValue(ptyp, &ffillValue);

    // add angle of attack
    dset = file_->createDataSet("/Condition/alpha", ptyp, dspace, plist);
    dset.write(&cond.alpha, ptyp);

    H5std_string strwritebuf2("deg");
    strwritebuf2.resize(H5_STRING_LEN, '\0'); // pad with \0
    attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf2);
    attr.close();

    dset.close();

    // add sideslip angle
    dset = file_->createDataSet("/Condition/beta", ptyp, dspace, plist);
    dset.write(&cond.beta, ptyp);

    attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf2);
    attr.close();

    dset.close();

    // add model roll angle
    dset = file_->createDataSet("/Condition/phi", ptyp, dspace, plist);
    dset.write(&cond.phi, ptyp);

    attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf2);
    attr.close();

    dset.close();

    // add Mach number
    dset = file_->createDataSet("/Condition/mach", ptyp, dspace, plist);
    dset.write(&cond.mach, ptyp);

    attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf);
    attr.close();

    dset.close();

    // add Reynolds number
    dset = file_->createDataSet("/Condition/reynolds_number", ptyp, dspace, plist);
    dset.write(&cond.rey, ptyp);

    H5std_string strwritebuf3("millions/ft");
    strwritebuf3.resize(H5_STRING_LEN, '\0'); // pad with \0
    attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf3);
    attr.close();

    dset.close();

    // add total pressure
    dset = file_->createDataSet("/Condition/total_pressure", ptyp, dspace, plist);
    dset.write(&cond.ptot, ptyp);

    H5std_string strwritebuf4("psf");
    strwritebuf4.resize(H5_STRING_LEN, '\0'); // pad with \0
    attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf4);
    attr.close();

    dset.close();

    // add dynamic pressure
    dset = file_->createDataSet("/Condition/dynamic_pressure", ptyp, dspace, plist);
    dset.write(&cond.qbar, ptyp);

    attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf4);
    attr.close();

    dset.close();

    // add total temperature
    dset = file_->createDataSet("/Condition/total_temperature", ptyp, dspace, plist);
    dset.write(&cond.ttot, ptyp);

    H5std_string strwritebuf5("degF");
    strwritebuf5.resize(H5_STRING_LEN, '\0'); // pad with \0
    attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf5);
    attr.close();

    dset.close();

    // add thermocouple average
    dset = file_->createDataSet("/Condition/thermocouple_average_temperature", ptyp, dspace, plist);
    dset.write(&cond.tcavg, ptyp);

    H5std_string strwritebuf6("degF");
    strwritebuf6.resize(H5_STRING_LEN, '\0'); // pad with \0
    attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf6);
    attr.close();

    dset.close();

    // add static pressure
    dset = file_->createDataSet("/Condition/static_pressure", ptyp, dspace, plist);
    dset.write(&cond.ps, ptyp);

    attr = dset.createAttribute("units",
            strdatatype, dspace);
    attr.write(strdatatype, strwritebuf4);
    attr.close();

    dset.close();

    dspace.close();

    } catch (...) {
        throw(std::invalid_argument("Could not write wind tunnel conditions" + 
              std::string(" to psp hdf5 file: ")));
    }
}

/*****************************************************************************/
template<typename Model>
void PSPWriter<Model>::write_code_version(std::string code_version) {
    write_string_attribute("code_version", code_version);
}

/*****************************************************************************/
template<typename Model>
void PSPWriter<Model>::write_string_attribute(std::string attr_name, 
        std::string attr_value) {

    try {
    H5::Group root_group = file_->openGroup("/");
    hsize_t attr_dims[] = { 1 };
    H5::DataSpace attr_dataspace(1, attr_dims);

    H5::StrType strdatatype(H5::PredType::C_S1, H5_STRING_LEN);
    H5std_string strwritebuf(attr_value);
    strwritebuf.resize(H5_STRING_LEN, '\0'); // pad with \0

    H5::Attribute attr = root_group.createAttribute(attr_name,
            strdatatype, attr_dataspace);
    attr.write(strdatatype, strwritebuf);

    attr.close();
    attr_dataspace.close();

    } catch (H5::GroupIException error) {
        throw(std::invalid_argument("Could not write wind tunnel conditions" + 
              std::string(" to psp hdf5 file: ") + error.getDetailMsg()));
    } catch (H5::AttributeIException error) {
        throw(std::invalid_argument("Could not write wind tunnel conditions" +
              std::string(" to psp hdf5 file: ") + error.getDetailMsg()));
    }
}

/*****************************************************************************/
template<typename Model>
void PSPWriter<Model>::set_chunk_size(float chunk_size) {
    chunk_size_ = chunk_size;
    set_chunk_length();
}

/*****************************************************************************/
template<typename Model>
template<typename T>
void PSPWriter<Model>::nominal_write_solution(unsigned int sol_id, 
        const std::vector<T>& sol) {
    static_assert(psph5_valid_type<T>(), "Solution must be float or double");

    if (sol.size() % data_pts_ != 0) {
        throw std::invalid_argument("Must have solution at every grid point");
    }

    unsigned int n_frames = sol.size() / data_pts_;

    //std::cout << "pspwrite " << n_frames << " frames " << std::endl;

    try {

    // define hdf5 data type for grid point values
    H5::DataType ptyp;
    if (sizeof(T) == 4) {
        ptyp = H5::PredType::NATIVE_FLOAT;
    } else if (sizeof(T) == 8) {
        ptyp = H5::PredType::NATIVE_DOUBLE;
    }

    // define property list
    T fillValue = 0.0;
    H5::DSetCreatPropList plist;
    plist.setFillValue(ptyp, &fillValue);

    // if first frame, create data set, prepare chunking and compression
    // else extend dataset
    if (curr_frame_[sol_id] == 0) {
        hsize_t chunk_dims[] = {(hsize_t) std::min(chunk_length_, num_frames_), 
                                (hsize_t) data_pts_};
        if (chunk_dims[0] == 0) {
            chunk_dims[0] = chunk_length_;
        }

        plist.setChunk(2, chunk_dims);
        plist.setDeflate(6);

        assert(!open_dataset_[sol_id]);

        hsize_t mdims[] = {H5S_UNLIMITED, (hsize_t) data_pts_};
        if (num_frames_ != 0) {
            mdims[0] = num_frames_;
            H5::DataSpace dspace(2, mdims);
            dataset_[sol_id] = file_->createDataSet(sols_[sol_id], ptyp, dspace, plist);
            dspace.close();
        } else {
            hsize_t cdims[] = {1, (hsize_t) data_pts_};
            H5::DataSpace dspace(2, cdims, mdims);
            dataset_[sol_id] = file_->createDataSet(sols_[sol_id], ptyp, dspace, plist);
            dspace.close();
        }

        open_dataset_[sol_id] = true;

    } else if (num_frames_ == 0) {
        hsize_t fdim2[] = {(hsize_t) (curr_frame_[sol_id] + 1), 
                (hsize_t) data_pts_};
        dataset_[sol_id].extend( fdim2 );
    }

    assert(open_dataset_[sol_id]);

    //hsize_t offset[] = {(hsize_t) curr_frame_[sol_id], 0};
    //hsize_t fdim[] = {n_frames, (hsize_t) data_pts_};

    //H5::DataSpace fspace(2, fdim);
    //H5::DataSpace mspace = dataset_[sol_id].getSpace();
    //mspace.selectHyperslab( H5S_SELECT_SET, fdim, offset);

    hsize_t offset[] = {(hsize_t) curr_frame_[sol_id], 0};
    hsize_t fdim[] = {(hsize_t) n_frames, (hsize_t) data_pts_};
    hsize_t count[] = {1,1};

    H5::DataSpace fspace(2, fdim);
    H5::DataSpace mspace = dataset_[sol_id].getSpace();
    mspace.selectHyperslab( H5S_SELECT_SET, count, offset, nullptr, fdim );

    dataset_[sol_id].write(&sol[0], ptyp, fspace, mspace);

    mspace.close();
    fspace.close();

    curr_frame_[sol_id] += n_frames;
    
    } // end try block 
    catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not write frame " + 
            std::to_string(curr_frame_[sol_id]) + " to psp hdf5 file: " + 
            error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not write frame " + 
            std::to_string(curr_frame_[sol_id]) + " to psp hdf5 file: " + 
            error.getDetailMsg()));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not write frame " + 
            std::to_string(curr_frame_[sol_id]) + " to psp hdf5 file: " + 
            error.getDetailMsg()));
    }

}

/*****************************************************************************/
template<typename Model>
template<typename T>
void PSPWriter<Model>::transpose_write_solution(unsigned int sol_id, 
        const std::vector<T>& sol) {
    static_assert(psph5_valid_type<T>(), "Solution must be float or double");

    if ((sol.size() == 0) || (sol.size() % data_pts_ != 0)) {
        throw std::invalid_argument("Must have solution at every grid point");
    }

    try {

    // define hdf5 data type for grid point values
    H5::DataType ptyp;
    if (sizeof(T) == 4) {
        ptyp = H5::PredType::NATIVE_FLOAT;
    } else if (sizeof(T) == 8) {
        ptyp = H5::PredType::NATIVE_DOUBLE;
    }

    // define property list
    T fillValue = 0.0;
    H5::DSetCreatPropList plist;
    plist.setFillValue(ptyp, &fillValue);

    // if first frame, create data set, prepare chunking and compression
    if (curr_frame_[sol_id] == 0) {
        hsize_t chunk_dims[] = {(hsize_t) trans_nodes_, (hsize_t) chunk_length_};

        plist.setChunk(2, chunk_dims);
        plist.setDeflate(6);

        assert(!open_dataset_[sol_id]);

        hsize_t mdims[] = {(hsize_t) data_pts_, (hsize_t) num_frames_};
        H5::DataSpace dspace(2, mdims);

        dataset_[sol_id] = file_->createDataSet(sols_[sol_id], ptyp, dspace, plist);
        open_dataset_[sol_id] = true;

        dspace.close();
    }

    assert(open_dataset_[sol_id]);

    hsize_t write_frames = sol.size() / data_pts_;

    hsize_t offset[] = {0, (hsize_t) curr_frame_[sol_id]};
    hsize_t fdim[] = {(hsize_t) data_pts_, (hsize_t) write_frames};
    hsize_t count[] = {1,1};

    H5::DataSpace fspace(2, fdim);
    H5::DataSpace mspace = dataset_[sol_id].getSpace();
    //mspace.selectHyperslab( H5S_SELECT_SET, fdim, offset);
    mspace.selectHyperslab( H5S_SELECT_SET, count, offset, nullptr, fdim );

    dataset_[sol_id].write(&sol[0], ptyp, fspace, mspace);

    mspace.close();
    fspace.close();

    curr_frame_[sol_id] += write_frames;
    
    } // end try block 
    catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not write frame " + 
            std::to_string(curr_frame_[sol_id]) + " to psp hdf5 file: " + 
            error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not write frame " + 
            std::to_string(curr_frame_[sol_id]) + " to psp hdf5 file: " + 
            error.getDetailMsg()));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not write frame " + 
            std::to_string(curr_frame_[sol_id]) + " to psp hdf5 file: " + 
            error.getDetailMsg()));
    }

}

/*****************************************************************************/
template<typename Model>
template<typename T>
void PSPWriter<Model>::transpose_write_solution2(unsigned int sol_id,
        const std::vector<T>& sol) {
    static_assert(psph5_valid_type<T>(), "Solution must be float or double");

    if ((sol.size() == 0) || (sol.size() % num_frames_ != 0)) {
        throw std::invalid_argument("Must have solution at every frame");
    }

    try {

    // define hdf5 data type for grid point values
    H5::DataType ptyp;
    if (sizeof(T) == 4) {
        ptyp = H5::PredType::NATIVE_FLOAT;
    } else if (sizeof(T) == 8) {
        ptyp = H5::PredType::NATIVE_DOUBLE;
    }

    // define property list
    T fillValue = 0.0;
    H5::DSetCreatPropList plist;
    plist.setFillValue(ptyp, &fillValue);

    // if first write, create data set, prepare chunking and compression
    // use curr_frame_ as curr_pt_
    if (curr_pt_[sol_id] == 0) {
        hsize_t chunk_dims[] = {(hsize_t) trans_nodes_, (hsize_t) chunk_length_};

        plist.setChunk(2, chunk_dims);
        plist.setDeflate(6);

        assert(!open_dataset_[sol_id]);

        hsize_t mdims[] = {(hsize_t) data_pts_, (hsize_t) num_frames_};
        H5::DataSpace dspace(2, mdims);

        dataset_[sol_id] = file_->createDataSet(sols_[sol_id], ptyp, dspace, plist);
        open_dataset_[sol_id] = true;

        dspace.close();
    }

    assert(open_dataset_[sol_id]);

    hsize_t write_nodes = sol.size() / num_frames_;

    hsize_t offset[] = {(hsize_t) curr_pt_[sol_id], 0};
    hsize_t fdim[] = {(hsize_t) write_nodes, (hsize_t) num_frames_};
    hsize_t count[] = {1,1};

    H5::DataSpace fspace(2, fdim);
    H5::DataSpace mspace = dataset_[sol_id].getSpace();
    mspace.selectHyperslab( H5S_SELECT_SET, count, offset, nullptr, fdim );

    dataset_[sol_id].write(&sol[0], ptyp, fspace, mspace);

    mspace.close();
    fspace.close();

    curr_pt_[sol_id] += write_nodes;
    
    } // end try block 
    catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not write pt " + 
            std::to_string(curr_pt_[sol_id]) + " to psp hdf5 file: " + 
            error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not write pt " + 
            std::to_string(curr_pt_[sol_id]) + " to psp hdf5 file: " + 
            error.getDetailMsg()));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not write pt " + 
            std::to_string(curr_pt_[sol_id]) + " to psp hdf5 file: " + 
            error.getDetailMsg()));
    }

}

/*****************************************************************************/
template<typename Model>
void PSPWriter<Model>::get_data_type(H5::DataType& ptyp) const {
    if (sizeof(mtype) == 4) {
        ptyp = H5::PredType::NATIVE_FLOAT;
    } else if (sizeof(mtype) == 8) {
        ptyp = H5::PredType::NATIVE_DOUBLE;
    }
}

/*****************************************************************************/
template<typename Model>
void PSPWriter<Model>::set_chunk_length() {
    if (transposed_) {
        chunk_length_ = std::min(std::max(1, 
                (int) round(chunk_size_ * 1e6 / (sizeof(mtype)*trans_nodes_))), 
                (int) num_frames_);
    } else {
        chunk_length_ = std::max(1,
                (int) round(chunk_size_ * 1e6 / (sizeof(mtype)*data_pts_)));
        if (num_frames_ != 0) {
            chunk_length_ = std::min(chunk_length_, num_frames_);
        }
    }
}

/********************************************************************
 * Functions for reading and manipulating upsp HDF5 files
********************************************************************/

/*****************************************************************************/
template<typename Model>
std::string hdf5_read_grid(std::string filename, Model*& model) {

    std::string units = "";

    if constexpr(is_structured<Model>()) {
        if (!hdf5_is_structured(filename)) {
            throw(std::invalid_argument("HDF5 file has unstructured grid, cannot read into structured model"));
        }
        units = hdf5_read_structured_grid(filename, model);
    } else {
        if (hdf5_is_structured(filename)) {
            throw(std::invalid_argument("HDF5 file has structured grid, canot read into unstructured model"));
        }
        units = hdf5_read_unstructured_grid(filename, model);
    }
    return units;
}
        
/*****************************************************************************/
template<typename Model>
std::string hdf5_read_structured_grid(std::string filename, Model*& model) {

    static_assert(is_structured<Model>());

    // get file version
    unsigned int file_ver = hdf5_get_version(filename);

    // Create structured grid
    using mtype = typename Model::data_type;
    StructuredGrid<mtype> grid;

    std::string units = "";

    try {

        // Open the file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // Open the group
        H5::Group group;
        if (file_ver == 0) {
            group = file.openGroup("/");
        } else {
            group = file.openGroup("/Grid");
        }

        // Get the number of zones
        std::string dset_name = "grid_sizes";
        if (file_ver == 0) {
            dset_name = "Grid_Sizes";
        }
        H5::DataSet gset = group.openDataSet(dset_name);
        H5::DataSpace gspace = gset.getSpace();
        int n_dims = gspace.getSimpleExtentNdims();
        assert(n_dims == 2);
        hsize_t dims[n_dims];
        gspace.getSimpleExtentDims(dims, nullptr);
        assert(dims[1] == 3);

        hsize_t zones = dims[0];

        grid.grid_size.resize(zones);
    
        // Set the grid sizes
        int gsize[zones*dims[1]];
        gset.read(gsize, H5::PredType::NATIVE_INT);
        unsigned int count = 0;
        unsigned int tot_pts = 0;
        for (unsigned int i=0; i < zones; ++i) {
            grid.grid_size[i].resize(dims[1]);
            for (unsigned int j=0; j < dims[1]; ++j) {
                grid.grid_size[i][j] = gsize[count++];
            }
            tot_pts += grid.zone_size(i);
        }

        gset.close();
        gspace.close();

        // Set data type
        H5::DataType ptyp;
        if (sizeof(mtype) == 4) {
            ptyp = H5::PredType::NATIVE_FLOAT;
        } else if (sizeof(mtype) == 8) {
            ptyp = H5::PredType::NATIVE_DOUBLE;
        }

        // Get the grid point vectors
        dset_name = "x";
        if (file_ver == 0) {
            dset_name = "X";
        }
        H5::DataSet x_dataset = group.openDataSet(dset_name);
        grid.x.resize(tot_pts);
        x_dataset.read(&grid.x[0], ptyp);

        dset_name = "y";
        if (file_ver == 0) {
            dset_name = "Y";
        }
        H5::DataSet y_dataset = group.openDataSet(dset_name);
        grid.y.resize(tot_pts);
        y_dataset.read(&grid.y[0], ptyp);

        dset_name = "z";
        if (file_ver == 0) {
            dset_name = "Z";
        }
        H5::DataSet z_dataset = group.openDataSet(dset_name);
        grid.z.resize(tot_pts);
        z_dataset.read(&grid.z[0], ptyp);

        // Read in the units
        if (file_ver > 0) {
            H5::Attribute attr = group.openAttribute("units");
            H5::DataType dt = attr.getDataType();
            units.reserve(H5_STRING_LEN); // just in case it isn't allocated
            attr.read(dt, units);
            dt.close();
            attr.close();
        }

        ptyp.close();
        x_dataset.close();
        y_dataset.close();
        z_dataset.close();
        group.close();
        file.close();

    } // end try block

    catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file grid: " 
                + error.getDetailMsg()));
    }
    catch (H5::GroupIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file grid: " 
                + error.getDetailMsg()));
    }
    catch (H5::DataTypeIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file grid: " 
                + error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file grid: " 
                + error.getDetailMsg()));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file grid: " 
                + error.getDetailMsg()));
    }       

    // Generate model
    model = new Model(grid);

    return units;
}

/*****************************************************************************/
template<typename Model>
std::string hdf5_read_unstructured_grid(std::string filename, Model*& model) {

    static_assert(!is_structured<Model>());

    // get file version
    unsigned int file_ver = hdf5_get_version(filename);

    assert(file_ver > 0);

    // Create structured grid
    using mtype = typename Model::data_type;
    UnstructuredGrid<mtype> grid;

    std::string units = "";

    try {

        // Open the file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // Open the group
        H5::Group group(file.openGroup("/Grid"));

        // Set data type
        H5::DataType ptyp;
        if (sizeof(mtype) == 4) {
            ptyp = H5::PredType::NATIVE_FLOAT;
        } else if (sizeof(mtype) == 8) {
            ptyp = H5::PredType::NATIVE_DOUBLE;
        }

        // Get the grid point vectors
        H5::DataSet x_dataset = group.openDataSet("x");
        H5::DataSpace xspace = x_dataset.getSpace();
        int xn_dims = xspace.getSimpleExtentNdims();
        assert(xn_dims == 1);
        hsize_t x_dims[xn_dims];
        xspace.getSimpleExtentDims(x_dims, nullptr);
        unsigned int tot_pts = x_dims[0];

        grid.x.resize(tot_pts);
        x_dataset.read(&grid.x[0], ptyp);

        H5::DataSet y_dataset = group.openDataSet("y");
        grid.y.resize(tot_pts);
        y_dataset.read(&grid.y[0], ptyp);

        H5::DataSet z_dataset = group.openDataSet("z");
        grid.z.resize(tot_pts);
        z_dataset.read(&grid.z[0], ptyp);

        ptyp.close();
        x_dataset.close();
        y_dataset.close();
        z_dataset.close();

        // Determine the number of triangles
        H5::DataSet tri_dset = group.openDataSet("triangles");
        H5::DataSpace gspace = tri_dset.getSpace();
        int n_dims = gspace.getSimpleExtentNdims();
        assert(n_dims == 2);
        hsize_t dims[n_dims];
        gspace.getSimpleExtentDims(dims, nullptr);
        assert(dims[1] == 3);

        hsize_t n_tris = dims[0];

        // Read in the triangles
        std::vector<unsigned int> tris(n_tris*3);
        tri_dset.read(&tris[0], H5::PredType::NATIVE_UINT);

        grid.tris.resize(n_tris);
        for (unsigned int i=0; i < n_tris; ++i) {
            grid.tris[i] = {tris[3*i],tris[3*i+1],tris[3*i+2]};
        }

        tri_dset.close();
        gspace.close();

        // Read in the components
        H5::DataSet comp_dset = group.openDataSet("components");
        grid.comps.resize(n_tris);
        comp_dset.read(&grid.comps[0], H5::PredType::NATIVE_INT);

        comp_dset.close();

        // Read in the units
        H5::Attribute attr = group.openAttribute("units");
        H5::DataType dt = attr.getDataType();
        units.reserve(H5_STRING_LEN); // just in case it isn't allocated
        attr.read(dt, units);
        dt.close();
        attr.close();
            
        group.close();
        file.close();

    } // end try block

    catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file grid: " 
                + error.getDetailMsg()));
    }
    catch (H5::GroupIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file grid: " 
                + error.getDetailMsg()));
    }
    catch (H5::DataTypeIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file grid: " 
                + error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file grid: " 
                + error.getDetailMsg()));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file grid: " 
                + error.getDetailMsg()));
    }       

    // Generate model
    model = new Model(grid);

    return units;
}

/*****************************************************************************/
template<typename T>
void hdf5_read_solution(std::string filename, unsigned int frame, std::vector<T>& sol) {
    hdf5_read_solution(filename, "frames", frame, sol);
}

/*****************************************************************************/
template<typename T>
void hdf5_read_solution(std::string filename, std::string name, 
                        unsigned int frame, std::vector<T>& sol) {
    static_assert(psph5_valid_type<T>(), "Solution must be float or double");

    // check that frame exists in file
    unsigned int num_frames = hdf5_num_frames(filename);
    if (frame >= num_frames) {
        throw std::invalid_argument("Cannot read frame " + std::to_string(frame) + ", only " + std::to_string(num_frames) + " in file");
    } 

    try {
        // Open the file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // define hdf5 data type for grid point values
        H5::DataType ptyp;
        if (sizeof(T) == 4) {
            ptyp = H5::PredType::NATIVE_FLOAT;
        } else if (sizeof(T) == 8) {
            ptyp = H5::PredType::NATIVE_DOUBLE;
        }

        // Get the size of the dataset
        unsigned int num_pts = hdf5_num_pts(filename);

        // Read the dataset
        H5::DataSet dataset = file.openDataSet(name);
        H5::DataSpace dspace = dataset.getSpace();

        hsize_t mdim[] = {1, num_pts};
        hsize_t offset[] = {(hsize_t) frame, 0};

        dspace.selectHyperslab( H5S_SELECT_SET, mdim, offset );
        H5::DataSpace mspace(2, mdim);

        sol.resize(num_pts);
        dataset.read(&sol[0], ptyp, mspace, dspace);

        ptyp.close();
        dataset.close();
        dspace.close();
        mspace.close();
        file.close();

    } // end try block 
    catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file solution: " 
                + error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file solution: " 
                + error.getDetailMsg()));
    }
    catch (H5::DataTypeIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file solution: " 
                + error.getDetailMsg()));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file solution: " 
                + error.getDetailMsg()));
    }

}

/*****************************************************************************/
template<typename T>
void hdf5_read_solution(std::string filename, unsigned int frame, std::vector<T>& sol,
    unsigned int grid_size) {
    static_assert(psph5_valid_type<T>(), "Solution must be float or double");

    // check that frame exists in file
    unsigned int num_frames = hdf5_num_frames(filename);
    if (frame >= num_frames) {
        throw std::invalid_argument("Cannot read frame " + std::to_string(frame) + ", only " + std::to_string(num_frames) + " in file");
    } 

    try {
        // Open the file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // define hdf5 data type for grid point values
        H5::DataType ptyp;
        if (sizeof(T) == 4) {
            ptyp = H5::PredType::NATIVE_FLOAT;
        } else if (sizeof(T) == 8) {
            ptyp = H5::PredType::NATIVE_DOUBLE;
        }

        // Open the dataset
        H5::DataSet dataset = file.openDataSet("frames");
        H5::DataSpace dspace = dataset.getSpace();

        // Read the dataset
        hsize_t mdim[] = {1, grid_size};
        hsize_t offset[] = {(hsize_t) frame, 0};

        dspace.selectHyperslab( H5S_SELECT_SET, mdim, offset );
        H5::DataSpace mspace(2, mdim);

        sol.resize(grid_size);
        dataset.read(&sol[0], ptyp, mspace, dspace);

        ptyp.close();
        dataset.close();
        dspace.close();
        mspace.close();
        file.close();

    } // end try block 
    catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataTypeIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }

}

/*****************************************************************************/
template<typename T>
void hdf5_read_solution_zone(std::string filename, unsigned int frame, unsigned int zone,
        std::vector<T>& sol) {
    static_assert(psph5_valid_type<T>(), "Solution must be float or double");

    // check format of HDF5 (structured or unstructured)
    if (!hdf5_is_structured(filename)) {
        throw(std::invalid_argument("Cannot handle unstructured grid"));
    }
    // check that frame exists in file
    unsigned int num_frames = hdf5_num_frames(filename);
    if (frame >= num_frames) {
        throw std::invalid_argument("Cannot read frame " + 
                std::to_string(frame) + ", only " + std::to_string(num_frames) + 
                " in file");
    } 

    // get the file version
    unsigned int file_ver = hdf5_get_version(filename);

    try {
        // Open the file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // Open the group
        H5::Group group;
        if (file_ver == 0) {
            group = file.openGroup("/");
        } else {
            group = file.openGroup("/Grid");
        }

        // define hdf5 data type for grid point values
        H5::DataType ptyp;
        if (sizeof(T) == 4) {
            ptyp = H5::PredType::NATIVE_FLOAT;
        } else if (sizeof(T) == 8) {
            ptyp = H5::PredType::NATIVE_DOUBLE;
        }

        // Get number of zones
        std::string dset_name = "grid_sizes";
        if (file_ver == 0) {
            dset_name = "Grid_Sizes";
        }
        H5::DataSet gset = group.openDataSet(dset_name);
        H5::DataSpace gspace = gset.getSpace();
        int n_dims = gspace.getSimpleExtentNdims();
        assert(n_dims == 2);
        hsize_t dims[n_dims];
        gspace.getSimpleExtentDims(dims, nullptr);
        assert(dims[1] == 3);

        hsize_t zones = dims[0];

        if (zone >= zones) {
            throw std::invalid_argument("Cannot read zone " + 
                    std::to_string(zone) + ", only " + std::to_string(zones) + 
                    " in file");
        } 

        // Set the grid sizes
        int gsize[zones][3];
        gset.read(gsize, H5::PredType::NATIVE_INT);
        unsigned int idx = 0;
        for (unsigned int i=0; i < zone; ++i) {
            idx += gsize[i][0] * gsize[i][1] * gsize[i][2];
        }
        unsigned int grid_size = gsize[zone][0] * gsize[zone][1] * gsize[zone][2];

        gset.close();
        gspace.close(); 

        // Open the dataset
        H5::DataSet dataset = file.openDataSet("frames");
        H5::DataSpace dspace = dataset.getSpace();

        // Get the hyperslab for reading 
        hsize_t mdim[] = {1, grid_size};
        hsize_t offset[] = {(hsize_t) frame, (hsize_t) idx};

        dspace.selectHyperslab( H5S_SELECT_SET, mdim, offset );
        H5::DataSpace mspace(2, mdim);

        sol.resize(grid_size);
        dataset.read(&sol[0], ptyp, mspace, dspace);

        dataset.close();
        mspace.close();
        dspace.close();
        group.close();
        file.close();

    } // end try block 
    catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::GroupIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataTypeIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }
    
}

/*****************************************************************************/
template<typename T>
void hdf5_read_solution(std::string filename, std::string name, 
        unsigned int frame_start, unsigned int frame_delta, 
        unsigned int frame_end, std::vector<T>& sol) {
    static_assert(psph5_valid_type<T>(), "Solution must be float or double");

    sol.clear();

    // check for read no frames
    if (frame_end < frame_start){
        return;
    }

    // check that frame range exists in file
    unsigned int total_frames = hdf5_num_frames(filename);
    if (frame_end >= total_frames){
        throw std::invalid_argument("Cannot read frames " + 
                std::to_string(frame_start) + " to " + std::to_string(frame_end) 
                + ", only " + std::to_string(total_frames) + " in file");
    }

    if (frame_end < 0) {
        frame_end = total_frames - 1;
        if (frame_end < frame_start) {
            return;
        }
    }

    unsigned int read_frames = (frame_end + 1 - frame_start + frame_delta -1)/ frame_delta;
    unsigned int n_pts = hdf5_num_pts(filename);

    try {
        // Open the file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // define hdf5 data type for grid point values
        H5::DataType ptyp;
        if (sizeof(T) == 4) {
            ptyp = H5::PredType::NATIVE_FLOAT;
        } else if (sizeof(T) == 8) {
            ptyp = H5::PredType::NATIVE_DOUBLE;
        }

        // open the dataset
        H5::DataSet dataset = file.openDataSet(name);
        H5::DataSpace dspace = dataset.getSpace();

        // Read the dataset
        hsize_t mdim[] = { (hsize_t) read_frames, (hsize_t) n_pts};
        hsize_t block[] = { (hsize_t) 1, (hsize_t) n_pts};
        hsize_t offset[] = {(hsize_t) frame_start, (hsize_t) 0};
        hsize_t stride[] = {(hsize_t) frame_delta, 1};
        hsize_t count[] = {(hsize_t) read_frames, 1};

        dspace.selectHyperslab( H5S_SELECT_SET, count, offset, stride, block );
        H5::DataSpace mspace(2, mdim);

        sol.resize(n_pts*read_frames);
        dataset.read(&sol[0], ptyp, mspace, dspace);

        ptyp.close();
        dataset.close();
        dspace.close();
        mspace.close();
        file.close();

    } // end try block 
    catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataTypeIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }

}

/*****************************************************************************/
template<typename T>
void hdf5_read_transpose_solution(std::string filename, unsigned int pt_start, 
        unsigned int pt_end, std::vector<T>& sol, unsigned int frame_start/*=0*/,
        unsigned int frame_incr/*=1*/, int frame_end/*=-1*/) {

    hdf5_read_transpose_solution(filename, "frames", pt_start, pt_end, 
            sol, frame_start, frame_incr, frame_end);
}

/*****************************************************************************/
template<typename T>
void hdf5_read_transpose_solution(std::string filename, std::string dset_name,
        unsigned int pt_start, 
        unsigned int pt_end, std::vector<T>& sol, unsigned int frame_start/*=0*/,
        unsigned int frame_incr/*=1*/, int frame_end/*=-1*/) {
    static_assert(psph5_valid_type<T>(), "Solution must be float or double");

    // check for valid node range
    if (pt_end < pt_start) {
        throw(std::invalid_argument("Cannot read transpose solution, " + 
                std::string("pt_end must be greater than or equal to pt_start")));
    }

    // check for valid frame range
    if (frame_end > -1) {
        // check for read no frames
        if (frame_end < frame_start){
            sol.clear();
            return;
        }
    }

    // check that node range exists in file
    unsigned int num_pts = hdf5_num_pts(filename);
    if (pt_end >= num_pts) {
        throw std::invalid_argument("Cannot read nodes " + 
                std::to_string(pt_start) + " to " + std::to_string(pt_end) 
                + ", only " + std::to_string(num_pts) + " in file");
    } 

    // check that frame range exists in file
    unsigned int total_frames = hdf5_num_frames(filename);
    if (frame_end >= static_cast<int>(total_frames)){
        throw std::invalid_argument("Cannot read frames " + 
                std::to_string(frame_start) + " to " + std::to_string(frame_end) 
                + ", only " + std::to_string(total_frames) + " in file");
    }

    if (frame_end < 0) {
        frame_end = total_frames - 1;
        if (frame_end < frame_start) {
            sol.clear();
            return;
        }
    }

    unsigned int read_frames = (frame_end + 1 - frame_start + frame_incr -1)/ frame_incr;
    
    try {
        // Open the file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // define hdf5 data type for grid point values
        H5::DataType ptyp;
        if (sizeof(T) == 4) {
            ptyp = H5::PredType::NATIVE_FLOAT;
        } else if (sizeof(T) == 8) {
            ptyp = H5::PredType::NATIVE_DOUBLE;
        }

        // open the dataset
        H5::DataSet dataset = file.openDataSet(dset_name);
        H5::DataSpace dspace = dataset.getSpace();

        // Read the dataset
        hsize_t mdim[] = { (hsize_t) pt_end-pt_start+1, (hsize_t) read_frames};
        hsize_t block[] = { (hsize_t) pt_end-pt_start+1, 1};
        hsize_t offset[] = {(hsize_t) pt_start, (hsize_t) frame_start};
        hsize_t stride[] = {1, (hsize_t) frame_incr};
        hsize_t count[] = {1, read_frames};

        //dspace.selectHyperslab( H5S_SELECT_SET, mdim, offset );
        dspace.selectHyperslab( H5S_SELECT_SET, count, offset, stride, block );
        H5::DataSpace mspace(2, mdim);

        sol.resize((pt_end-pt_start+1)*read_frames);
        dataset.read(&sol[0], ptyp, mspace, dspace);

        ptyp.close();
        dataset.close();
        dspace.close();
        mspace.close();
        file.close();

    } // end try block 
    catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataTypeIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not read psp hdf5 file: " + error.getDetailMsg()));
    }

}

/*****************************************************************************/
template<typename T>
std::string hdf5_read_dataset(std::string filename, std::string name, 
        std::vector<T>& sol) {

    static_assert(psph5_valid_type<T>(), "Dataset must be float or double");

    try {
        // Open the file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // define hdf5 data type for grid point values
        H5::DataType ptyp;
        if (sizeof(T) == 4) {
            ptyp = H5::PredType::NATIVE_FLOAT;
        } else if (sizeof(T) == 8) {
            ptyp = H5::PredType::NATIVE_DOUBLE;
        }

        // Open the dataset
        H5::DataSet dataset = file.openDataSet(name);
        H5::DataSpace dspace = dataset.getSpace();

        // get the size of the dataset
        int n_dims = dspace.getSimpleExtentNdims();
        assert(n_dims == 1);
        hsize_t dims[n_dims];
        dspace.getSimpleExtentDims(dims, nullptr);

        // Read the dataset
        hsize_t mdim[] = {dims[0]};
        hsize_t offset[] = {0};

        dspace.selectHyperslab( H5S_SELECT_SET, mdim, offset );
        H5::DataSpace mspace(1, mdim);

        sol.resize(dims[0]);
        dataset.read(&sol[0], ptyp, mspace, dspace);

        ptyp.close();
        dataset.close();
        dspace.close();
        mspace.close();
        file.close();

    } // end try block 
    catch (H5::FileIException error) {
        throw(std::invalid_argument("Could not read dataset " + name + 
                " from psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Could not read dataset " + name + 
                " from psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataTypeIException error) {
        throw(std::invalid_argument("Could not read dataset " + name + 
                " from psp hdf5 file: " + error.getDetailMsg()));
    }
    catch (H5::DataSpaceIException error) {
        throw(std::invalid_argument("Could not read dataset " + name + 
                " from psp hdf5 file: " + error.getDetailMsg()));
    }

    return hdf5_read_dataset_units(filename, name);
}

/*****************************************************************************/
template<typename Cond>
void hdf5_read_tunnel_conditions(std::string filename, Cond& cond) {
    
    try {

        // get the version number
        unsigned int file_ver = hdf5_get_version(filename);

        // open file
        H5::H5File file(filename, H5F_ACC_RDONLY);

        // open goup
        H5::Group group;
        if (file_ver == 0) {
            group = file.openGroup("/");
        } else {
            group = file.openGroup("/Condition");
        }

        if (file_ver == 0) {
            // read float attributes
            H5::Attribute attr = group.openAttribute("Alpha");
            attr.read(H5::PredType::NATIVE_FLOAT, &cond.alpha);
            attr.close();

            attr = group.openAttribute("Beta");
            attr.read(H5::PredType::NATIVE_FLOAT, &cond.beta);
            attr.close();
            attr = group.openAttribute("Phi");
            attr.read(H5::PredType::NATIVE_FLOAT, &cond.phi);
            attr.close();
            attr = group.openAttribute("Mach");
            attr.read(H5::PredType::NATIVE_FLOAT, &cond.mach);
            attr.close();
            attr = group.openAttribute("Reynolds Number");
            attr.read(H5::PredType::NATIVE_FLOAT, &cond.rey);
            attr.close();
            attr = group.openAttribute("Total Pressure");
            attr.read(H5::PredType::NATIVE_FLOAT, &cond.ptot);
            attr.close();
            attr = group.openAttribute("Dynamic Pressure");
            attr.read(H5::PredType::NATIVE_FLOAT, &cond.qbar);
            attr.close();
            attr = group.openAttribute("Total Temperature");
            attr.read(H5::PredType::NATIVE_FLOAT, &cond.ttot);
            attr.close();
            attr = group.openAttribute("Static Pressure");
            attr.read(H5::PredType::NATIVE_FLOAT, &cond.ps);
            attr.close();

            // read integer attributes
            attr = group.openAttribute("Run");
            attr.read(H5::PredType::NATIVE_INT, &cond.run);
            attr.close();
            attr = group.openAttribute("Sequence");
            attr.read(H5::PredType::NATIVE_INT, &cond.seq);
            attr.close();

            // read string attributes
            attr = group.openAttribute("Test Number");
            H5::DataType dt = attr.getDataType();
            cond.test_id.reserve(H5_STRING_LEN); // just in case it isn't allocated
            attr.read(dt, cond.test_id);
            dt.close();
            attr.close();
        } else {

            // prepare to read datasets
            hsize_t mdim[] = {1};
            H5::DataSpace mspace(1, mdim);

            // read float attributes
            H5::DataSet dset = group.openDataSet("alpha");
            dset.read(&cond.alpha, H5::PredType::NATIVE_FLOAT, mspace, mspace); 
            dset.close();

            dset = group.openDataSet("beta");
            dset.read(&cond.beta, H5::PredType::NATIVE_FLOAT, mspace, mspace);
            dset.close();
            dset = group.openDataSet("phi");
            dset.read(&cond.phi, H5::PredType::NATIVE_FLOAT, mspace, mspace);
            dset.close();
            dset = group.openDataSet("mach");
            dset.read(&cond.mach, H5::PredType::NATIVE_FLOAT, mspace, mspace);
            dset.close();
            dset = group.openDataSet("reynolds_number");
            dset.read(&cond.rey, H5::PredType::NATIVE_FLOAT, mspace, mspace);
            dset.close();
            dset = group.openDataSet("total_pressure");
            dset.read(&cond.ptot, H5::PredType::NATIVE_FLOAT, mspace, mspace);
            dset.close();
            dset = group.openDataSet("dynamic_pressure");
            dset.read(&cond.qbar, H5::PredType::NATIVE_FLOAT, mspace, mspace);
            dset.close();
            dset = group.openDataSet("total_temperature");
            dset.read(&cond.ttot, H5::PredType::NATIVE_FLOAT, mspace, mspace);
            dset.close();
            dset = group.openDataSet("thermocouple_average_temperature");
            dset.read(&cond.tcavg, H5::PredType::NATIVE_FLOAT, mspace, mspace);
            dset.close();
            dset = group.openDataSet("static_pressure");
            dset.read(&cond.ps, H5::PredType::NATIVE_FLOAT, mspace, mspace);
            dset.close();

            // read integer attributes
            dset = group.openDataSet("run");
            dset.read(&cond.run, H5::PredType::NATIVE_INT, mspace, mspace);
            dset.close();
            dset = group.openDataSet("sequence");
            dset.read(&cond.seq, H5::PredType::NATIVE_INT, mspace, mspace);
            dset.close();

            // read string attributes
            dset = group.openDataSet("test_id");
            H5::StrType strdatatype(H5::PredType::C_S1, H5_STRING_LEN);
            cond.test_id.reserve(H5_STRING_LEN); // just in case it isn't allocated
            dset.read(cond.test_id, strdatatype, mspace, mspace);
            dset.close();

            // close out
            mspace.close();
        } 

        group.close();
        file.close();

    } catch (H5::FileIException error) {
        throw(std::invalid_argument("Unable to read wind tunnel condtions from file"));
    } catch (H5::GroupIException error) {
        throw(std::invalid_argument("Unable to read wind tunnel condtions from file"));
    } catch (H5::AttributeIException error) {
        throw(std::invalid_argument("Unable to read wind tunnel condtions from file"));
    } catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Unable to read wind tunnel condtions from file"));
    } catch (H5::DataTypeIException error) {
        throw(std::invalid_argument("Unable to read wind tunnel condtions from file"));
    }

}

/*****************************************************************************/
template<typename CamSettings>
void hdf5_read_camera_settings(std::string filename, CamSettings& cs) {

    // get file version
    unsigned int file_ver = hdf5_get_version(filename);

    try {

    // open file
    H5::H5File file(filename, H5F_ACC_RDONLY);

    // open goup
    H5::Group group;
    if (file_ver == 0) {
        group = file.openGroup("/");
    } else {
        group = file.openGroup("/Condition");
    }

    if (file_ver == 0) {
        
        // read fstop
        H5::Attribute attr = group.openAttribute("fstop");
        attr.read(H5::PredType::NATIVE_FLOAT, &cs.fstop);
        attr.close();
        
        // read exposure if included
        bool has_exposure = hdf5_has_attribute(filename, "Exposure");
        if (has_exposure) {
            attr = group.openAttribute("Exposure");
            attr.read(H5::PredType::NATIVE_FLOAT, &cs.exposure);
            attr.close();
        }

        // read frame rate
        attr = group.openAttribute("Frame Rate");
        attr.read(H5::PredType::NATIVE_INT, &cs.framerate);
        attr.close();

        // check for number of camera focal lengths
        std::vector<std::string> attr_names;
        hdf5_attributes(filename, attr_names);
        
        for (unsigned int i=0; i < attr_names.size(); ++i) {
            if (attr_names[i].find("Focal Length") != std::string::npos) {
                // extract camera number
                std::vector<std::string> terms;
                split_whitespace(attr_names[i], terms);

                int camera_number = -1;
                try {
                    camera_number = stoi(terms[1]);
                } catch(...) {
                    continue;
                }
                cs.cam_nums.push_back(camera_number);
            }
        }
        if (cs.cam_nums.size() > 1) {
            std::sort(cs.cam_nums.begin(), cs.cam_nums.end());
        }
        cs.focal_lengths.resize(cs.cam_nums.size());    

        for (unsigned int i=0; i < cs.focal_lengths.size(); ++i) {
            std::string fl_name = "Cam " + std::to_string(i+1) + " Focal Length";
            attr = group.openAttribute(fl_name);
            attr.read(H5::PredType::NATIVE_FLOAT, &cs.focal_lengths[i]);
            attr.close();
        } 

    } else {

        // prepare to read datasets
        hsize_t mdim[] = {1};
        H5::DataSpace mspace(1, mdim);

        // read frame rate
        H5::DataSet dset = group.openDataSet("frame_rate");
        dset.read(&cs.framerate, H5::PredType::NATIVE_INT, mspace, mspace);
        dset.close();

        // read fstop
        dset = group.openDataSet("fstop");
        dset.read(&cs.fstop, H5::PredType::NATIVE_FLOAT, mspace, mspace);
        dset.close();

        // read exposure
        dset = group.openDataSet("exposure");
        dset.read(&cs.exposure, H5::PredType::NATIVE_FLOAT, mspace, mspace);
        dset.close();

        // read focal lengths
        dset = group.openDataSet("focal_length");
        H5::DataSpace dspace = dset.getSpace();

        int n_dims = dspace.getSimpleExtentNdims();
        assert(n_dims == 1);
        hsize_t dims[n_dims];
        dspace.getSimpleExtentDims(dims, nullptr);

        //hsize_t fldim[] = {dims[0]};

        cs.focal_lengths.resize(dims[0]);
        dset.read(&cs.focal_lengths[0], H5::PredType::NATIVE_FLOAT, dspace, dspace);

        // close out 
        mspace.close();
        dspace.close();
    }

    group.close();
    file.close();

    } catch (H5::FileIException error) {
        throw(std::invalid_argument("Unable to read camera settings from file"));
    } catch (H5::GroupIException error) {
        throw(std::invalid_argument("Unable to read camera settings from file"));
    } catch (H5::AttributeIException error) {
        throw(std::invalid_argument("Unable to read camera settings from file"));
    } catch (H5::DataSetIException error) {
        throw(std::invalid_argument("Unable to read camera settings from file"));
    } catch (H5::DataTypeIException error) {
        throw(std::invalid_argument("Unable to read camera settings from file"));
    }

}

} /* end namespace upsp */

