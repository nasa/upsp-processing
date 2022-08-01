
/*****************************************************************************/
template<typename Model>
void create_data(const Model& model, bool nodal, unsigned int n_frames, 
        std::vector<float>& sol) {

    // Get sizing
    unsigned int n_pts = 0;
    if (nodal) {
        n_pts = model.size();
    } else {
        n_pts = model.number_of_faces();
    }

    // Allocate vectors
    sol.clear();
    sol.resize(n_pts*n_frames);

    // Assign values for the first frame
    if (nodal) {
        for (unsigned int n=0; n < n_pts; ++n) {
            cv::Point3f pos = model.get_position(n);
            sol[n] = 0.1 * pos.x + pos.y + pos.z;
        }
    } else {
        unsigned int count = 0;
        for (auto f=model.cface_begin(); f != model.cface_end(); ++f) {
            cv::Point3f pos = (*f).get_center();
            sol[count] = 0.1 * pos.x + pos.y + pos.z;
            ++count;
        }
    }

    // Assign values for all other frames
    for (unsigned int f=1; f < n_frames; ++f) {
        std::copy(sol.begin(), sol.begin()+n_pts, sol.begin()+f*n_pts);
        for (unsigned int n=0; n < n_pts; ++n) {
            sol[n+f*n_pts] += f;
        }
    }

}

