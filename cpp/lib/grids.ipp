/** @file
 *  @brief  Grid Data Structure Definitions
 *  @date   Nov 15, 2018
 *  @author jmpowel2
 */

namespace upsp {

/*****************************************************************************/
template<typename T>
unsigned int StructuredGrid<T>::zone_size(unsigned int zone) const {
    if (zone >= grid_size.size()) {
        throw(std::invalid_argument("Exceeded number of zones"));
    }
    unsigned int prod = 1;
    for (unsigned int j=0; j < grid_size[zone].size(); ++j) {
        prod *= grid_size[zone][j];
    }
    return prod;
}

/*****************************************************************************/
template<typename T>
unsigned int StructuredGrid<T>::zone_size(unsigned int zone, unsigned int idx) const {
    if (zone >= grid_size.size()) {
        throw(std::invalid_argument("Exceeded number of zones"));
    }
    if (grid_size[zone].size() <= idx) {
        return 0;
    } else {
        return grid_size[zone][idx];
    }
}

/*****************************************************************************/
template<typename T>
unsigned int StructuredGrid<T>::zone_start_idx(unsigned int zone) const {
    if (zone >= grid_size.size()) {
        throw(std::invalid_argument("Exceeded number of zones"));
    }
    unsigned int idx=0;
    for (unsigned int i=0; i < zone; ++i) {
        idx += zone_size(i);
    }
    return idx;
}   

/*****************************************************************************/
template<typename T>
unsigned int RefStructuredGrid<T>::zone_size(unsigned int zone) const {
    if (zone >= grid_size.size()) {
        throw(std::invalid_argument("Exceeded number of zones"));
    }
    unsigned int prod = 1;
    for (unsigned int j=0; j < grid_size[zone].size(); ++j) {
        prod *= grid_size[zone][j];
    }
    return prod;
}

/*****************************************************************************/
template<typename T>
void UnstructuredGrid<T>::sort_by_components() {

    assert(comps.size() == tris.size());    

    // sort by first gathering the ordered indices
    std::vector<size_t> idx(comps.size());
    std::iota(idx.begin(), idx.end(), 0);
    auto& ref_comps = comps;
    std::sort(idx.begin(), idx.end(), [&ref_comps](size_t i1, size_t i2) {
            return ref_comps[i1] < ref_comps[i2];});
    
    // copy over entire vectors and then update with indices
    // not ideal memory-wise, but it shouldn't be too bad
    // there are other workarounds with boost and tuples if it becomes an issue
    // but it is non-trivial
    std::vector<int> tmp_comps(comps);
    std::vector<simple_tri> tmp_tris(tris);
    n_comps = 1;
    for (unsigned int i=0; i < idx.size(); ++i) {
        comps[i] = tmp_comps[idx[i]];
        tris[i] = tmp_tris[idx[i]];
        if (i > 0) {
            if ( comps[i] != comps[i-1]) {
                ++n_comps;
            }
        }
    }

    has_n_comps = true;
}

} /* end namespace upsp */

