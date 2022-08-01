/** @file
 *  @brief  Interpolate between grids
 *  @date   September 10, 2019
 *  @author jmpowel2
 */

#ifndef UFML_INTERPOLATION_H_
#define UFML_INTERPOLATION_H_

#include <vector>

namespace upsp {

/** Return True if a node is a data node */
template<typename DataModel>
struct DataNodePredicate {

    typedef typename DataModel::Node model_node;
    typedef typename DataModel::Face model_face;

    bool operator()(const model_node& n) {
        return n.is_datanode();
    }

    bool operator()(const model_face& f) {
        return f.is_dataface();
    }
};

/** Interpolate from one grid onto another
 *  for nodal data
 *  uses an inverse distance approach with k nearest nodes
 * 
 * @param[in] data_grid     model where data is located
 * @param[in] data          model data at each node
 * @param[in] out_grid      grid to map the data onto
 * @param[in] k             number of nodes to use in weighting
 * @param[in] p             exponent on inverse distance weights
 * @return                  data at each grid in @a out_grid
 *
 * @pre data.size() == data_grid.size()
 */
template<typename InModel, typename OutModel>
std::vector<typename OutModel::data_type> interpolate(const InModel& in_model,
        const std::vector<typename InModel::data_type>& data, 
        const OutModel& out_model, unsigned int k, float p = 2.0);

} /* end namespace upsp */

#include "../lib/interpolation.ipp"

#endif /* UFML_INTERPOLATION_H_ */
