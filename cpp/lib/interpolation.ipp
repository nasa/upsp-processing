/** @file
 *  @brief  Interpolate between grids
 *  @date   September 10, 2019
 *  @author jmpowel2
 */

#include <cmath>
#include <opencv2/opencv.hpp>

#include "models.h"

#include "Octree.h"

namespace upsp {

/*****************************************************************************/
template<typename InModel, typename OutModel>
std::vector<typename OutModel::data_type> interpolate(const InModel& in_model,
        const std::vector<typename InModel::data_type>& data,
        const OutModel& out_model, unsigned int k, float p /* = 2.0 */) {

    assert(in_model.size() == data.size());

    typedef typename InModel::data_type FP;
    typedef typename InModel::Node Node;

    // Form an octree around the in_model nodes
    Octree<InModel,Node> tree(&in_model, in_model.cnode_begin(), in_model.cnode_end());

    // Create a predicate for determining nodes in the InModel that
    // have data (are data nodes)
    DataNodePredicate<InModel> data_nodes;
   
    // For each new node, find k nearest nodes and use an inverse distance
    // weighting
    std::vector<FP> out(out_model.size(), 0.0);
    for (unsigned int i=0; i < out_model.size(); ++i) {

        // get the position of this node
        cv::Point3_<FP> pt = out_model.get_position(i);

        // find the nearest nodes
        std::vector<Node> nodes = nearest_k_neighbors(tree, pt, k, data_nodes);

        // use inverse distance weighting to get result
        FP total_weight = 0.0;
        for (unsigned int j=0; j < nodes.size(); ++j) {
            cv::Point3_<FP> pt2 = nodes[j].get_position();
            unsigned int nidx = nodes[j].get_nidx();

            FP dist = cv::norm(pt2 - pt);

            // If the points are identical, just use that data
            if (dist == 0.0) {
                total_weight = 1.0;
                out[i] = data[nidx];
                break;
            }
                
            FP weight = 1.0 / pow(dist, p);

            out[i] += data[nidx]*weight;
        
            total_weight += weight;
        }
        out[i] /= total_weight;
    }

    return out;
}

} /* end namespace upsp */
