/** @file
 *  @brief  Model Abstract Base Class
 *  @date   March 20, 2019
 *  @author jmpowel2
 */

#ifndef UFML_MODEL_H_
#define UFML_MODEL_H_

namespace upsp {

/** Model Abstract Base Class
 *   more complex version of the Grid Abstract Base Struct
 *   includes tools for easier manipulation of the underlying grid
 */
template<typename FP>
class Model {
public:

    typedef unsigned int node_idx;
    typedef unsigned int face_idx;

    Model() : use_nodes_(true) {}

    /** Copy constructor */
    Model(const Model<FP>& m_in) : use_nodes_(m_in.use_nodes_), 
            data_nodes_(m_in.data_nodes_), data_faces_(m_in.data_faces_) {}

    /** Return the number of nodes in the model */
    virtual unsigned int size() const = 0;

    /** Return the number of faces in the model */
    virtual unsigned int number_of_faces() const = 0;

    /** Switch to face mode */
    void use_nondata_faces() { use_nodes_ = false; }

    /** Switch to node mode */
    void use_nondata_nodes() { use_nodes_ = true; }

    /** Check mode */
    bool using_nodes() const { return use_nodes_; }

    /** Mark node as non-data node
     *
     * @param[in] nidx      node index
     *
     * @pre nidx < size()
     */
    void set_node_nondata(node_idx nidx) { 
        if (data_nodes_.size() < size()) {
            data_nodes_.resize(size(), true);
        }
        data_nodes_[nidx] = false;
    }

    /** Check if a node is a data node
     *
     * @param[in] nidx      node index
     *
     * @pre nidx < size()
     */
    bool is_datanode(node_idx nidx) const { 
        if (nidx >= data_nodes_.size()) {
            return true;
        }
        return data_nodes_[nidx];
    }

    /** Mark face as a non-data node
     *
     * @param[in] fidx      face index
     *
     * @pre fidx < number_of_faces()
     * @pre using_nodes() == false
     */
    void set_face_nondata(face_idx fidx) {
        if (data_faces_.size() < number_of_faces()) {
            data_faces_.resize(number_of_faces(), true);
        }
        data_faces_[fidx] = false;
    }

    /** Check if a face is a data face
     *
     * @param[in] fidx  face index
     *
     * @pre fidx < number_of_faces()
     */
    bool is_dataface(face_idx fidx) const {
        if (fidx >= data_faces_.size()) {
            return true;
        }
        return data_faces_[fidx];
    }

    /** Return a reference to the vector of node x locations */
    virtual const std::vector<FP>& get_x() const = 0;
    /** Return a reference to the vector of node y locations */
    virtual const std::vector<FP>& get_y() const = 0;
    /** Return a reference to the vector of node z locations */
    virtual const std::vector<FP>& get_z() const = 0;

    /** Adjust solution for any missing data */
    virtual void adjust_solution(std::vector<FP>& sol) const {}
    
private:
    
    bool use_nodes_;

    // if true, node can hold data
    // if false, node is just there to complete the surface (watertight)
    std::vector<bool> data_nodes_;

    // if true, face can hold data
    // if false, face is just there to complete the surface (watertight)
    std::vector<bool> data_faces_;

};

} /* end namespace upsp */

#endif /* UFML_MODEL_H_ */
