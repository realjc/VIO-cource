#ifndef MYSLAM_BACKEND_VERTEX_H
#define MYSLAM_BACKEND_VERTEX_H

#include "eigen_types.h"

namespace myslam {
namespace backend {

/**
 * @brief vertex (parameter block)
 * vertex is stored as VecX, the dimension has to be assigned when constructed
 */
class Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * constructor
     * @param num_dimension vertex dimension
     * @param local_dimension local dimension, is -1 when it is same as vertex dimension
     * eg. dim of quanternion is 4, but its loacl_dimension is 3
     */
    explicit Vertex(int num_dimension, int local_dimension = -1);

    virtual ~Vertex();

    /// return variable dimension
    int Dimension() const;

    /// return variable local dimension
    int LocalDimension() const;

    /// return vertex id
    unsigned long Id() const { return id_; }

    /// return estimated parameter stored in vertex
    VecX Parameters() const { return parameters_; }

    /// return reference of parameter
    VecX &Parameters() { return parameters_; }

    /// set parameter
    void SetParameters(const VecX &params) { parameters_ = params; }

    /// addition
    /// default is vector additiion
    virtual void Plus(const VecX &delta);

    // return type name of vertex
    virtual std::string TypeInfo() const = 0;

    int OrderingId() const { return ordering_id_; }

    void SetOrderingId(unsigned long id) { ordering_id_ = id; };

    /// fix the vertex, it won't be optimized
    // the coresponding part in Hessian is 0
    void SetFixed(bool fixed = true) {
        fixed_ = fixed;
    }

    /// check is fixed
    bool IsFixed() const { return fixed_; }

protected:
    VecX parameters_;   // parameters that to be estimated
    int local_dimension_;   
    unsigned long id_;  // vectex id

    // ordering id is the arranged id in problem, it is used to find coresponding Jacobian block
    // eg. ordering_id = 6 coresponds to the 6th column in Hessian
    unsigned long ordering_id_ = 0;

    bool fixed_ = false;
};

}
}

#endif