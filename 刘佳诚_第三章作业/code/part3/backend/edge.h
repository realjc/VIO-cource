#ifndef MYSLAM_BACKEND_EDGE_H
#define MYSLAM_BACKEND_EDGE_H

#include <memory>
#include <string>
#include "eigen_types.h"

namespace myslam {
namespace backend {

class Vertex;

/**
 * edge is used to compute residual
 * residual: prediction - measurement
 */

class Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * constructor, can allocate dimension of Jacobian matrix
     * @param residual_dimension
     * @param num_verticies
     * @param verticies_types   name of vertex type (optional)
     */
    explicit Edge(int residual_dimension, int num_verticies,
                  const std::vector<std::string> &verticies_types = std::vector<std::string>());

    virtual ~Edge();

    /// return id
    unsigned long Id() const { return id_; }

    /**
     * add one vectex
     * @param vertex vectex to be added
     */
    bool AddVertex(std::shared_ptr<Vertex> vertex) {
        verticies_.emplace_back(vertex);
        return true;
    }

    /**
     * add several verticies
     * @param vertices verticies to be added
     * @return
     */
    bool SetVertex(const std::vector<std::shared_ptr<Vertex>> &vertices) {
        verticies_ = vertices;
        return true;
    }

    /// return i-th vertex
    std::shared_ptr<Vertex> GetVertex(int i) {
        return verticies_[i];
    }

    /// return all verticis
    std::vector<std::shared_ptr<Vertex>> Verticies() const {
        return verticies_;
    }

    /// return number of verticies
    size_t NumVertices() const { return verticies_.size(); }

    /// return type name of edge
    /// has to be overrided in driven class
    virtual std::string TypeInfo() const = 0;

    /// compute residual
    virtual void ComputeResidual() = 0;

    /// compute Jacobian matrix
    virtual void ComputeJacobians() = 0;

//    /// compute Hession Matrix
//    virtual void ComputeHessionFactor() = 0;

    /// compute squre error, here the Information Matrix is used
    double Chi2();

    /// return residual
    VecX Residual() const { return residual_; }

    /// return Jacobian matrix
    std::vector<MatXX> Jacobians() const { return jacobians_; }

    /// set Information matrix, default value is identity matrix
    void SetInformation(const MatXX &information) {
        information_ = information;
    }

    /// return Information matrix
    MatXX Information() const {
        return information_;
    }

    /// set measurement
    void SetObservation(const VecX &observation) {
        observation_ = observation;
    }

    /// return measurement
    VecX Observation() const { return observation_; }

    /// check if all edge information is set
    bool CheckValid();

    int OrderingId() const { return ordering_id_; }

    void SetOrderingId(int id) { ordering_id_ = id; };

protected:
    unsigned long id_;  // edge id
    int ordering_id_;   //edge id in problem
    std::vector<std::string> verticies_types_; 
    std::vector<std::shared_ptr<Vertex>> verticies_; 
    VecX residual_;                 // residual
    std::vector<MatXX> jacobians_;  // Jacobian matrix. The dim of each Jacobian is residual x vertex[i]
    MatXX information_;             // information matrix
    VecX observation_;              // measurement
};

}
}

#endif
