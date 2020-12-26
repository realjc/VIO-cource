#ifndef MYSLAM_BACKEND_PROBLEM_H
#define MYSLAM_BACKEND_PROBLEM_H

#include <unordered_map>
#include <map>
#include <memory>

#include "eigen_types.h"
#include "edge.h"
#include "vertex.h"

typedef unsigned long ulong;

namespace myslam {
namespace backend {

class Problem {
public:

    /**
     * type of problem
     * 
     * if it is SLAM problem, pose and landmark are sepreated, Hessian is sparse
     * if it is normal problem, the Hessian is dense. Or user can set some verticies as marginalized sepecifically
     */
    enum class ProblemType {
        SLAM_PROBLEM,
        GENERIC_PROBLEM
    };
    typedef unsigned long ulong;
//    typedef std::unordered_map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
    typedef std::map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
    typedef std::unordered_map<unsigned long, std::shared_ptr<Edge>> HashEdge;
    typedef std::unordered_multimap<unsigned long, std::shared_ptr<Edge>> HashVertexIdToEdge;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Problem(ProblemType problemType);

    ~Problem();

    bool AddVertex(std::shared_ptr<Vertex> vertex);

    /**
     * remove a vertex
     * @param vertex_to_remove
     */
    bool RemoveVertex(std::shared_ptr<Vertex> vertex);

    bool AddEdge(std::shared_ptr<Edge> edge);

    bool RemoveEdge(std::shared_ptr<Edge> edge);

    /**
     * get outlier edges in the optimization process, so that the front-end can remove these measurement
     * @param outlier_edges
     */
    void GetOutlierEdges(std::vector<std::shared_ptr<Edge>> &outlier_edges);

    /**
     * solve the problem iteratively
     * @param iterations
     * @return
     */
    bool Solve(int iterations);

    /// marginalize a frame and its landmark
    bool Marginalize(std::shared_ptr<Vertex> frameVertex,
                     const std::vector<std::shared_ptr<Vertex>> &landmarkVerticies);

    bool Marginalize(const std::shared_ptr<Vertex> frameVertex);

    //test compute prior
    void TestComputePrior();

private:

    /// solve normal problem
    bool SolveGenericProblem(int iterations);

    /// solve SLAM problem
    bool SolveSLAMProblem(int iterations);

    /// set ordering_index of all verticies
    void SetOrdering();

    /// set ordering for new vertex in slam problem
    void AddOrderingSLAM(std::shared_ptr<Vertex> v);

    /// build Hessian matrix
    void MakeHessian();

    /// use schur to solve sparse BA problem
    void SchurSBA();

    /// solver linear problem
    void SolveLinearSystem();

    /// update state variable
    void UpdateStates();

    // sometimes the residual become too big after update, reject this update
    void RollbackStates();

    /// compute and update prior
    void ComputePrior();

    /// check if a vertex is Pose vertex
    bool IsPoseVertex(std::shared_ptr<Vertex> v);

    /// check if a vertex is Landmark vertex
    bool IsLandmarkVertex(std::shared_ptr<Vertex> v);

    /// after adding a vertex, the size of Hessian matrix need to be adjusted
    void ResizePoseHessiansWhenAddingPose(std::shared_ptr<Vertex> v);

    /// check if ordering is correct
    bool CheckOrdering();

    void LogoutVectorSize();

    /// return the connected edges the vertex
    std::vector<std::shared_ptr<Edge>> GetConnectedEdges(std::shared_ptr<Vertex> vertex);

    /// Levenberg
    /// compute initial Lambda of LM algorithom
    void ComputeLambdaInitLM();

    /// add or remove Lambda in Hessian diagonal elements
    void AddLambdatoHessianLM();

    void RemoveLambdaHessianLM();

    /// check if Lambda is appropriate for last iteration
    bool IsGoodStepInLM();

    /// PCG iterative linear solver
    VecX PCGSolver(const MatXX &A, const VecX &b, int maxIter);

    double currentLambda_;
    double currentChi_;
    double stopThresholdLM_;    // LM loop break threshold
    double ni_;                 // Lambda scale parameter

    ProblemType problemType_;

    /// the whole information matrix
    MatXX Hessian_;
    VecX b_;
    VecX delta_x_;

    /// prior part
    MatXX H_prior_;
    VecX b_prior_;
    MatXX Jt_prior_inv_;
    VecX err_prior_;

    /// pose part of sparse BA
    MatXX H_pp_schur_;
    VecX b_pp_schur_;

    // landmark part and pose part
    MatXX H_pp_;
    VecX b_pp_;
    MatXX H_ll_;
    VecX b_ll_;

    /// all vertices
    HashVertex verticies_;

    /// all edges
    HashEdge edges_;

    /// use vertex id to query edge
    HashVertexIdToEdge vertexToEdge_;

    /// Ordering related
    ulong ordering_poses_ = 0;
    ulong ordering_landmarks_ = 0;
    ulong ordering_generic_ = 0;

    /// use ordering to arrange pose verticies
    std::map<unsigned long, std::shared_ptr<Vertex>> idx_pose_vertices_;
    /// use ordering to arrange landmark verticies
    std::map<unsigned long, std::shared_ptr<Vertex>> idx_landmark_vertices_;

    // verticies need to marg. <Ordering_id_, Vertex>
    HashVertex verticies_marg_;

    bool bDebug = false;
    double t_hessian_cost_ = 0.0;
    double t_PCGsovle_cost_ = 0.0;
};

}
}
#endif
