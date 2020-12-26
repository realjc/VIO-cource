#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <glog/logging.h>
#include "problem.h"
#include "../utils/tic_toc.h"
#include <fstream>

#ifdef USE_OPENMP

#include <omp.h>

#endif

using namespace std;


namespace myslam {
namespace backend {
void Problem::LogoutVectorSize() {
    // LOG(INFO) <<
    //           "1 problem::LogoutVectorSize verticies_:" << verticies_.size() <<
    //           " edges:" << edges_.size();
}

Problem::Problem(ProblemType problemType) :
        problemType_(problemType) {
    LogoutVectorSize();
    verticies_marg_.clear();
}

Problem::~Problem() {}

bool Problem::AddVertex(std::shared_ptr<Vertex> vertex) {
    if (verticies_.find(vertex->Id()) != verticies_.end()) {
        // LOG(WARNING) << "Vertex " << vertex->Id() << " has been added before";
        return false;
    } else {
        verticies_.insert(pair<unsigned long, shared_ptr<Vertex>>(vertex->Id(), vertex));
    }

    return true;
}



bool Problem::AddEdge(shared_ptr<Edge> edge) {
    if (edges_.find(edge->Id()) == edges_.end()) {
        edges_.insert(pair<ulong, std::shared_ptr<Edge>>(edge->Id(), edge));
    } else {
        // LOG(WARNING) << "Edge " << edge->Id() << " has been added before!";
        return false;
    }

    for (auto &vertex: edge->Verticies()) {
        vertexToEdge_.insert(pair<ulong, shared_ptr<Edge>>(vertex->Id(), edge));
    }
    return true;
}


bool Problem::Solve(int iterations) {


    if (edges_.size() == 0 || verticies_.size() == 0) {
        std::cerr << "\nCannot solve problem without edges or verticies" << std::endl;
        return false;
    }

    TicToc t_solve;
    // compute the dimension of estimated variable, prepare for building H matrix
    SetOrdering();

    // traverse edges, build H = J^T * J
    MakeHessian();

    // LM initialization
    ComputeLambdaInitLM();

    // LM iteration
    bool stop = false;
    int iter = 0;
    std::vector<double> lambda_set; 

    while (!stop && (iter < iterations)) {
        std::cout << "iter: " << iter << " , chi= " << currentChi_ << " , Lambda= " << currentLambda_
                  << std::endl;
        
        lambda_set.push_back(currentLambda_);

        bool oneStepSuccess = false;
        int false_cnt = 0;

        // keep trying different Lambda, until one is successful
        while (!oneStepSuccess) 
        {
            // setLambda
            AddLambdatoHessianLM();
            // solve linear problem: H X = B
            SolveLinearSystem();
            RemoveLambdaHessianLM();

            // break condition 1： delta_x_ is small enough
            if (delta_x_.squaredNorm() <= 1e-6 || false_cnt > 10) {
                stop = true;
                break;
            }

            // update state variable: X = X+ delta_x
            UpdateStates();

            // check if current step is ok
            oneStepSuccess = IsGoodStepInLM();

            if (oneStepSuccess) {
                // construct Hessian matrix at linerized point
                MakeHessian();
                false_cnt = 0;
            } else {
                false_cnt++;
                // error doesn't goes down, rollback
                RollbackStates();   // 误差没下降，回滚
            }
        }
        iter++;

        // break condition 2： currentChi is less than 1e-6 times of last chi2
        if (sqrt(currentChi_) <= stopThresholdLM_)
            stop = true;
    }
    std::cout << "problem solve cost: " << t_solve.toc() << " ms" << std::endl;
    std::cout << "   makeHessian cost: " << t_hessian_cost_ << " ms" << std::endl;


    // output lambda date
    std::ofstream outfile("lambda_set2.txt");
    for(uint i=0; i<lambda_set.size(); i++)
        outfile << i << " " << lambda_set[i] << endl;
    outfile.close();


    return true;
}


void Problem::SetOrdering() {

    // count from begining
    ordering_poses_ = 0;
    ordering_generic_ = 0;
    ordering_landmarks_ = 0;

    // Note:: type of verticies_  is map, the order is according to id
    // compute total dimenstino of to be estimated variables
    for (auto vertex: verticies_) {
        ordering_generic_ += vertex.second->LocalDimension();
    }
}

void Problem::MakeHessian() {
    TicToc t_h;
    // construct large H matrix
    ulong size = ordering_generic_;
    MatXX H(MatXX::Zero(size, size));
    VecX b(VecX::Zero(size));

    // traverse all edges, compute their Jacobian, construct H = J^T * J
    for (auto &edge: edges_) {

        edge.second->ComputeResidual();
        edge.second->ComputeJacobians();

        auto jacobians = edge.second->Jacobians();
        auto verticies = edge.second->Verticies();
        assert(jacobians.size() == verticies.size());
        for (size_t i = 0; i < verticies.size(); ++i) {
            auto v_i = verticies[i];

            // H doesn't need this Jacobian (set it to 0)
            if (v_i->IsFixed()) continue;

            auto jacobian_i = jacobians[i];
            ulong index_i = v_i->OrderingId();
            ulong dim_i = v_i->LocalDimension();

            MatXX JtW = jacobian_i.transpose() * edge.second->Information();
            for (size_t j = i; j < verticies.size(); ++j) {
                auto v_j = verticies[j];

                if (v_j->IsFixed()) continue;

                auto jacobian_j = jacobians[j];
                ulong index_j = v_j->OrderingId();
                ulong dim_j = v_j->LocalDimension();

                assert(v_j->OrderingId() != -1);
                MatXX hessian = JtW * jacobian_j;
                
                // add all these Hessian
                H.block(index_i, index_j, dim_i, dim_j).noalias() += hessian;
                if (j != i) {
                    // H is symmetric
                    H.block(index_j, index_i, dim_j, dim_i).noalias() += hessian.transpose();
                }
            }
            b.segment(index_i, dim_i).noalias() -= JtW * edge.second->Residual();
        }

    }
    Hessian_ = H;
    b_ = b;
    t_hessian_cost_ += t_h.toc();

    delta_x_ = VecX::Zero(size);  // initial delta_x = 0_n;

}

/*
* Solve Hx = b, we can use PCG iterative method or use sparse Cholesky
*/
void Problem::SolveLinearSystem() {

        delta_x_ = Hessian_.inverse() * b_;
//        delta_x_ = H.ldlt().solve(b_);

}

void Problem::UpdateStates() {
    for (auto vertex: verticies_) {
        ulong idx = vertex.second->OrderingId();
        ulong dim = vertex.second->LocalDimension();
        VecX delta = delta_x_.segment(idx, dim);

        // x_{k+1} = x_{k} + delta_x
        vertex.second->Plus(delta);
    }
}

void Problem::RollbackStates() {
    for (auto vertex: verticies_) {
        ulong idx = vertex.second->OrderingId();
        ulong dim = vertex.second->LocalDimension();
        VecX delta = delta_x_.segment(idx, dim);

        // after the update, the error goes up, so we reject this iteration.
        // rollback to last iteration, add the substracted delta
        vertex.second->Plus(-delta);
    }
}

/// LM
void Problem::ComputeLambdaInitLM() {
    ni_ = 2.;
    currentLambda_ = -1.;
    currentChi_ = 0.0;
    // TODO:: robust cost chi2
    for (auto edge: edges_) {
        currentChi_ += edge.second->Chi2();
    }
    if (err_prior_.rows() > 0)
        currentChi_ += err_prior_.norm();

    // break condition, error doesn't change much, 1e-6
    stopThresholdLM_ = 1e-6 * currentChi_;          

    double maxDiagonal = 0;
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    for (ulong i = 0; i < size; ++i) {
        maxDiagonal = std::max(fabs(Hessian_(i, i)), maxDiagonal);
    }
    double tau = 1e-5;
    currentLambda_ = tau * maxDiagonal;
}

void Problem::AddLambdatoHessianLM() {
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    for (ulong i = 0; i < size; ++i) {
        Hessian_(i, i) += currentLambda_;
        //Hessian_(i, i) += currentLambda_ * Hessian_(i, i);
    }
}

void Problem::RemoveLambdaHessianLM() {
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    // TODO:: 这里不应该减去一个，数值的反复加减容易造成数值精度出问题？而应该保存叠加lambda前的值，在这里直接赋值
    for (ulong i = 0; i < size; ++i) {
        Hessian_(i, i) -= currentLambda_;
         //Hessian_(i, i) /= 1.0 + currentLambda_;
    }
}

bool Problem::IsGoodStepInLM() {

    // recompute residuals after update state
    double tempChi = 0.0;
    for (auto edge: edges_) {
        edge.second->ComputeResidual();
        tempChi += edge.second->Chi2();
    }

    double scale = 0;
    //scale = delta_x_.transpose() * (currentLambda_ * delta_x_ + b_);

    // scale = delta_x_.transpose() * (currentLambda_ * Hessian_.diagonal()*delta_x_ + b_);



    Eigen::MatrixXd JWfh(1,1);
    JWfh = b_.transpose()*delta_x_;
    double d_JWfh = JWfh(0,0);
    double alpha = d_JWfh /((currentChi_ - tempChi)/2+2*d_JWfh);

    delta_x_  *= alpha;
    scale = (alpha*delta_x_).transpose()*(currentLambda_*delta_x_*alpha+b_);

    scale += 1e-3;    // make sure it's non-zero :)

    RollbackStates();
    UpdateStates();
    
    tempChi = 0.0;
    for (auto edge: edges_) {
        edge.second->ComputeResidual();
        tempChi += edge.second->Chi2();
    }

    double rho = (currentChi_ - tempChi) / scale;

    // if (rho > 0 && isfinite(tempChi))   // last step was good, error goes down
    // {
    //     double alpha = 1. - pow((2 * rho - 1), 3);
    //     alpha = std::min(alpha, 2. / 3.);
    //     double scaleFactor = (std::max)(1. / 3., alpha);
    //     currentLambda_ *= scaleFactor;
    //     ni_ = 2;
    //     currentChi_ = tempChi;
    //     return true;
    // } else {
    //     currentLambda_ *= ni_;
    //     ni_ *= 2;
    //     return false;
    // }


    // if (rho > 0.0 && isfinite(tempChi))   // last step was good, error goes down
    // {
    //     if (rho < 0.25) {
    //         currentLambda_ *= 2;
    //     }
    //     else if (rho > 0.75) {
    //         currentLambda_ /= 3;
    //     }

    //     currentChi_ = tempChi;
    //     return true;
    // } else {
    //     currentLambda_ *= ni_;
    //     ni_ *= 2;
    //     return false;
    // }

    // if (rho > 0.0 && isfinite(tempChi))   // last step was good, error goes down
    // {
    //     currentLambda_ = std::max(currentLambda_/9., 1e-7);
    //     currentChi_ = tempChi;
    //     return true;
    // } else {
    //     currentLambda_ = std::min(currentLambda_*11., 1e7);
    //     return false;
    // }

    if (rho > 0.0 && isfinite(tempChi))
    {
        currentLambda_ = std::max(currentLambda_/(1+alpha), 1e-7);
        currentChi_ = tempChi;
        return true;
    } else {
        currentLambda_ += std::abs(tempChi - currentChi_)/(2*alpha);
        RollbackStates();
        return false;
    }

}

/** @brief conjugate gradient with perconditioning
*
*  the jacobi PCG method
*
*/
VecX Problem::PCGSolver(const MatXX &A, const VecX &b, int maxIter = -1) {
    assert(A.rows() == A.cols() && "PCG solver ERROR: A is not a square matrix");
    int rows = b.rows();
    int n = maxIter < 0 ? rows : maxIter;
    VecX x(VecX::Zero(rows));
    MatXX M_inv = A.diagonal().asDiagonal().inverse();
    VecX r0(b);  // initial r = b - A*0 = b
    VecX z0 = M_inv * r0;
    VecX p(z0);
    VecX w = A * p;
    double r0z0 = r0.dot(z0);
    double alpha = r0z0 / p.dot(w);
    VecX r1 = r0 - alpha * w;
    int i = 0;
    double threshold = 1e-6 * r0.norm();
    while (r1.norm() > threshold && i < n) {
        i++;
        VecX z1 = M_inv * r1;
        double r1z1 = r1.dot(z1);
        double belta = r1z1 / r0z0;
        z0 = z1;
        r0z0 = r1z1;
        r0 = r1;
        p = belta * p + z1;
        w = A * p;
        alpha = r1z1 / p.dot(w);
        x += alpha * p;
        r1 -= alpha * w;
    }
    return x;
}

    }
}






