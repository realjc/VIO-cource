#include <iostream>
#include <random>
#include "../backend/problem.h"

using namespace myslam::backend;
using namespace std;

// vectex
class CurveFittingVertex: public Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 3 parameters to be estimated (a,b,c)
    // dimension of vectex is 3
    CurveFittingVertex(): Vertex(3) {}
    virtual std::string TypeInfo() const { return "abc"; }
};

// edge
class CurveFittingEdge: public Edge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // input parameter: measurement
    // construction parameter: order of residual dimension, number of vertex, name of vertex
    CurveFittingEdge( double x, double y ): Edge(1,1, std::vector<std::string>{"abc"}) {
        x_ = x;
        y_ = y;
    }
    // compute the ressidual
    virtual void ComputeResidual() override
    {
        // extract estimated parameter from vectex
        Vec3 abc = verticies_[0]->Parameters();
        // build residual function
        residual_(0) = std::exp( abc(0)*x_*x_ + abc(1)*x_ + abc(2) ) - y_;
        // residual_(0) = abc(0)*x_*x_ + abc(1)*x_ + abc(2) - y_;

    }

    // compute Jacobian of residual function
    virtual void ComputeJacobians() override
    {
        Vec3 abc = verticies_[0]->Parameters();
        double exp_y = std::exp( abc(0)*x_*x_ + abc(1)*x_ + abc(2) );

        // dim of residual function is 1, dim of state variable is 3
        // the dim of Jacobian is 1*3
        Eigen::Matrix<double, 1, 3> jaco_abc;
        jaco_abc << x_ * x_ * exp_y, x_ * exp_y , 1 * exp_y;
        // jaco_abc << x_ * x_, x_, 1;
        jacobians_[0] = jaco_abc;
    }

    virtual std::string TypeInfo() const override { return "CurveFittingEdge"; }
public:

    //measurement
    double x_,y_;
};

int main()
{
    // true parameter
    double a=1.0, b=2.0, c=1.0;
    // number of data
    int N = 100;
    // variance of data
    double w_sigma= 1.;          
    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.,w_sigma);

    // build problem
    Problem problem(Problem::ProblemType::GENERIC_PROBLEM);
    shared_ptr< CurveFittingVertex > vertex(new CurveFittingVertex());

    // initial value of to be estimated parameters
    vertex->SetParameters(Eigen::Vector3d (0.,0.,0.));
    // add vectex
    problem.AddVertex(vertex);

    // totally N measuremnet
    for (int i = 0; i < N; ++i) {

        double x = i/100.;
        double n = noise(generator);

        // measurement
        double y = std::exp( a*x*x + b*x + c ) + n;
        // double y = a*x*x + b*x + c + n;

        // for each measurement, build an edge
        shared_ptr< CurveFittingEdge > edge(new CurveFittingEdge(x,y));
        std::vector<std::shared_ptr<Vertex>> edge_vertex;
        edge_vertex.push_back(vertex);
        edge->SetVertex(edge_vertex);

        // add edge
        problem.AddEdge(edge);
    }

    std::cout<<"\nTest CurveFitting start..."<<std::endl;
    /// use LM to optimize
    problem.Solve(30);

    std::cout << "-------After optimization, we got these parameters :" << std::endl;
    std::cout << vertex->Parameters().transpose() << std::endl;
    std::cout << "-------ground truth: " << std::endl;
    std::cout << "1.0,  2.0,  1.0" << std::endl;

    return 0;
}


