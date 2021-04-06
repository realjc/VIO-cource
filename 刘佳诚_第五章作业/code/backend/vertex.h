
#include "vertex.h"
#include "bits/stdc++.h"

using namespace std;

class Vertex{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    explicit Vertex(int num_dimension, int local_dimension = -1);
    virtual ~Vertex();// 基函数设置为虚函数，可以避免内存泄露

};

class VertexPose: public Vertex{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPose() : Vertex(7,6){}

    virtual void Plus(const VecX&delta) override;
    string TypeInfo() const{
        return "VertexPose";
    }
};


class VertexInverseDepth:public:Vertex{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexInverseDepth() :Vertex(1){}
    virtual std::string TypeInfo() const { return "VertexInverseDepth"; }
};