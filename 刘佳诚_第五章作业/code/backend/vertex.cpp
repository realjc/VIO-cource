#include "vertex.h"

unsigned long global_vertex_id = 0;

Vertex::Vertex(int num_dimension, int local_dimension){
    parameters_.resize(num_dimension,1);
    local_demension_ = local_dimension>0? local_dimension:num_dimension;
    id_ = global_vertex_id++;//全局参数，确保定位vertex参数
}

Vertex::~Vertex(){}

int Vertex::Dimension() const{
    return parameters_.rows();
}

int Vertex::LocalDimension() const{
    return local_dimension_;
}

void Vertex::Plus(const VecX&delta){
    parameters_ += delta;
}

void VertexPose::Plus(const VecX&delta){
    VecX &parameters = Parameters();
    parameters.head<3>() += delta.head<3>();
    Qd q(parameters[6],parameters[3],parameters[4],parameters[5]);
    q  = q*Sophus::SO3d::exp(Vec3(delta[3],delta[4],delta[5])).unit_quaternion();
    q.normalized();
    parameters[3] = q.x(); //存储顺序为x,y,z，w
    parameters[4] = q.y();
    parameters[5] = q.z();
    parameters[6] = q.w();

}