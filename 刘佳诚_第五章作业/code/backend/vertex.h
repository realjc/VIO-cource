
class Vertex{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    explicit Vertex(int num_dimension, int local_dimension = -1);
    virtual ～Vertex();// 基函数设置为虚函数，可以避免内存泄露

};