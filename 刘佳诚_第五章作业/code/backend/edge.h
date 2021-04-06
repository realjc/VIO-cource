

class Vertex;

class Edge{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    explicit Edge(int residual_dimension, int num_vertices,
        const vector<string> &verticies_types = vector<string>());
    
    virtual ~Edge();
    unsigned long Id() const {return id_;}

    bool AddVertex(shared_ptr<Vertex> vertex){
        verticies_.emplace_back(vertex);
        return true;
    }

    bool SetVertex(vector<shared_ptr<Vertex>> vertices){
        verticies_ = vertices;
        return true;
    }

    shared_ptr<Vertex> GetVertices(int i){
        return vretices_[i];
    }

    virtual void ComputeResidual() = 0;

    virtual void ComputeJacobians() = 0;


protected:
    unsigned long id_;
    int ordering_id_;
    vector<string> verticies_types_;
    vector<shared_ptr<Vertex>> verticies_;
    VecX residual_;
    vector<MatXX> jacobians_;
    MatXX information_;
    VecX observation_;

};


class EdgeReProjection: public Edge{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeReProjection(const Vec3&pts_i, const Vec3 &pts_j): Edge(2, 3, 
        std::vector<std::string>{"VertexInverseDepth", "VertexPose", "VertexPose"})
    {
        pts_i_ = pts_i;
        pts_j_ = pts_j;
    }

    virtual void ComputeResidual() override;
    virtual void ComputeJacobians() override;
    void SetTranslationImuFromCamera(Eigen::Quaterniond&qic_, Vec3 &tic_);

private:
    Qd qic;
    Vec3 tic;
    Vec3 pts_i_, pts_j_;
    
};

class EdgeSE3Prior : public Edge{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE3Prior(const Vec3&p, const Qd&q) :Edge(6,1,vector<string>{"VertexPose"}),
        Pp_(p),Qp_(q){}

    virtual string TypeInfo() const override {return "EdgeSE3Prior";}

    virtual void ComputeResidual() override;
    virtual void ComputeJacobians() override;
private:
    Vec3 Pp_;
    Qd Qp_;
};