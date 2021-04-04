class Problem{
public:
    enum class ProblemType{
        SLAM_PROBLEM,
        GENERIC_PROBLEM
    };
    typedef std::map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
    Problem(ProblemType problemTYpe);

    void LogoutVectorSize();

private:
    ProblemType problemType_;
    HashVertex verticies_marg_;// id-vertex map
};