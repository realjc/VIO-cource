
struct Frame{
    //这里的Rwc是以世界坐标系作为参考
    Frame(Eigen::Matrix3d R, Eigen::Vector3d t):Rwc(R),qwc(R),twc(t){}
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniod qwc; // 四元数形式
    Eigen::Vector3d twc;

    unordered_map<int,Eigen::Vector3d> featurePerId;
    // 观测到的特征点在归一化平面的坐标
};


void GetSimDataInWorldFrame(vector<Frame>& cameraPoses,vector<Eigen::Vector3d>& points){
    int featureNums = 20;// 特征点总数
    int poseNums = 3;//位姿总数

    double radius = 8;
    // 生成相机位姿
    for(int n=0;n<poseNums;++n){
        double theta = n*2*M_PI /(poseNums*4);// 在90度圆弧上插值
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());//平移
        Eigen::Vector3d t = Eigen::Vector3d(radius*cos(theta)-radius,radius*sin(theta),1*sin(2*theta) );
        cameraPoses.push_back(Frame(R,t));
    }
    //生成观测点
    std::default_random_engine generator;
    std::normal_distribution<double> noise_pdf(0,1/1000.);//
    for(int j = 0;j<featureNums;j++){
        //uniform_real_distribution：指定范围的随机实数。
        std::uniform_real_distribution<double> xy_rand(-4,4.0);
        std::uniform_real_distribution<double> z_rand(4,8);
        // 产生三维随机点
        Eigen::Vector3d Pw(xy_rand(generator),xy_rand(generator),z_rand(generator));
        points.push_back(Pw);

        for(int i=0;i<poseNums;i++){
            // 将世界坐标系下的三维点转换到相机坐标系下
            Eigen::Vector3d Pc = cameraPoses[i].Rwc.transpose()*(Pw-cameraPoses[i].twc);
            Pc = Pc/Pc.z();// 转换到归一化平面
            Pc[0] += noise_pdf(generator);
            Pc[1] += noise_pdf(generator);
            cameraPoses[i].featurePerId.insert(make_pair(j,Pc));
        }

    }


}


int main(){
    vector<Frame> cameras; // Frame类即每个观测帧
    vector<Eigen::Vector3d> points;
    GetSimDataInWorldFrame(cameras, points);
    Eigen::Quaterniond qic(1,0,0,0);//w,x,y,z
    Eigen::Vector3d tic(0,0,0) // ?
    //构建模型
    Problem problem(Problem::ProblemType::SLAM_PROBLEM);
    
    vector<shared_ptr<VertexPose> vertexCam(new VertexPose());
    Eigen::VectorXd pose(7);
    

    return 0;
}