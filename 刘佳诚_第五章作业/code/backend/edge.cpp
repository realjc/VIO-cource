




void EdgeReprojection::ComputeResidual(){
    double inv_dep_i = vertices_[0]->Prameters()[0];
    VecX param_i = verticies_[0]->Parameters();
    Qd Qi(param_i[6], param_i[3],param_i[4],param_i[5]);
    Vec3 Pi = param_i.head<3>();

    VecX param_j = verticies_[2]->Parameters();
    Qd Qj(param_j[6], param_j[3], param_j[4], param_j[5]);
    Vec3 Pj = param_j.head<3>();

    Vec3 pts_camera_i = pts_i /inv_dep_i;
    Vec3 pts_imu_i = qic*pts_camera_i + tic; // 转到imu-i坐标系
    Vec3 pts_w = Qi*pts_imu_i + Pi;// 转到世界坐标系
    Vec3 pts_imu_j = Qj.inverse()*(pts_w-Pj);//转到imu-j坐标系
    Vec3 pts_camera_j qic.inverse()*(pts_imu_j-tic);//转到camera-j
    double dep_j = pts_camera_j.z();
    residual_ = (pts_camera_j/dep_j).head<2>() - pts_j_.head<2>();    
}

void EdgeReprojection::SetTranslationImuFromCamera(Eigen::Quaterniond &qic_, Vec3 &tic_) {
    qic = qic_;
    tic = tic_;
}

void EdgeReprojection::ComputeJacobians(){
    double inv_dep_i = verticies_[0]->Parameters()[0]; //特征点i逆深度

    VecX param_i = verticies_[1]->Parameters();  // cam_i
    Qd Qi(param_i[6], param_i[3], param_i[4], param_i[5]);
    Vec3 Pi = param_i.head<3>();

    VecX param_j = verticies_[2]->Parameters();// cam_j
    Qd Qj(param_j[6], param_j[3], param_j[4], param_j[5]);
    Vec3 Pj = param_j.head<3>();

    Vec3 pts_camera_i = pts_i_ / inv_dep_i;
    Vec3 pts_imu_i = qic * pts_camera_i + tic;
    Vec3 pts_w = Qi * pts_imu_i + Pi;
    Vec3 pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Vec3 pts_camera_j = qic.inverse() * (pts_imu_j - tic);

    double dep_j = pts_camera_j.z();

    Mat33 Ri = Qi.toRotationMatrix();
    Mat33 Rj = Qj.toRotationMatrix();
    Mat33 ric = qic.toRotationMatrix();
    Mat23 reduce(2, 3);
    reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),// 反投影误差对dep_ 求导
        0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

    Eigen::Matrix<double, 2, 6> jacobian_pose_i;
    Eigen::Matrix<double, 3, 6> jaco_i;
    jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose(); // 转换到世界坐标系
    jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri * -Sophus::SO3d::hat(pts_imu_i);//(I,- pts_imu_i^)
    jacobian_pose_i.leftCols<6>() = reduce * jaco_i;

    Eigen::Matrix<double, 2, 6> jacobian_pose_j;
    Eigen::Matrix<double, 3, 6> jaco_j;
    jaco_j.leftCols<3>() = ric.transpose() * Rj.transpose(); 
    jaco_j.rightCols<3>() = ric.transpose() * Sophus::SO3d::hat(pts_imu_j);
    jacobian_pose_j.leftCols<6>() = reduce * jaco_j;

    Eigen::Vector2d jacobian_feature;
    jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i_ * -1.0 / (inv_dep_i * inv_dep_i);

    jacobians_[0] = jacobian_feature;
    jacobians_[1] = jacobian_pose_i;
    jacobians_[2] = jacobian_pose_j;
    
}
// 一元边，只有pose, 可以看作是gnss的观测
void EdgeSE3Prior::ComputeResidual() {
    VecX param_i = verticies_[0]->Parameters();
    Qd Qi(param_i[6], param_i[3], param_i[4], param_i[5]);
    Vec3 Pi = param_i.head<3>();

    Sophus::SO3d ri(Qi);
    Sophus::SO3d rp(Qp_);
    Sophus::SO3d res_r = rp.inverse()*ri;

    residual_.block<3,1>(0,0) = Sophus::SO3d::log(res_r);
    residual_.block<3,1>(3,0) = Pi - Pp_;
}

void EdgeSE3Prior::ComputeJacobians(){
    VecX param_i = verticies_[0]->Parameters();
    Qd Qi(param_i[6], param_i[3], param_i[4], param_i[5]);
    Eigen::Matrix<double, 6, 6> jacobian_pose_i = Eigen::Matrix<double, 6, 6>::Zero();
    Sophus::SO3d ri(Qi);
    Sophus::SO3d rp(Qp_);
    Sophus::SO3d res_r = rp.inverse() * ri;
    // http://rpg.ifi.uzh.ch/docs/RSS15_Forster.pdf  公式A.32
    jacobian_pose_i.block<3,3>(0,3) = Sophus::SO3d::JacobianRInv(res_r.log());
    jacobian_pose_i.block<3,3>(3,0) = Mat33::Identity();

    jacobians_[0] = jacobian_pose_i;
}


