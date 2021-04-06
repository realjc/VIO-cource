
class IMUIntegration{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    explicit IMUIntegration(const Vec3 &ba, const Vec3 &bg) : ba_(ba), bg_(bg) {
        const Mat33 i3 = Mat33::Identity();
        noise_measurement_.block<3, 3>(0, 0) = (acc_noise_ * acc_noise_) * i3;
        noise_measurement_.block<3, 3>(3, 3) = (gyr_noise_ * gyr_noise_) * i3;
        noise_random_walk_.block<3, 3>(0, 0) = (acc_random_walk_ * acc_random_walk_) * i3;
        noise_random_walk_.block<3, 3>(3, 3) = (gyr_random_walk_ * gyr_random_walk_) * i3;
    }
    ~IMUIntegration(){}

    void Propagate(double dt, const Vec3 &acc, const Vec3&gyr);

    void Correct(const Vec3 &delta_ba, const Vec3 &delta_bg);

    void SetBiasG(const Vec3 &bg){bg_ = bg;}

    void SetBiasA(const Vec3 &ba) { ba_ = ba; }

    void Repropagate();

    void Reset(){
        sum_dt_ = 0;
        delta_r_ = Sophus::SO3d();
        delta_v_ = Vec3::Zeros();
        delta_p_ = Vec3::Zeros();

        // jacobian w.r.t bg and ba
        dr_dbg_ = Mat33::Zero();
        dv_dbg_ = Mat33::Zero();
        dv_dba_ = Mat33::Zero();
        dp_dbg_ = Mat33::Zero();
        dp_dba_ = Mat33::Zero();

        // noise propagation
        covariance_measurement_ = Mat99::Zero();
        covariance_random_walk_ = Mat66::Zero();
        A_ = Mat99::Zero();
        B_ = Mat96::Zero();
    }

    
};