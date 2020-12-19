
### 1、VIO文献阅读
- 1.1、视觉与IMU融合的优势
    - 对于单目相机，与IMU融合可消除其尺度不确定性
    - IMU虽然有累积误差，但对快速运动也很敏感
    - 对于视觉SLAM的劣势场景：弱纹理、光照变化、动态环境，IMU可以提供参考位姿
    - 两者融合后，能够提高定位精度
- 1.2、常见的视觉+IMU方案
  - 基于滤波的：MSCKF、ROVIO
  - 基于优化的：VINS-Mono、okvis
  - 业界应用：ARKit、Tango、Hololens
- 1.3、VIO新进展
  - 基于深度学习的VIO
    - DeepVIO: Self-supervised Deep Learning of Monocular Visual Inertial Odometry using 3D Geometric Constraints
      - DeepVIO包括光流估计网络和IMU积分网络、VI融合网络, 直接输出绝对位姿
    - Unsupervised Monocular Visual-inertial Odometry Network
      - 学习生成深度图，并构建滑窗优化融合IMU



### 2、四元数和李代数更新
代码：
```cpp
#include <iostream>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
#include<ctime>
#include<random>
using namespace std;

Eigen::Matrix3d update_by_lie(Eigen::Matrix3d initMatrix, Eigen::Vector3d w )
{
    double theta=w.norm();
    Eigen::Vector3d n_w=w/theta;
    Eigen::Matrix3d n_w_skew;
    n_w_skew<<   0,    -n_w(2),    n_w(1),
		n_w(2),     0,     -n_w(0),
	       -n_w(1),  n_w(0),      0;
    Eigen::Matrix3d R_w=cos(theta)*Eigen::Matrix3d::Identity()+(1-cos(theta))*n_w*n_w.transpose()+sin(theta)*n_w_skew;
    Eigen::Matrix3d rUpdated=initMatrix*R_w;
    return rUpdated;
}

Eigen::Matrix3d update_by_quat(Eigen::Matrix3d initMatrix, Eigen::Vector3d w )
{
    Eigen::Quaterniond q(initMatrix);
    Eigen::Quaterniond q_w(1,w(0)/2,w(1)/2,w(2)/2);
    Eigen::Quaterniond q_update=q*q_w;
    q_update=q_update.normalized();
    return q_update.toRotationMatrix();
}

int main(int argc, char **argv) {
    random_device e; 
    uniform_real_distribution<double> u(0, 1);
    Eigen::Vector3d w(0.01,0.02,0.03);
    Eigen::Matrix3d rUpdated1,rUpdated2;
    for(int i =0;i<10;i++){
        double roll =  2*u(e)*M_PI;
        double yaw = 2*u(e)*M_PI;
        double pitch = 2*u(e)*M_PI;
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
        Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
        Eigen::Matrix3d initMatrix = q.matrix();
        rUpdated1 = update_by_lie(initMatrix,w);
        rUpdated2 = update_by_quat(initMatrix,w);
        cout<<rUpdated1-rUpdated2<<endl;
        cout<<endl;
    }
    return 0;
}
```
运行结果：

![avatar](./Screenshot%20from%202020-12-10%2000-31-31.png)

### 3、其他导数

#### 题1：

$$\frac{d(R^{-1}p)}{dR}$$
这里采用右扰动模型进行求导：
扰动$\Delta R$对应的李代数为$\varphi$, 对$\varphi$求导有：

$$\frac{d(R^{-1}p)}{d\varphi} = \lim_{\varphi \rightarrow 0} \frac{(R \exp(\varphi ^{\wedge} ))^{-1}p-R^{-1}p}{\varphi} $$
由$(AB)^{-1}=B^{-1}A^{-1}$有
$$\lim_{\varphi \rightarrow 0} \frac{\exp(\varphi ^{\wedge} )^{-1}R^{-1}p-R^{-1}p}{\varphi}$$ 
对指数项进行展开:
$$\exp(\varphi ^{\wedge} )= \sum_{n=0}^{\infty} \frac{1}{n!} (\varphi^{\wedge})^n$$
仅保留线性项:
$$\lim_{\varphi \rightarrow 0} \frac{(I+\varphi^{\wedge})^{-1}R^{-1}p-R^{-1}p}{\varphi}$$ 
$$=\lim_{\varphi \rightarrow 0} \frac{(I-\varphi^{\wedge})R^{-1}p-R^{-1}p}{\varphi}$$ 
$$=\lim_{\varphi \rightarrow 0} \frac{-\varphi^{\wedge}(R^{-1}p)}{\varphi}$$ 

由外积公式$a\times b=-b \times a=a^{\wedge}b=-b^{\wedge}a$有：
$$\lim_{\varphi \rightarrow 0} \frac{(R^{-1}p)^{\wedge}\varphi}{\varphi}\\
=(R^{-1}p)^{\wedge}$$ 

#### 题2：
$$\frac{d\ln(R_1R_2^{-1})^{\vee}}{dR_2}$$
同样采用右扰动模型进行求导。
扰动$\Delta R$对应的李代数为$\varphi$, 对$\varphi$求导有：
$$\frac{d\ln(R_1R_2^{-1})^{\vee}}{dR_2} = \lim_{\varphi \rightarrow 0} \frac{\ln(R_1(R_2\exp(\varphi^{\wedge}))^{-1})^{\vee}-\ln(R_1R_2^{-1})^{\vee}}{\varphi} $$

$$= \lim_{\varphi \rightarrow 0} \frac{\ln(R_1\exp(\varphi^{\wedge})^{-1}R_2^{-1})^{\vee}-\ln(R_1R_2^{-1})^{\vee}}{\varphi} $$

$$= \lim_{\varphi \rightarrow 0} \frac{\ln(R_1R_2^{-1}R_2\exp(\varphi^{\wedge})^{-1}R_2^{-1})^{\vee}-\ln(R_1R_2^{-1})^{\vee}}{\varphi} $$

由$SO_3$的伴随性质$R\exp(p^{\wedge})R^T=\exp((Rp)^{\wedge})$有：
$$\lim_{\varphi \rightarrow 0} \frac{\ln(R_1R_2^{-1}\exp(-R_2\varphi^{\wedge}))^{\vee}-\ln(R_1R_2^{-1})^{\vee}}{\varphi} $$

根据BCH的线性近似：
$$\ln(\exp(\varphi_1^{\wedge})\exp(\varphi_2^{\wedge})) \approx 
\begin{cases}
J_l(\varphi_2)^{-1}\varphi_1+\varphi_2& \varphi_1 \to 0  \\
J_r(\varphi_1)^{-1}\varphi_2+\varphi_1& \varphi_2 \to 0
\end{cases}
$$
有：
$$\lim_{\varphi \rightarrow 0} \frac{\ln(R_1R_2^{-1})^{\vee}-J_r^{-1}(\ln(R_1R_2^{-1})^{\vee})(R_2\varphi)-\ln(R_1R_2^{-1})^{\vee}}{\varphi} =\\
-J_r^{-1}(\ln(R_1R_2^{-1})^{\vee})R_2
$$

