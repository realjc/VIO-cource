### 基础作业

#### 1、设置IMU仿真代码中的不同参数， 生成Allen方差标定曲线
这里设置了三组实验参数，分别对应高精度、中精度、低精度的IMU。
||IMU0|IMU1|IMU2|
|-|-|-|-|
|陀螺仪高斯白噪声($m/s^2\sqrt{Hz}$)|1.5e-3|1.5e-2|1.5e-1|
|陀螺仪随机游走噪声($m/s^3\sqrt{Hz}$)|5e-6|5e-5|5e-4|
|加速度计高斯白噪声($rad/s\sqrt{Hz}$)|1.9e-3|1.9e-2|1.9e-1|
|加速度计随机游走噪声($rad/s^2\sqrt{Hz}$)|5e-5|5e-4|5e-3|


##### 实验结果
**IMU0**
仿真数据：
![avatar](./IMU0Sim.png)
标定结果：
![avatar](./IMU0Calib.png)
Allen曲线：
![avatar](./IMU0Allen.png)
**IMU1**
仿真数据：
![avatar](./IMU1Sim.png)
标定结果：
![avatar](./IMU1Calib.png)
Allen曲线：
![avatar](./IMU1Allen.png)
**IMU2**
仿真数据：
![avatar](./IMU2Sim.png)
标定结果：
![avatar](./IMU2Calib.png)
Allen曲线：
![avatar](./IMU2Allen.png)

#### 2、使用中值积分替换欧拉积分
中值积分代码：
```cpp
// 中值积分
MotionData last_imupose = imudata[i-1];
MotionData imupose = imudata[i];
Eigen::Quaterniond dq;
Eigen::Vector3d dtheta_half = (last_imupose.imu_gyro+ imupose.imu_gyro)* dt / 4.0;
dq.w() = 1;
dq.x() = dtheta_half.x();
dq.y() = dtheta_half.y();
dq.z() = dtheta_half.z();
dq.normalize();
Eigen::Vector3d acc_w = (Qwb * dq * (imupose.imu_acc) + gw + Qwb * (last_imupose.imu_acc) + gw)/2;
Qwb = Qwb * dq;
Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
Vw = Vw + acc_w * dt;
```
欧拉积分与中值积分的结果比较：
![avatar](./euler.png)
![avatar](./median.png)
