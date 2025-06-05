#pragma once

#include <Eigen/Core>
#include <memory>

#include "common_utils/common_data_types.h"
#include "common_utils/print.h"

// ESKF类，固定使用18维状态 [r, p, v, bg, ba, g]
// 其中：r(姿态), p(位置), v(速度), bg(陀螺偏置), ba(加速偏置), g(重力向量)

// ESKF配置结构体
struct ESKFConfig {
    // 基本配置
    static constexpr int STATE_DIM = 18;  // 固定18维状态
    double freq = 0.01;                   // IMU采样间隔，单位s，即100hz
    double gravity = 9.8012;              // 单位m/s^2

    // 传感器噪声标准差配置
    struct SensorNoiseStdConfig {
        double acc_noise_std_x = 9.8e-3;  // 单位m/s^2
        double acc_noise_std_y = 9.8e-3;
        double acc_noise_std_z = 9.8e-3;
        double gyro_noise_std_x = 1.9e-4;  // 单位rad/s
        double gyro_noise_std_y = 1.9e-4;
        double gyro_noise_std_z = 4.8481e-5;
    } sensor_noise_std;

    // 偏置随机游走标准差配置
    struct BiasRandomWalkStdConfig {
        double acc_bias_std_x = 6.6667e-7;  // 单位m/s^2
        double acc_bias_std_y = 6.6667e-7;
        double acc_bias_std_z = 6.6667e-7;
        double gyro_bias_std_x = 1.0167e-7;  // 单位rad/s
        double gyro_bias_std_y = 1.0167e-7;
        double gyro_bias_std_z = 1.0167e-8;
    } bias_random_walk_std;

    // 初始标准差配置
    struct InitialStdConfig {
        double pos_init_std = 1e-2;  // 位置初始标准差：初始位置通常不太确定，设为较大值
        double vel_init_std = 1e-3;  // 速度初始标准差：初始速度通常可以设为0，但有一定不确定性
        double rot_init_std = 1e-2;  // 姿态初始标准差：初始姿态通常不太确定，设为较大值
        double gyro_bias_init_std = 1e-4;  // 陀螺仪偏置初始标准差：偏置通常较小，保持较小值
        double acc_bias_init_std = 1e-4;  // 加速度计偏置初始标准差：偏置通常较小，保持较小值
        double gravity_init_std = 1e-3;  // 重力向量初始标准差：重力向量有一定不确定性
    } initial_std;

    // 配置验证方法
    bool IsValid() const { return freq > 0; }

    // 配置打印方法
    void Print() const;
};

class ESKF {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<const ESKF> ConstPtr;
    typedef std::shared_ptr<ESKF> Ptr;

    using Vec3d = Eigen::Matrix<double, 3, 1>;
    using Vec2d = Eigen::Matrix<double, 2, 1>;
    using Vec2f = Eigen::Matrix<float, 2, 1>;
    using Vec4d = Eigen::Matrix<double, 4, 1>;
    using Mat3d = Eigen::Matrix<double, 3, 3>;
    using Mat4d = Eigen::Matrix<double, 4, 4>;
    using Mat18d = Eigen::Matrix<double, 18, 18>;
    using VecXd = Eigen::VectorXd;
    using MatXd = Eigen::MatrixXd;

    // 工厂方法创建实例
    static Ptr Create(const ESKFConfig& config) { return std::make_shared<ESKF>(config); }

    // 使用配置的构造函数
    explicit ESKF(const ESKFConfig& config);

    // 默认构造函数（使用默认配置）
    ESKF();

    ~ESKF();

    void SetInitConditions(const double& timestamp, const Vec3d& init_bg, const Vec3d& init_ba,
                           const Mat3d& init_r, const Vec3d& init_p);

    bool Predict(const std::map<double, IMUData>& imu_data);

    void Predict(std::vector<IMUData>);

    void PrintStates(const std::string& str_states = std::string("ESKF states: "));

    void PrintDiff(const Mat3d& r_truth, const Vec3d& p_truth);

    void GetDiff(const Mat3d& r_truth, const Vec3d& p_truth, Vec3d& p_offset, Vec3d& r_offset);

    Mat3d r() { return r_; }
    Vec3d p() { return p_; }
    Vec3d v() { return v_; }
    Vec3d bg() { return bg_; }
    Vec3d ba() { return ba_; }
    Vec3d g() { return g_; }

   private:
    // 初始化协方差矩阵
    void InitializeCovarianceMatrix();

    // 初始化传感器噪声矩阵
    void InitializeSensorNoiseMatrices();

    void CalKalmanGainAndCov(const MatXd& H, const MatXd& R, const VecXd& z);

    void UpdateStates();

    void ProjectCov();

    // 18维状态变量
    Mat3d r_;   // [0-2] 姿态
    Vec3d p_;   // [3-5] 位置
    Vec3d v_;   // [6-8] 速度
    Vec3d bg_;  // [9-11] 陀螺仪偏置
    Vec3d ba_;  // [12-14] 加速度计偏置
    Vec3d g_;   // [15-17] 重力向量

    // predict state (用于纯IMU积分, 验证IMU预测结果, 不参与卡尔曼滤波)
    Vec3d predict_bg_;
    Vec3d predict_ba_;
    Vec3d predict_g_;
    Mat3d predict_r_;
    Vec3d predict_p_;
    Vec3d predict_v_;

    double current_time_;
    IMUData current_imu_data_;

    Mat3d gyr_cov_;
    Mat3d acc_cov_;
    Mat3d bias_gyr_cov_;
    Mat3d bias_acc_cov_;

    ESKFConfig config_;
    Eigen::Matrix<double, 18, 1> delta_x_;    // 固定18维
    Eigen::Matrix<double, 18, 18> cov_;       // 固定18x18协方差矩阵
    Eigen::Matrix<double, 18, 18> pro_j_;     // 固定18x18
    Eigen::Matrix<double, 18, 18> F_x_;       // 固定18x18
    Eigen::Matrix<double, 18, 18> f_x_;       // 固定18x18
    Eigen::Matrix<double, 18, 18> Q_x_;       // 固定18x18
    Eigen::Matrix<double, 18, 18> I_matrix_;  // 固定18x18单位矩阵
};