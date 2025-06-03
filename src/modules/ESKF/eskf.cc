#include "eskf.h"

#include <Eigen/SparseCore>
#include <unsupported/Eigen/MatrixFunctions>
#include "common_utils/so3_math.h"

// ESKFConfig的Print方法实现
void ESKFConfig::Print() const {
    PRINT_INFO("ESKF Configuration Parameters:\n");
    PRINT_INFO("  State Dimension: %d\n", STATE_DIM);
    PRINT_INFO("  IMU Sampling Time: %.6f ms\n", freq * 1000);
    PRINT_INFO("  Sensor Noise Standard Deviation:\n");
    PRINT_INFO("    Accelerometer: x=%.6e, y=%.6e, z=%.6e\n", 
             sensor_noise_std.acc_noise_std_x, sensor_noise_std.acc_noise_std_y, sensor_noise_std.acc_noise_std_z);
    PRINT_INFO("    Gyroscope: x=%.6e, y=%.6e, z=%.6e\n", 
             sensor_noise_std.gyro_noise_std_x, sensor_noise_std.gyro_noise_std_y, sensor_noise_std.gyro_noise_std_z);
    PRINT_INFO("  Bias Random Walk Standard Deviation:\n");
    PRINT_INFO("    Accelerometer: x=%.6e, y=%.6e, z=%.6e\n", 
             bias_random_walk_std.acc_bias_std_x, bias_random_walk_std.acc_bias_std_y, bias_random_walk_std.acc_bias_std_z);
    PRINT_INFO("    Gyroscope: x=%.6e, y=%.6e, z=%.6e\n", 
             bias_random_walk_std.gyro_bias_std_x, bias_random_walk_std.gyro_bias_std_y, bias_random_walk_std.gyro_bias_std_z);
    PRINT_INFO("  Initial Standard Deviation:\n");
    PRINT_INFO("    Position: %.6e, Velocity: %.6e, Attitude: %.6e\n", 
             initial_std.pos_init_std, initial_std.vel_init_std, initial_std.rot_init_std);
    PRINT_INFO("    Gyro Bias: %.6e, Accel Bias: %.6e, Gravity: %.6e\n", 
             initial_std.gyro_bias_init_std, initial_std.acc_bias_init_std, initial_std.gravity_init_std);
}

// 默认构造函数（使用默认配置）
ESKF::ESKF() : ESKF(ESKFConfig{}) {}

// 使用配置的构造函数
ESKF::ESKF(const ESKFConfig& config) : config_(config) {
    // 验证配置
    if (!config_.IsValid()) {
        PRINT_ERROR("Invalid ESKF configuration parameters");
    }

    // 基本状态变量初始化
    current_time_ = 0.0;
    r_ = Mat3d::Identity();
    p_ = Vec3d::Zero();
    v_ = Vec3d::Zero();
    bg_ = Vec3d::Zero();
    ba_ = Vec3d::Zero();
    g_ = Vec3d(0, 0, -config_.gravity);

    // 快速状态初始化
    fast_r_ = Mat3d::Identity();
    fast_p_ = Vec3d::Zero();
    fast_v_ = Vec3d::Zero();
    init_bg_ = Vec3d::Zero();
    init_ba_ = Vec3d::Zero();
    init_g_ = Vec3d(0, 0, -config_.gravity);

    // 初始化协方差矩阵
    InitializeCovarianceMatrix();
    // 初始化传感器噪声矩阵
    InitializeSensorNoiseMatrices();

    // 打印配置信息
    config_.Print();
    PRINT_INFO("ESKF initialization completed. 18-dimensional state vector: [r, p, v, bg, ba, g]\n");
}

ESKF::~ESKF() {}

void ESKF::InitializeCovarianceMatrix() {
    // 初始化18x18协方差矩阵
    cov_.setZero();
    
    // 设置对应的协方差
    cov_.block<3,3>(0,0) = Mat3d::Identity() * config_.initial_std.rot_init_std * config_.initial_std.rot_init_std;        // 姿态 [0-2]
    cov_.block<3,3>(3,3) = Mat3d::Identity() * config_.initial_std.pos_init_std * config_.initial_std.pos_init_std;        // 位置 [3-5]
    cov_.block<3,3>(6,6) = Mat3d::Identity() * config_.initial_std.vel_init_std * config_.initial_std.vel_init_std;        // 速度 [6-8]
    cov_.block<3,3>(9,9) = Mat3d::Identity() * config_.initial_std.gyro_bias_init_std * config_.initial_std.gyro_bias_init_std;   // 陀螺仪偏置 [9-11]
    cov_.block<3,3>(12,12) = Mat3d::Identity() * config_.initial_std.acc_bias_init_std * config_.initial_std.acc_bias_init_std; // 加速度计偏置 [12-14]
    cov_.block<3,3>(15,15) = Mat3d::Identity() * config_.initial_std.gravity_init_std * config_.initial_std.gravity_init_std;  // 重力 [15-17]

    // 初始化其他矩阵
    pro_j_.setIdentity();
    F_x_.setIdentity();
    f_x_.setZero();
    Q_x_.setZero();
    I_matrix_.setIdentity();
    delta_x_.setZero();
}

void ESKF::InitializeSensorNoiseMatrices() {
    // 加速度计噪声协方差
    acc_cov_ = Vec3d(config_.sensor_noise_std.acc_noise_std_x * config_.sensor_noise_std.acc_noise_std_x,
                     config_.sensor_noise_std.acc_noise_std_y * config_.sensor_noise_std.acc_noise_std_y,
                     config_.sensor_noise_std.acc_noise_std_z * config_.sensor_noise_std.acc_noise_std_z)
                   .asDiagonal();

    // 陀螺仪噪声协方差
    gyr_cov_ = Vec3d(config_.sensor_noise_std.gyro_noise_std_x * config_.sensor_noise_std.gyro_noise_std_x,
                     config_.sensor_noise_std.gyro_noise_std_y * config_.sensor_noise_std.gyro_noise_std_y,
                     config_.sensor_noise_std.gyro_noise_std_z * config_.sensor_noise_std.gyro_noise_std_z)
                   .asDiagonal();

    // 加速度计偏置随机游走协方差
    bias_acc_cov_ = Vec3d(config_.bias_random_walk_std.acc_bias_std_x * config_.bias_random_walk_std.acc_bias_std_x,
                          config_.bias_random_walk_std.acc_bias_std_y * config_.bias_random_walk_std.acc_bias_std_y,
                          config_.bias_random_walk_std.acc_bias_std_z * config_.bias_random_walk_std.acc_bias_std_z)
                        .asDiagonal();

    // 陀螺仪偏置随机游走协方差
    bias_gyr_cov_ = Vec3d(config_.bias_random_walk_std.gyro_bias_std_x * config_.bias_random_walk_std.gyro_bias_std_x,
                          config_.bias_random_walk_std.gyro_bias_std_y * config_.bias_random_walk_std.gyro_bias_std_y,
                          config_.bias_random_walk_std.gyro_bias_std_z * config_.bias_random_walk_std.gyro_bias_std_z)
                        .asDiagonal();
}

// void ESKF::ProjectCov() {
//     pro_j_.block<3, 3>(0, 0) =
//         Mat3d::Identity() - 0.5 * Skew(delta_x_.block<3, 1>(0, 0).eval());
//     cov_ = pro_j_ * cov_ * pro_j_.transpose();
// }

// void ESKF::UpdateStates() {
//     r_ = r_ * Exp(delta_x_.block<3, 1>(0, 0).eval());
//     p_ += delta_x_.block<3, 1>(3, 0);
//     v_ += delta_x_.block<3, 1>(6, 0);
//     bg_ += delta_x_.block<3, 1>(9, 0);
//     ba_ += delta_x_.block<3, 1>(12, 0);
//     g_ += delta_x_.block<3, 1>(15, 0);

//     ProjectCov();
//     delta_x_.setZero();
// }

// void ESKF::CalKalmanGainAndCov(const MatXd& H, const MatXd& R, const VecXd& z) {
//     auto K = cov_ * H.transpose() * (H * cov_ * H.transpose() + R).inverse();
//     delta_x_ = K * z;
    
//     PRINT_INFO(" 状态增量:");
//     PRINT_INFO(" add_r = [{:.6f}, {:.6f}, {:.6f}]", delta_x_(0), delta_x_(1), delta_x_(2));
//     PRINT_INFO(" add_p = [{:.6f}, {:.6f}, {:.6f}]", delta_x_(3), delta_x_(4), delta_x_(5));
//     PRINT_INFO(" add_v = [{:.6f}, {:.6f}, {:.6f}]", delta_x_(6), delta_x_(7), delta_x_(8));
//     PRINT_INFO(" add_bg = [{:.6f}, {:.6f}, {:.6f}]", delta_x_(9), delta_x_(10), delta_x_(11));
//     PRINT_INFO(" add_ba = [{:.6f}, {:.6f}, {:.6f}]", delta_x_(12), delta_x_(13), delta_x_(14));
//     PRINT_INFO(" add_g = [{:.4f}, {:.4f}, {:.4f}]", delta_x_(15), delta_x_(16), delta_x_(17));
    
//     cov_ = (I_matrix_ - K * H) * cov_;
// }

// void ESKF::PrintStates(const std::string& str_states) {
//     Vec3d eular = Rot2Rpy(r_) * 57.3;
//     PRINT_INFO(" " + std::to_string(current_time_) + ", " + str_states);
//     PRINT_INFO(" r = [{:.6f}, {:.6f}, {:.6f}] (deg)", eular(0), eular(1), eular(2));
//     PRINT_INFO(" p = [{:.6f}, {:.6f}, {:.6f}] (m)", p_(0), p_(1), p_(2));
//     PRINT_INFO(" v = [{:.6f}, {:.6f}, {:.6f}] (m/s)", v_(0), v_(1), v_(2));
//     PRINT_INFO(" bg = [{:.8f}, {:.8f}, {:.8f}] (rad/s)", bg_(0), bg_(1), bg_(2));
//     PRINT_INFO(" ba = [{:.8f}, {:.8f}, {:.8f}] (m/s^2)", ba_(0), ba_(1), ba_(2));
//     PRINT_INFO(" g = [{:.4f}, {:.4f}, {:.4f}] (m/s^2)", g_(0), g_(1), g_(2));
// }

// void ESKF::PrintDiff(const Mat3d& r_truth, const Vec3d& p_truth) {
//     Vec3d p_offset = p_truth - p_;

//     Vec3d eular = Rot2Rpy(r_) * 180 / M_PI;
//     Vec3d eular_truth = Rot2Rpy(r_truth) * 180 / M_PI;
//     Vec3d r_offset = eular_truth - eular;
    
//     PRINT_INFO(" " + std::to_string(current_time_) + ", diff: ");
//     PRINT_INFO(" p diff(m) = [{:.6f}, {:.6f}, {:.6f}]", p_offset(0), p_offset(1), p_offset(2));
//     PRINT_INFO(" r diff(degs) = [{:.6f}, {:.6f}, {:.6f}]", r_offset(0), r_offset(1), r_offset(2));
// }

// void ESKF::GetDiff(const Mat3d& r_truth, const Vec3d& p_truth, Vec3d& p_offset, Vec3d& r_offset) {
//     p_offset = (p_truth - p_) * 100;

//     Vec3d eular = Rot2Rpy(r_) * 180 / M_PI;
//     Vec3d eular_truth = Rot2Rpy(r_truth) * 180 / M_PI;
//     r_offset = eular_truth - eular;
// }