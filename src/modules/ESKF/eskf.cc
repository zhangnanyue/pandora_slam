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
    PRINT_INFO("    Accelerometer: x=%.6e, y=%.6e, z=%.6e\n", sensor_noise_std.acc_noise_std_x,
               sensor_noise_std.acc_noise_std_y, sensor_noise_std.acc_noise_std_z);
    PRINT_INFO("    Gyroscope: x=%.6e, y=%.6e, z=%.6e\n", sensor_noise_std.gyro_noise_std_x,
               sensor_noise_std.gyro_noise_std_y, sensor_noise_std.gyro_noise_std_z);
    PRINT_INFO("  Bias Random Walk Standard Deviation:\n");
    PRINT_INFO("    Accelerometer: x=%.6e, y=%.6e, z=%.6e\n", bias_random_walk_std.acc_bias_std_x,
               bias_random_walk_std.acc_bias_std_y, bias_random_walk_std.acc_bias_std_z);
    PRINT_INFO("    Gyroscope: x=%.6e, y=%.6e, z=%.6e\n", bias_random_walk_std.gyro_bias_std_x,
               bias_random_walk_std.gyro_bias_std_y, bias_random_walk_std.gyro_bias_std_z);
    PRINT_INFO("  Initial Standard Deviation:\n");
    PRINT_INFO("    Position: %.6e, Velocity: %.6e, Attitude: %.6e\n", initial_std.pos_init_std,
               initial_std.vel_init_std, initial_std.rot_init_std);
    PRINT_INFO("    Gyro Bias: %.6e, Accel Bias: %.6e, Gravity: %.6e\n",
               initial_std.gyro_bias_init_std, initial_std.acc_bias_init_std,
               initial_std.gravity_init_std);
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

    // 这里的状态量仅参与纯IMU积分，不参与卡尔曼滤波
    predict_r_ = Mat3d::Identity();
    predict_p_ = Vec3d::Zero();
    predict_v_ = Vec3d::Zero();
    predict_bg_ = Vec3d::Zero();
    predict_ba_ = Vec3d::Zero();
    predict_g_ = Vec3d(0, 0, -config_.gravity);

    // 初始化协方差矩阵
    InitializeCovarianceMatrix();
    // 初始化传感器噪声矩阵
    InitializeSensorNoiseMatrices();

    // 打印配置信息
    config_.Print();
    PRINT_INFO(
        "ESKF initialization completed. 18-dimensional state vector: [r, p, v, bg, ba, g]\n");
}

ESKF::~ESKF() {}

void ESKF::InitializeCovarianceMatrix() {
    // 初始化18x18协方差矩阵
    cov_.setZero();

    // 设置对应的协方差
    cov_.block<3, 3>(0, 0) = Mat3d::Identity() * config_.initial_std.rot_init_std *
                             config_.initial_std.rot_init_std;  // 姿态 [0-2]
    cov_.block<3, 3>(3, 3) = Mat3d::Identity() * config_.initial_std.pos_init_std *
                             config_.initial_std.pos_init_std;  // 位置 [3-5]
    cov_.block<3, 3>(6, 6) = Mat3d::Identity() * config_.initial_std.vel_init_std *
                             config_.initial_std.vel_init_std;  // 速度 [6-8]
    cov_.block<3, 3>(9, 9) = Mat3d::Identity() * config_.initial_std.gyro_bias_init_std *
                             config_.initial_std.gyro_bias_init_std;  // 陀螺仪偏置 [9-11]
    cov_.block<3, 3>(12, 12) = Mat3d::Identity() * config_.initial_std.acc_bias_init_std *
                               config_.initial_std.acc_bias_init_std;  // 加速度计偏置 [12-14]
    cov_.block<3, 3>(15, 15) = Mat3d::Identity() * config_.initial_std.gravity_init_std *
                               config_.initial_std.gravity_init_std;  // 重力 [15-17]

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
    acc_cov_ =
        Vec3d(config_.sensor_noise_std.acc_noise_std_x * config_.sensor_noise_std.acc_noise_std_x,
              config_.sensor_noise_std.acc_noise_std_y * config_.sensor_noise_std.acc_noise_std_y,
              config_.sensor_noise_std.acc_noise_std_z * config_.sensor_noise_std.acc_noise_std_z)
            .asDiagonal();

    // 陀螺仪噪声协方差
    gyr_cov_ =
        Vec3d(config_.sensor_noise_std.gyro_noise_std_x * config_.sensor_noise_std.gyro_noise_std_x,
              config_.sensor_noise_std.gyro_noise_std_y * config_.sensor_noise_std.gyro_noise_std_y,
              config_.sensor_noise_std.gyro_noise_std_z * config_.sensor_noise_std.gyro_noise_std_z)
            .asDiagonal();

    // 加速度计偏置随机游走协方差
    bias_acc_cov_ = Vec3d(config_.bias_random_walk_std.acc_bias_std_x *
                              config_.bias_random_walk_std.acc_bias_std_x,
                          config_.bias_random_walk_std.acc_bias_std_y *
                              config_.bias_random_walk_std.acc_bias_std_y,
                          config_.bias_random_walk_std.acc_bias_std_z *
                              config_.bias_random_walk_std.acc_bias_std_z)
                        .asDiagonal();

    // 陀螺仪偏置随机游走协方差
    bias_gyr_cov_ = Vec3d(config_.bias_random_walk_std.gyro_bias_std_x *
                              config_.bias_random_walk_std.gyro_bias_std_x,
                          config_.bias_random_walk_std.gyro_bias_std_y *
                              config_.bias_random_walk_std.gyro_bias_std_y,
                          config_.bias_random_walk_std.gyro_bias_std_z *
                              config_.bias_random_walk_std.gyro_bias_std_z)
                        .asDiagonal();
}

void ESKF::SetInitConditions(const double& timestamp, const Vec3d& init_bg, const Vec3d& init_ba,
                             const Mat3d& init_r, const Vec3d& init_p) {
    PRINT_INFO("ESKF init timestamp: %.6f\n", current_time_);

    r_ = init_r;
    p_ = init_p;
    bg_ = init_bg;
    ba_ = init_ba;

    predict_r_ = init_r;
    predict_p_ = init_p;
    predict_bg_ = init_bg;
    predict_ba_ = init_ba;

    current_time_ = timestamp;

    Eigen::Vector3d r_deg = Rot2Rpy(r_) * 180 / M_PI;
    PRINT_INFO(" init r = [%.6f, %.6f, %.6f]\n", r_deg(0), r_deg(1), r_deg(2));
    PRINT_INFO(" init p = [%.6f, %.6f, %.6f]\n", p_(0), p_(1), p_(2));
    PRINT_INFO(" init bg = [%.6f, %.6f, %.6f]\n", bg_(0), bg_(1), bg_(2));
    PRINT_INFO(" init ba = [%.6f, %.6f, %.6f]\n", ba_(0), ba_(1), ba_(2));
    PRINT_INFO(" init g = [%.6f, %.6f, %.6f]\n", g_(0), g_(1), g_(2));
}

bool ESKF::Predict(const std::map<double, IMUData>& imu_data) {
    PRINT_INFO("ESKF predict start\n");

    double dt = imu_data.begin()->first - current_time_;
    if (dt > (5 * config_.freq) || dt < 0) {
      PRINT_ERROR("imu data dt too large!!!! break ESKF predict!\n");
      current_time_ = imu_data.rbegin()->first;
      return false;
    }

    Vec3d eular = Rot2Rpy(r_) * 180 / M_PI;
    PRINT_INFO("r start = [%.6f, %.6f, %.6f]\n", eular(0), eular(1), eular(2));
    PRINT_INFO("p start = [%.6f, %.6f, %.6f]\n", p_(0), p_(1), p_(2));
    PRINT_INFO("v start = [%.6f, %.6f, %.6f]\n", v_(0), v_(1), v_(2));

    Vec3d gyr_mean = Vec3d::Zero();
    Vec3d acc_mean = Vec3d::Zero();

    auto it_imu = imu_data.begin();
    for (; it_imu != (--imu_data.end()); it_imu++) {
        auto it_head = it_imu;
        IMUData head = it_head->second;
        IMUData tail = (++it_head)->second;
        // PRINT_INFO("head-tail %.6f-%.6f\n", head.timestamp, tail.timestamp);

        gyr_mean << 0.5 * (head.gyro(0) + tail.gyro(0)), 0.5 * (head.gyro(1) + tail.gyro(1)),
            0.5 * (head.gyro(2) + tail.gyro(2));
        acc_mean << 0.5 * (head.accel(0) + tail.accel(0)), 0.5 * (head.accel(1) + tail.accel(1)),
            0.5 * (head.accel(2) + tail.accel(2));

        double dt = tail.timestamp - head.timestamp;
        if (dt > 0.02) {
            PRINT_WARNING("imu data dt too large!!!!!!\n");
        }

        Vec3d new_p =
            p_ + v_ * dt + 0.5 * (r_ * (acc_mean - ba_).eval()) * dt * dt + 0.5 * g_ * dt * dt;
        Vec3d new_v = v_ + (r_ * (acc_mean - ba_).eval()) * dt + g_ * dt;
        Mat3d new_r = r_ * Exp((gyr_mean - bg_).eval(), dt);

        p_ = new_p;
        v_ = new_v;
        r_ = new_r;

        
        Vec3d new_predict_p = predict_p_ + predict_v_ * dt +
                            0.5 * (predict_r_ * (acc_mean - predict_ba_).eval()) * dt * dt +
                            0.5 * predict_g_ * dt * dt;
        Vec3d new_predict_v =
            predict_v_ + (predict_r_ * (acc_mean - predict_ba_).eval()) * dt + predict_g_ * dt;
        Mat3d new_predict_r = predict_r_ * Exp((gyr_mean - predict_bg_).eval(), dt);

        predict_p_ = new_predict_p;
        predict_v_ = new_predict_v;
        predict_r_ = new_predict_r;
        

        // r, p, v, bg, ba, g
        // theta to theta
        F_x_.block<3, 3>(0, 0) = Exp((gyr_mean - bg_).eval(), -dt);
        // theta to bg
        F_x_.block<3, 3>(0, 9) = -Eigen::Matrix3d::Identity() * dt;
        // p to v
        F_x_.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
        // v to theta
        F_x_.block<3, 3>(6, 0) = -r_ * Skew((acc_mean - ba_).eval()) * dt;
        // v to ba
        F_x_.block<3, 3>(6, 12) = -r_ * dt;
        // v to g
        F_x_.block<3, 3>(6, 15) = Eigen::Matrix3d::Identity() * dt;

        // theta to n_theta
        Q_x_.block<3, 3>(0, 0) = gyr_cov_ * dt * dt;
        // v to n_v
        Q_x_.block<3, 3>(6, 6) = r_ * acc_cov_ * r_.transpose() * dt * dt;
        // W_bg
        Q_x_.block<3, 3>(9, 9) = bias_gyr_cov_ * dt;
        // W_ba
        Q_x_.block<3, 3>(12, 12) = bias_acc_cov_ * dt;

        // std::cout << "Q: " << std::endl;
        // PrintMatrix(Q);

        // test demo
        // theta to theta
        // f_x_.block<3, 3>(0, 0) = -Skew((gyr_mean - bg_).eval());
        // // theta to bg
        // f_x_.block<3, 3>(0, 9) = -Eigen::Matrix3d::Identity();
        // // p to v
        // f_x_.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();
        // // v to theta
        // f_x_.block<3, 3>(6, 0) = -r_ * Skew((acc_mean - ba_).eval());
        // // v to ba
        // f_x_.block<3, 3>(6, 12) = -r_;
        // // v to g
        // if (states_dim_ > 15) {
        //   f_x_.block<3, 3>(6, 15) = Eigen::Matrix3d::Identity();
        // }
        // MatXd Phi;
        // Phi.resize(states_dim_, states_dim_);
        // Phi = (f_x_ * dt).exp();
        // cov_ = Phi * cov_ * Phi.transpose() + Q_x_;

        cov_ = F_x_ * cov_ * F_x_.transpose() + Q_x_;

        // std::cout << "predict cov: " << std::endl << state.cov << std::endl;
    }

    eular = Rot2Rpy(r_) * 180 / M_PI;
    PRINT_INFO("r end = [%.6f, %.6f, %.6f]\n", eular(0), eular(1), eular(2));
    PRINT_INFO("p end = [%.6f, %.6f, %.6f]\n", p_(0), p_(1), p_(2));
    PRINT_INFO("v end = [%.6f, %.6f, %.6f]\n", r_(0), r_(1), r_(2));

    current_time_ = imu_data.rbegin()->first;

    F_x_.setIdentity();
    Q_x_.setZero();
    f_x_.setZero();
    PRINT_INFO("ESKF predict end\n");

    return true;
}

void ESKF::PrintStates(const std::string& str_states) {
    Vec3d euler = Rot2Rpy(r_) * 57.3;  // Convert to degrees
    PRINT_INFO("Time: %.6f, %s\n", current_time_, str_states.c_str());
    PRINT_INFO("  Attitude (deg): [%.6f, %.6f, %.6f]\n", euler(0), euler(1), euler(2));
    PRINT_INFO("  Position (m): [%.6f, %.6f, %.6f]\n", p_(0), p_(1), p_(2));
    PRINT_INFO("  Velocity (m/s): [%.6f, %.6f, %.6f]\n", v_(0), v_(1), v_(2));
    PRINT_INFO("  Gyro Bias (rad/s): [%.8f, %.8f, %.8f]\n", bg_(0), bg_(1), bg_(2));
    PRINT_INFO("  Accel Bias (m/s^2): [%.8f, %.8f, %.8f]\n", ba_(0), ba_(1), ba_(2));
    PRINT_INFO("  Gravity (m/s^2): [%.4f, %.4f, %.4f]\n", g_(0), g_(1), g_(2));
}

void ESKF::PrintDiff(const Mat3d& r_truth, const Vec3d& p_truth) {
    Vec3d p_offset = p_truth - p_;
    Vec3d euler = Rot2Rpy(r_) * 180 / M_PI;
    Vec3d euler_truth = Rot2Rpy(r_truth) * 180 / M_PI;
    Vec3d r_offset = euler_truth - euler;

    PRINT_INFO("Time: %.6f, State Differences:\n", current_time_);
    PRINT_INFO("  Position Error (m): [%.6f, %.6f, %.6f]\n", p_offset(0), p_offset(1), p_offset(2));
    PRINT_INFO("  Attitude Error (deg): [%.6f, %.6f, %.6f]\n", r_offset(0), r_offset(1),
               r_offset(2));
}

void ESKF::GetDiff(const Mat3d& r_truth, const Vec3d& p_truth, Vec3d& p_offset, Vec3d& r_offset) {
    p_offset = (p_truth - p_) * 100;  // Convert to centimeters
    Vec3d euler = Rot2Rpy(r_) * 180 / M_PI;
    Vec3d euler_truth = Rot2Rpy(r_truth) * 180 / M_PI;
    r_offset = euler_truth - euler;
}

void ESKF::UpdateStates() {
    // Update rotation
    r_ = r_ * Exp(delta_x_.block<3, 1>(0, 0).eval());

    // Update position, velocity, biases and gravity
    p_ += delta_x_.block<3, 1>(3, 0);
    v_ += delta_x_.block<3, 1>(6, 0);
    bg_ += delta_x_.block<3, 1>(9, 0);
    ba_ += delta_x_.block<3, 1>(12, 0);
    g_ += delta_x_.block<3, 1>(15, 0);

    // Project covariance
    ProjectCov();

    // Reset delta state
    delta_x_.setZero();
}

void ESKF::ProjectCov() {
    // Update projection matrix
    pro_j_.block<3, 3>(0, 0) = Mat3d::Identity() - 0.5 * Skew(delta_x_.block<3, 1>(0, 0).eval());

    // Update covariance
    cov_ = pro_j_ * cov_ * pro_j_.transpose();
}

void ESKF::CalKalmanGainAndCov(const MatXd& H, const MatXd& R, const VecXd& z) {
    // Calculate Kalman gain
    auto K = cov_ * H.transpose() * (H * cov_ * H.transpose() + R).inverse();

    // Update state increment
    delta_x_ = K * z;

    // Print state increment
    PRINT_INFO("State Increment:\n");
    PRINT_INFO("  Attitude: [%.6f, %.6f, %.6f]\n", delta_x_(0), delta_x_(1), delta_x_(2));
    PRINT_INFO("  Position: [%.6f, %.6f, %.6f]\n", delta_x_(3), delta_x_(4), delta_x_(5));
    PRINT_INFO("  Velocity: [%.6f, %.6f, %.6f]\n", delta_x_(6), delta_x_(7), delta_x_(8));
    PRINT_INFO("  Gyro Bias: [%.6f, %.6f, %.6f]\n", delta_x_(9), delta_x_(10), delta_x_(11));
    PRINT_INFO("  Accel Bias: [%.6f, %.6f, %.6f]\n", delta_x_(12), delta_x_(13), delta_x_(14));
    PRINT_INFO("  Gravity: [%.4f, %.4f, %.4f]\n", delta_x_(15), delta_x_(16), delta_x_(17));

    // Update covariance
    cov_ = (I_matrix_ - K * H) * cov_;
}