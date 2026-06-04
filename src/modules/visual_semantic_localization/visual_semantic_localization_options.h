#ifndef VISUAL_SEMANTIC_LOCALIZATION_OPTIONS_H
#define VISUAL_SEMANTIC_LOCALIZATION_OPTIONS_H

#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "common_utils/opencv_yaml_parse.h"
#include "common_utils/print.h"
#include "common_utils/utils.h"

struct VisualSemanticLocalizationOptions {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 配置文件路径结构体
    struct ConfigPaths {
        std::string root_path;
        std::string camera_intrinsics_file;
        std::string extrinsics_backlidar_imu_file;
        std::string extrinsics_camera_backlidar_file;
        std::string extrinsics_frontlidar_backlidar_file;
    } config;

    // 数据路径结构体
    struct DataPaths {
        std::string root_path;
        std::string raw_image_data_folder_path;
        std::string semantic_data_folder_path;
        std::string raw_imu_data_file_path;
        std::string ground_truth_data_file_path;
        std::string semantic_pointcloud_map_folder_path;
        std::string semantic_contours_file_path;
    } data;

    // 输出路径结构体
    struct OutputPaths {
        std::string debug_output_path;
        std::string result_path;
        std::string log_path;
    } output;

    // 相机参数结构体
    struct CameraParameters {
        int width = 1920;
        int height = 1536;
        Eigen::Matrix3d intrinsics_K = Eigen::Matrix3d::Identity();
        std::vector<double> distortion_coeffs;
        double fx, fy, cx, cy;
        double k1, k2, p1, p2, k3;
    } camera;

    // 外参结构体
    struct ExtrinsicParameters {
        Eigen::Matrix4d T_backlidar_camera = Eigen::Matrix4d::Identity();

        Eigen::Matrix4d T_backlidar_frontlidar = Eigen::Matrix4d::Identity();

        Eigen::Matrix4d T_imu_backlidar = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d r_l_to_i = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_l_in_i = Eigen::Vector3d::Zero();

        Eigen::Matrix4d T_backlidar_imu = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d r_i_to_l_ = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_i_in_l_ = Eigen::Vector3d::Zero();

        Eigen::Matrix4d T_camera_imu = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d r_i_to_c_ = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_i_in_c_ = Eigen::Vector3d::Zero();

    } extrinsics;

    // IMU 参数配置
    struct IMUParameters {
        double freq = 0.01;       // IMU 采样频率
        double gravity = 9.8012;  // 重力值

        // 传感器噪声标准差参数
        struct SensorNoiseStdParameters {
            double acc_noise_std_x = 9.8e-3;
            double acc_noise_std_y = 9.8e-3;
            double acc_noise_std_z = 9.8e-3;
            double gyro_noise_std_x = 1.9e-4;
            double gyro_noise_std_y = 1.9e-4;
            double gyro_noise_std_z = 4.8481e-5;
        } sensor_noise_std;

        // 偏置随机游走标准差参数
        struct BiasRandomWalkStdParameters {
            double acc_bias_std_x = 6.6667e-7;
            double acc_bias_std_y = 6.6667e-7;
            double acc_bias_std_z = 6.6667e-7;
            double gyro_bias_std_x = 1.0167e-7;
            double gyro_bias_std_y = 1.0167e-7;
            double gyro_bias_std_z = 1.0167e-8;
        } bias_random_walk_std;

        // 初始标准差参数
        struct InitialStdParameters {
            double pos_init_std = 1e-2;
            double vel_init_std = 1e-3;
            double rot_init_std = 1e-2;
            double gyro_bias_init_std = 1e-4;
            double acc_bias_init_std = 1e-4;
            double gravity_init_std = 1e-3;
        } initial_std;
    } imu;

    // 图像参数配置
    struct ImageParameters {
        bool is_need_distortion = false;  // 是否需要畸变校正
        bool is_need_downsample = true;   // 是否需要下采样
        int downsample_factor = 3;        // 下采样因子
    } image;

    // 检查YAML文件是否有效
    static bool CheckYamlFile(const std::string& yaml_file, const std::string& context) {
        if (yaml_file.empty()) {
            PRINT_ERROR("YAML file path is empty for %s\n", context.c_str());
            return false;
        }

        if (access(yaml_file.c_str(), R_OK) != 0) {
            PRINT_ERROR("YAML file does not exist or cannot be read for %s: %s\n", context.c_str(),
                        yaml_file.c_str());
            return false;
        }

        std::ifstream file(yaml_file);
        if (file.peek() == std::ifstream::traits_type::eof()) {
            PRINT_ERROR("YAML file is empty for %s: %s\n", context.c_str(), yaml_file.c_str());
            return false;
        }

        return true;
    }

    // 完整路径
    static std::string GetFullPath(const std::string& root, const std::string& rel) {
        if (rel.empty() || rel[0] == '/') return rel;
        if (root.empty()) return rel;
        if (root.back() == '/')
            return root + rel;
        else
            return root + "/" + rel;
    }

    // 四元数转旋转矩阵
    static Eigen::Matrix3d QuaternionToRotationMatrix(double x, double y, double z, double w) {
        Eigen::Quaterniond q(w, x, y, z);
        q.normalize();
        return q.toRotationMatrix();
    }

    // 手动解析外参YAML文件的辅助函数
    static bool LoadExtrinsicsFromYaml(const std::string& yaml_file, Eigen::Matrix4d& T) {
        if (access(yaml_file.c_str(), R_OK) != 0) {
            PRINT_ERROR("Extrinsic file does not exist or cannot be read: %s\n", yaml_file.c_str());
            return false;
        }

        try {
            YAML::Node node = YAML::LoadFile(yaml_file);

            if (node["transform"] && node["transform"]["translation"] &&
                node["transform"]["rotation"]) {
                double qw = node["transform"]["rotation"]["w"].as<double>();
                double qx = node["transform"]["rotation"]["x"].as<double>();
                double qy = node["transform"]["rotation"]["y"].as<double>();
                double qz = node["transform"]["rotation"]["z"].as<double>();

                double tx = node["transform"]["translation"]["x"].as<double>();
                double ty = node["transform"]["translation"]["y"].as<double>();
                double tz = node["transform"]["translation"]["z"].as<double>();

                Eigen::Matrix3d R = QuaternionToRotationMatrix(qx, qy, qz, qw);
                T.block<3, 3>(0, 0) = R;
                T.block<3, 1>(0, 3) << tx, ty, tz;
                T(3, 0) = T(3, 1) = T(3, 2) = 0.0;
                T(3, 3) = 1.0;

                return true;
            } else {
                PRINT_ERROR("Extrinsic file format error, missing transform node: %s\n",
                            yaml_file.c_str());
                return false;
            }
        } catch (const YAML::Exception& e) {
            PRINT_ERROR("Failed to parse extrinsic file: %s, Error: %s\n", yaml_file.c_str(),
                        e.what());
            return false;
        }
    }

    // 手动解析相机内参YAML文件的辅助函数
    static bool LoadCameraIntrinsicsFromYaml(const std::string& yaml_file, int& image_width,
                                             int& image_height,
                                             Eigen::Matrix3d& camera_intrinsics_K,
                                             std::vector<double>& distortion_coeffs) {
        if (access(yaml_file.c_str(), R_OK) != 0) {
            PRINT_ERROR("Camera intrinsics file does not exist or cannot be read: %s\n",
                        yaml_file.c_str());
            return false;
        }

        try {
            YAML::Node node = YAML::LoadFile(yaml_file);

            if (node["width"]) {
                image_width = node["width"].as<int>();
            }
            if (node["height"]) {
                image_height = node["height"].as<int>();
            }

            if (node["K"] && node["K"].IsSequence() && node["K"].size() == 9) {
                std::vector<double> K_vec;
                for (size_t i = 0; i < 9; ++i) {
                    K_vec.push_back(node["K"][i].as<double>());
                }
                camera_intrinsics_K << K_vec[0], K_vec[1], K_vec[2], K_vec[3], K_vec[4], K_vec[5],
                    K_vec[6], K_vec[7], K_vec[8];
            }

            if (node["D"] && node["D"].IsSequence()) {
                distortion_coeffs.clear();
                for (size_t i = 0; i < node["D"].size(); ++i) {
                    distortion_coeffs.push_back(node["D"][i].as<double>());
                }
            }

            return true;
        } catch (const YAML::Exception& e) {
            PRINT_ERROR("Failed to parse camera intrinsics file: %s, Error: %s\n",
                        yaml_file.c_str(), e.what());
            return false;
        }
    }

    // 从YAML文件加载配置
    static bool LoadFromYaml(const std::string& yaml_file,
                             VisualSemanticLocalizationOptions& options) {
        if (!CheckYamlFile(yaml_file, "main configuration")) {
            return false;
        }

        try {
            YamlParser::Ptr parser(new YamlParser(yaml_file));

            // 读取config部分
            parser->ParseConfig("config.root_path", options.config.root_path);
            parser->ParseConfig("config.camera_intrinsics", options.config.camera_intrinsics_file);
            parser->ParseConfig("config.extrinsics_backlidar_imu",
                                options.config.extrinsics_backlidar_imu_file);
            parser->ParseConfig("config.extrinsics_camera_backlidar",
                                options.config.extrinsics_camera_backlidar_file);
            parser->ParseConfig("config.extrinsics_frontlidar_backlidar",
                                options.config.extrinsics_frontlidar_backlidar_file);

            // 读取IMU参数
            parser->ParseConfig("imu.freq", options.imu.freq);
            parser->ParseConfig("imu.gravity", options.imu.gravity);

            // IMU 噪声参数
            parser->ParseConfig("imu.sensor_noise_std.acc_noise_std_x",
                                options.imu.sensor_noise_std.acc_noise_std_x);
            parser->ParseConfig("imu.sensor_noise_std.acc_noise_std_y",
                                options.imu.sensor_noise_std.acc_noise_std_y);
            parser->ParseConfig("imu.sensor_noise_std.acc_noise_std_z",
                                options.imu.sensor_noise_std.acc_noise_std_z);
            parser->ParseConfig("imu.sensor_noise_std.gyro_noise_std_x",
                                options.imu.sensor_noise_std.gyro_noise_std_x);
            parser->ParseConfig("imu.sensor_noise_std.gyro_noise_std_y",
                                options.imu.sensor_noise_std.gyro_noise_std_y);
            parser->ParseConfig("imu.sensor_noise_std.gyro_noise_std_z",
                                options.imu.sensor_noise_std.gyro_noise_std_z);

            // IMU 偏置随机游走参数
            parser->ParseConfig("imu.bias_random_walk_std.acc_bias_std_x",
                                options.imu.bias_random_walk_std.acc_bias_std_x);
            parser->ParseConfig("imu.bias_random_walk_std.acc_bias_std_y",
                                options.imu.bias_random_walk_std.acc_bias_std_y);
            parser->ParseConfig("imu.bias_random_walk_std.acc_bias_std_z",
                                options.imu.bias_random_walk_std.acc_bias_std_z);
            parser->ParseConfig("imu.bias_random_walk_std.gyro_bias_std_x",
                                options.imu.bias_random_walk_std.gyro_bias_std_x);
            parser->ParseConfig("imu.bias_random_walk_std.gyro_bias_std_y",
                                options.imu.bias_random_walk_std.gyro_bias_std_y);
            parser->ParseConfig("imu.bias_random_walk_std.gyro_bias_std_z",
                                options.imu.bias_random_walk_std.gyro_bias_std_z);

            // IMU 初始标准差参数
            parser->ParseConfig("imu.initial_std.pos_init_std",
                                options.imu.initial_std.pos_init_std);
            parser->ParseConfig("imu.initial_std.vel_init_std",
                                options.imu.initial_std.vel_init_std);
            parser->ParseConfig("imu.initial_std.rot_init_std",
                                options.imu.initial_std.rot_init_std);
            parser->ParseConfig("imu.initial_std.gyro_bias_init_std",
                                options.imu.initial_std.gyro_bias_init_std);
            parser->ParseConfig("imu.initial_std.acc_bias_init_std",
                                options.imu.initial_std.acc_bias_init_std);
            parser->ParseConfig("imu.initial_std.gravity_init_std",
                                options.imu.initial_std.gravity_init_std);

            // 读取图像参数
            parser->ParseConfig("image.is_need_distortion", options.image.is_need_distortion);
            parser->ParseConfig("image.is_need_downsample", options.image.is_need_downsample);
            parser->ParseConfig("image.downsample_factor", options.image.downsample_factor);

            // 读取output部分
            parser->ParseConfig("output.debug_output_path", options.output.debug_output_path);
            parser->ParseConfig("output.result_path", options.output.result_path);
            parser->ParseConfig("output.log_path", options.output.log_path);

            // 读取data部分
            std::string relative_root_path;
            parser->ParseConfig("data.root_path", relative_root_path);
            options.data.root_path = GetFullPath("", relative_root_path);

            std::string relative_path;
            parser->ParseConfig("data.raw_image_data_folder", relative_path);
            options.data.raw_image_data_folder_path =
                GetFullPath(options.data.root_path, relative_path);

            parser->ParseConfig("data.semantic_data_folder", relative_path);
            options.data.semantic_data_folder_path =
                GetFullPath(options.data.root_path, relative_path);

            parser->ParseConfig("data.raw_imu_data_file", relative_path);
            options.data.raw_imu_data_file_path =
                GetFullPath(options.data.root_path, relative_path);

            parser->ParseConfig("data.ground_truth_data_file", relative_path);
            options.data.ground_truth_data_file_path =
                GetFullPath(options.data.root_path, relative_path);

            parser->ParseConfig("data.semantic_pointcloud_map_folder", relative_path);
            options.data.semantic_pointcloud_map_folder_path =
                GetFullPath(options.data.root_path, relative_path);

            parser->ParseConfig("data.semantic_contours_file", relative_path);
            options.data.semantic_contours_file_path =
                GetFullPath(options.data.root_path, relative_path);

            // 读取相机内参
            std::string camera_intrinsics_path =
                GetFullPath(options.config.root_path, options.config.camera_intrinsics_file);
            if (!LoadCameraIntrinsicsFromYaml(camera_intrinsics_path, options.camera.width,
                                              options.camera.height, options.camera.intrinsics_K,
                                              options.camera.distortion_coeffs)) {
                PRINT_ERROR("Failed to load camera intrinsics, using default values\n");
            }

            // 设置相机参数
            options.camera.fx = options.camera.intrinsics_K(0, 0);
            options.camera.fy = options.camera.intrinsics_K(1, 1);
            options.camera.cx = options.camera.intrinsics_K(0, 2);
            options.camera.cy = options.camera.intrinsics_K(1, 2);

            if (options.camera.distortion_coeffs.size() >= 5) {
                options.camera.k1 = options.camera.distortion_coeffs[0];
                options.camera.k2 = options.camera.distortion_coeffs[1];
                options.camera.p1 = options.camera.distortion_coeffs[2];
                options.camera.p2 = options.camera.distortion_coeffs[3];
                options.camera.k3 = options.camera.distortion_coeffs[4];
            }

            // 读取外参
            // T_backlidar_camera
            std::string extrinsics_camera_backlidar_path = GetFullPath(
                options.config.root_path, options.config.extrinsics_camera_backlidar_file);
            if (!LoadExtrinsicsFromYaml(extrinsics_camera_backlidar_path,
                                        options.extrinsics.T_backlidar_camera)) {
                PRINT_ERROR(
                    "Failed to load camera to backlidar extrinsics, using identity matrix\n");
                options.extrinsics.T_backlidar_camera = Eigen::Matrix4d::Identity();
            }

            // T_backlidar_frontlidar
            std::string extrinsics_frontlidar_backlidar_path = GetFullPath(
                options.config.root_path, options.config.extrinsics_frontlidar_backlidar_file);
            if (!LoadExtrinsicsFromYaml(extrinsics_frontlidar_backlidar_path,
                                        options.extrinsics.T_backlidar_frontlidar)) {
                PRINT_ERROR(
                    "Failed to load frontlidar to backlidar extrinsics, using identity matrix\n");
                options.extrinsics.T_backlidar_frontlidar = Eigen::Matrix4d::Identity();
            }

            // T_imu_backlidar
            std::string extrinsics_backlidar_imu_path =
                GetFullPath(options.config.root_path, options.config.extrinsics_backlidar_imu_file);
            if (!LoadExtrinsicsFromYaml(extrinsics_backlidar_imu_path,
                                        options.extrinsics.T_imu_backlidar)) {
                PRINT_ERROR("Failed to load backlidar to IMU extrinsics, using identity matrix\n");
                options.extrinsics.T_imu_backlidar = Eigen::Matrix4d::Identity();
            }
            options.extrinsics.r_l_to_i = options.extrinsics.T_imu_backlidar.block<3, 3>(0, 0);
            options.extrinsics.t_l_in_i = options.extrinsics.T_imu_backlidar.block<3, 1>(0, 3);

            // T_backlidar_imu
            options.extrinsics.T_backlidar_imu = options.extrinsics.T_imu_backlidar.inverse();
            options.extrinsics.r_i_to_l_ = options.extrinsics.T_backlidar_imu.block<3, 3>(0, 0);
            options.extrinsics.t_i_in_l_ = options.extrinsics.T_backlidar_imu.block<3, 1>(0, 3);

            // T_camera_imu
            options.extrinsics.T_camera_imu = options.extrinsics.T_backlidar_camera.inverse() *
                                              options.extrinsics.T_imu_backlidar.inverse();
            options.extrinsics.r_i_to_c_ = options.extrinsics.T_camera_imu.block<3, 3>(0, 0);
            options.extrinsics.t_i_in_c_ = options.extrinsics.T_camera_imu.block<3, 1>(0, 3);

            return true;
        } catch (const std::exception& e) {
            PRINT_ERROR("Failed to load configuration: %s\n", e.what());
            return false;
        }
    }

    // 打印配置信息
    void Print() const {
        PRINT_INFO("====== Configuration File Paths ======\n");
        PRINT_INFO("Config Root Path: %s\n", config.root_path.c_str());
        PRINT_INFO("Camera Intrinsics File: %s\n", config.camera_intrinsics_file.c_str());
        PRINT_INFO("Extrinsics Backlidar-IMU File: %s\n",
                   config.extrinsics_backlidar_imu_file.c_str());
        PRINT_INFO("Extrinsics Camera-Backlidar File: %s\n",
                   config.extrinsics_camera_backlidar_file.c_str());
        PRINT_INFO("Extrinsics Frontlidar-Backlidar File: %s\n",
                   config.extrinsics_frontlidar_backlidar_file.c_str());

        PRINT_INFO("====== Data Paths ======\n");
        PRINT_INFO("Data Root Path: %s\n", data.root_path.c_str());
        PRINT_INFO("Raw Image Data Folder: %s\n", data.raw_image_data_folder_path.c_str());
        PRINT_INFO("Semantic Data Folder: %s\n", data.semantic_data_folder_path.c_str());
        PRINT_INFO("Raw IMU Data File: %s\n", data.raw_imu_data_file_path.c_str());
        PRINT_INFO("Ground Truth Data File: %s\n", data.ground_truth_data_file_path.c_str());
        PRINT_INFO("Semantic Pointcloud Map Folder: %s\n",
                   data.semantic_pointcloud_map_folder_path.c_str());
        PRINT_INFO("Semantic Contours File: %s\n", data.semantic_contours_file_path.c_str());

        PRINT_INFO("====== Output Paths ======\n");
        PRINT_INFO("Debug Output Path: %s\n", output.debug_output_path.c_str());
        PRINT_INFO("Result Path: %s\n", output.result_path.c_str());
        PRINT_INFO("Log Path: %s\n", output.log_path.c_str());

        PRINT_INFO("====== Camera Parameters ======\n");
        PRINT_INFO("Image Resolution: %d x %d\n", camera.width, camera.height);
        PRINT_INFO("Camera Intrinsics (fx, fy, cx, cy): %f, %f, %f, %f\n", camera.fx, camera.fy,
                   camera.cx, camera.cy);
        PRINT_INFO("Camera Intrinsics Matrix:\n%s\n",
                   EigenMatrixToString(camera.intrinsics_K).c_str());
        PRINT_INFO("Distortion Coefficients (k1, k2, p1, p2, k3): %f, %f, %f, %f, %f\n", camera.k1,
                   camera.k2, camera.p1, camera.p2, camera.k3);

        PRINT_INFO("====== Extrinsic Parameters ======\n");
        PRINT_INFO("T_backlidar_camera (Camera to Backlidar):\n%s\n",
                   EigenMatrixToString(extrinsics.T_backlidar_camera).c_str());
        PRINT_INFO("T_imu_backlidar (Backlidar to IMU):\n%s\n",
                   EigenMatrixToString(extrinsics.T_imu_backlidar).c_str());
        PRINT_INFO("T_backlidar_frontlidar (Frontlidar to Backlidar):\n%s\n",
                   EigenMatrixToString(extrinsics.T_backlidar_frontlidar).c_str());

        PRINT_INFO("====== IMU Parameters ======\n");
        PRINT_INFO("IMU Frequency: %f Hz\n", 1.0 / imu.freq);
        PRINT_INFO("Gravity: %f m/s^2\n", imu.gravity);
        PRINT_INFO("IMU Noise Standard Deviation:\n");
        PRINT_INFO("  Accelerometer: x=%e, y=%e, z=%e\n", imu.sensor_noise_std.acc_noise_std_x,
                   imu.sensor_noise_std.acc_noise_std_y, imu.sensor_noise_std.acc_noise_std_z);
        PRINT_INFO("  Gyroscope: x=%e, y=%e, z=%e\n", imu.sensor_noise_std.gyro_noise_std_x,
                   imu.sensor_noise_std.gyro_noise_std_y, imu.sensor_noise_std.gyro_noise_std_z);
        PRINT_INFO("IMU Bias Random Walk Standard Deviation:\n");
        PRINT_INFO("  Accelerometer Bias: x=%e, y=%e, z=%e\n",
                   imu.bias_random_walk_std.acc_bias_std_x, imu.bias_random_walk_std.acc_bias_std_y,
                   imu.bias_random_walk_std.acc_bias_std_z);
        PRINT_INFO("  Gyroscope Bias: x=%e, y=%e, z=%e\n", imu.bias_random_walk_std.gyro_bias_std_x,
                   imu.bias_random_walk_std.gyro_bias_std_y,
                   imu.bias_random_walk_std.gyro_bias_std_z);
        PRINT_INFO("IMU Initial Standard Deviation:\n");
        PRINT_INFO("  Position: %e, Velocity: %e, Rotation: %e\n", imu.initial_std.pos_init_std,
                   imu.initial_std.vel_init_std, imu.initial_std.rot_init_std);
        PRINT_INFO("  Gyro Bias: %e, Accel Bias: %e, Gravity: %e\n",
                   imu.initial_std.gyro_bias_init_std, imu.initial_std.acc_bias_init_std,
                   imu.initial_std.gravity_init_std);

        PRINT_INFO("====== Image Parameters ======\n");
        PRINT_INFO("Need Distortion Correction: %s\n", image.is_need_distortion ? "true" : "false");
        PRINT_INFO("Need Downsample: %s\n", image.is_need_downsample ? "true" : "false");
        PRINT_INFO("Downsample Factor: %d\n", image.downsample_factor);
    }
};

#endif  // VISUAL_SEMANTIC_LOCALIZATION_OPTIONS_H
