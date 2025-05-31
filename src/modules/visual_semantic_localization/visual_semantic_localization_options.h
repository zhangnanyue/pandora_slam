#ifndef VISUAL_SEMANTIC_LOCALIZATION_OPTIONS_H
#define VISUAL_SEMANTIC_LOCALIZATION_OPTIONS_H

#include "common_utils/opencv_yaml_parse.h"
#include "common_utils/print.h"
#include "common_utils/utils.h"
#include "common_utils/eigen_types.h"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <fstream>
#include <unistd.h>

struct VisualSemanticLocalizationOptions {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 传感器参数配置文件路径
  std::string config_root_path;
  std::string camera_intrinsics_file;
  std::string extrinsics_backlidar_imu_file;
  std::string extrinsics_camera_backlidar_file;
  std::string extrinsics_frontlidar_backlidar_file;

  // 数据路径
  std::string data_root_path;
  std::string raw_image_data_folder_path;
  std::string semantic_data_folder_path;
  std::string raw_imu_data_file_path;
  std::string ground_truth_data_file_path;
  std::string raw_lidar_data_folder_path;
  std::string semantic_contours_file_path;

  // 输出路径
  std::string debug_output_path;
  std::string result_path;
  std::string log_path;

  // 通用参数
  int downsample_factor = 1;

  // 相机参数
  int image_width = 1920;
  int image_height = 1536;
  Eigen::Matrix3d camera_intrinsics_K = Eigen::Matrix3d::Identity();
  std::vector<double> distortion_coeffs;
  double fx, fy, cx, cy;
  double k1, k2, p1, p2, k3;

  // 外参 - 使用四元数和平移向量表示的变换
  Eigen::Matrix4d T_backlidar_camera = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_imu_backlidar = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_backlidar_frontlidar = Eigen::Matrix4d::Identity();

  // 检查YAML文件是否有效
  bool CheckYamlFile(const std::string& yaml_file, const std::string& context) {
    // 检查yaml文件是否存在
    if (yaml_file.empty()) {
      PRINT_ERROR("YAML file path is empty for %s\n", context.c_str());
      return false;
    }

    // 检查文件是否存在和可读
    if (access(yaml_file.c_str(), R_OK) != 0) {
      PRINT_ERROR("YAML file does not exist or cannot be read for %s: %s\n", 
                 context.c_str(), yaml_file.c_str());
      return false;
    }

    // 检查文件是否为空
    std::ifstream file(yaml_file);
    if (file.peek() == std::ifstream::traits_type::eof()) {
      PRINT_ERROR("YAML file is empty for %s: %s\n", 
                 context.c_str(), yaml_file.c_str());
      return false;
    }

    return true;
  }

  // 完整路径
  std::string GetFullPath(const std::string& root, const std::string& rel) const {
    if (rel.empty() || rel[0] == '/') return rel;
    if (root.empty()) return rel;
    if (root.back() == '/')
      return root + rel;
    else
      return root + "/" + rel;
  }

  // 四元数转旋转矩阵
  Eigen::Matrix3d QuaternionToRotationMatrix(double x, double y, double z, double w) const {
    Eigen::Quaterniond q(w, x, y, z);
    q.normalize();
    return q.toRotationMatrix();
  }

  // 手动解析外参YAML文件的辅助函数
  bool LoadExtrinsicsFromYaml(const std::string& yaml_file, Eigen::Matrix4d& T) const {
    // 检查文件是否存在和可读
    if (access(yaml_file.c_str(), R_OK) != 0) {
      PRINT_ERROR("Extrinsic file does not exist or cannot be read: %s\n", yaml_file.c_str());
      return false;
    }

    try {
      YAML::Node node = YAML::LoadFile(yaml_file);
      
      // 检查是否有transform节点
      if (node["transform"] && node["transform"]["translation"] && node["transform"]["rotation"]) {
        // 读取四元数
        double qw = node["transform"]["rotation"]["w"].as<double>();
        double qx = node["transform"]["rotation"]["x"].as<double>();
        double qy = node["transform"]["rotation"]["y"].as<double>();
        double qz = node["transform"]["rotation"]["z"].as<double>();
        
        // 读取平移向量
        double tx = node["transform"]["translation"]["x"].as<double>();
        double ty = node["transform"]["translation"]["y"].as<double>();
        double tz = node["transform"]["translation"]["z"].as<double>();
        
        // 构造变换矩阵
        Eigen::Matrix3d R = QuaternionToRotationMatrix(qx, qy, qz, qw);
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) << tx, ty, tz;
        T(3,0) = T(3,1) = T(3,2) = 0.0;
        T(3,3) = 1.0;
        
        return true;
      } else {
        PRINT_ERROR("Extrinsic file format error, missing transform node: %s\n", yaml_file.c_str());
        return false;
      }
    } catch (const YAML::Exception& e) {
      PRINT_ERROR("Failed to parse extrinsic file: %s, Error: %s\n", yaml_file.c_str(), e.what());
      return false;
    }
  }

  // 手动解析相机内参YAML文件的辅助函数
  bool LoadCameraIntrinsicsFromYaml(const std::string& yaml_file) {
    // 检查文件是否存在和可读
    if (access(yaml_file.c_str(), R_OK) != 0) {
      PRINT_ERROR("Camera intrinsics file does not exist or cannot be read: %s\n", yaml_file.c_str());
      return false;
    }

    try {
      YAML::Node node = YAML::LoadFile(yaml_file);
      
      // 读取图像尺寸
      if (node["width"]) {
        image_width = node["width"].as<int>();
      }
      if (node["height"]) {
        image_height = node["height"].as<int>();
      }
      
      // 读取内参矩阵K (9个元素的数组)
      if (node["K"] && node["K"].IsSequence() && node["K"].size() == 9) {
        std::vector<double> K_vec;
        for (size_t i = 0; i < 9; ++i) {
          K_vec.push_back(node["K"][i].as<double>());
        }
        camera_intrinsics_K << K_vec[0], K_vec[1], K_vec[2],
                               K_vec[3], K_vec[4], K_vec[5],
                               K_vec[6], K_vec[7], K_vec[8];
      }
      
      // 读取畸变参数D
      if (node["D"] && node["D"].IsSequence()) {
        distortion_coeffs.clear();
        for (size_t i = 0; i < node["D"].size(); ++i) {
          distortion_coeffs.push_back(node["D"][i].as<double>());
        }
      }
      
      return true;
    } catch (const YAML::Exception& e) {
      PRINT_ERROR("Failed to parse camera intrinsics file: %s, Error: %s\n", yaml_file.c_str(), e.what());
      return false;
    }
  }

  // 读取yaml配置
  void LoadSensorParamsAndPrint(const std::string& yaml_file) {
    PRINT_INFO("====== Loading and printing sensor parameters ======\n");

    if (!CheckYamlFile(yaml_file, "sensor parameters")) {
      return;
    }

    YamlParser::Ptr parser(new YamlParser(yaml_file));

    // 读取config部分
    parser->ParseConfig("config.root_path", config_root_path);
    parser->ParseConfig("config.camera_intrinsics", camera_intrinsics_file);
    parser->ParseConfig("config.extrinsics_backlidar_imu", extrinsics_backlidar_imu_file);
    parser->ParseConfig("config.extrinsics_camera_backlidar", extrinsics_camera_backlidar_file);
    parser->ParseConfig("config.extrinsics_frontlidar_backlidar", extrinsics_frontlidar_backlidar_file);

    // 读取common部分
    parser->ParseConfig("common.downsample_factor", downsample_factor);

    // 打印基本配置信息
    PRINT_DEBUG("====== Sensor Configuration Parameters ======\n");
    PRINT_DEBUG("Config Root Path: %s\n", config_root_path.c_str());
    PRINT_DEBUG("Camera Intrinsics File: %s\n", camera_intrinsics_file.c_str());
    PRINT_DEBUG("Extrinsics Backlidar-IMU File: %s\n", extrinsics_backlidar_imu_file.c_str());
    PRINT_DEBUG("Extrinsics Camera-Backlidar File: %s\n", extrinsics_camera_backlidar_file.c_str());
    PRINT_DEBUG("Extrinsics Frontlidar-Backlidar File: %s\n", extrinsics_frontlidar_backlidar_file.c_str());

    // 读取相机内参
    std::string camera_intrinsics_path = GetFullPath(config_root_path, camera_intrinsics_file);
    PRINT_DEBUG("====== Loading Camera Intrinsics ======\n");
    PRINT_DEBUG("Camera Intrinsics File Path: %s\n", camera_intrinsics_path.c_str());
    
    // 使用手动解析函数替代OpenCV解析，避免格式问题
    if (!LoadCameraIntrinsicsFromYaml(camera_intrinsics_path)) {
      PRINT_ERROR("Failed to load camera intrinsics, using default values\n");
      // 如果解析失败，保持默认值
    }

    // 应用下采样因子
    fx = camera_intrinsics_K(0, 0);
    fy = camera_intrinsics_K(1, 1);
    cx = camera_intrinsics_K(0, 2);
    cy = camera_intrinsics_K(1, 2);

    if (distortion_coeffs.size() >= 5) {
      k1 = distortion_coeffs[0];
      k2 = distortion_coeffs[1];
      p1 = distortion_coeffs[2];
      p2 = distortion_coeffs[3];
      k3 = distortion_coeffs[4];
    }

    // 打印相机参数
    PRINT_DEBUG("Original Resolution: %d x %d\n", image_width, image_height);
    PRINT_DEBUG("Camera Intrinsics (fx, fy, cx, cy): %f, %f, %f, %f\n", fx, fy, cx, cy);
    PRINT_DEBUG("Camera Intrinsics Matrix:\n%s\n", EigenMatrixToString(camera_intrinsics_K).c_str());
    PRINT_DEBUG("Distortion Coefficients (k1, k2, p1, p2, k3): %f, %f, %f, %f, %f\n", k1, k2, p1, p2, k3);

    PRINT_DEBUG("====== Loading Extrinsic Parameters ======\n");

    // 读取相机到后激光雷达外参
    std::string extrinsics_camera_backlidar_path = GetFullPath(config_root_path, extrinsics_camera_backlidar_file);
    PRINT_DEBUG("Camera to Backlidar Extrinsic File Path: %s\n", extrinsics_camera_backlidar_path.c_str());
    
    // 使用手动解析函数
    if (!LoadExtrinsicsFromYaml(extrinsics_camera_backlidar_path, T_backlidar_camera)) {
      PRINT_ERROR("Failed to load camera to backlidar extrinsics, using identity matrix\n");
      T_backlidar_camera = Eigen::Matrix4d::Identity();
    }
    
    PRINT_DEBUG("T_backlidar_camera (Camera to Backlidar):\n%s\n", EigenMatrixToString(T_backlidar_camera).c_str());

    // 读取后激光雷达到IMU外参
    std::string extrinsics_backlidar_imu_path = GetFullPath(config_root_path, extrinsics_backlidar_imu_file);
    PRINT_DEBUG("Backlidar to IMU Extrinsic File Path: %s\n", extrinsics_backlidar_imu_path.c_str());
    
    // 使用手动解析函数
    if (!LoadExtrinsicsFromYaml(extrinsics_backlidar_imu_path, T_imu_backlidar)) {
      PRINT_ERROR("Failed to load backlidar to IMU extrinsics, using identity matrix\n");
      T_imu_backlidar = Eigen::Matrix4d::Identity();
    }
    
    PRINT_DEBUG("T_imu_backlidar (Backlidar to IMU):\n%s\n", EigenMatrixToString(T_imu_backlidar).c_str());

    // 读取前激光雷达到后激光雷达外参
    std::string extrinsics_frontlidar_backlidar_path = GetFullPath(config_root_path, extrinsics_frontlidar_backlidar_file);
    PRINT_DEBUG("Frontlidar to Backlidar Extrinsic File Path: %s\n", extrinsics_frontlidar_backlidar_path.c_str());
    
    // 使用手动解析函数
    if (!LoadExtrinsicsFromYaml(extrinsics_frontlidar_backlidar_path, T_backlidar_frontlidar)) {
      PRINT_ERROR("Failed to load frontlidar to backlidar extrinsics, using identity matrix\n");
      T_backlidar_frontlidar = Eigen::Matrix4d::Identity();
    }
    
    PRINT_DEBUG("T_backlidar_frontlidar (Frontlidar to Backlidar):\n%s\n", EigenMatrixToString(T_backlidar_frontlidar).c_str());

    // PRINT_DEBUG("====== 数据路径配置 ======\n");
    // PRINT_DEBUG("data_root_path: %s\n", data_root_path.c_str());
    // PRINT_DEBUG("raw_image_data_folder: %s\n", raw_image_data_folder.c_str());
    // PRINT_DEBUG("semantic_data_folder: %s\n", semantic_data_folder.c_str());
    // PRINT_DEBUG("raw_imu_data_file: %s\n", raw_imu_data_file.c_str());
    // PRINT_DEBUG("ground_truth_data_file: %s\n", ground_truth_data_file.c_str());
    // PRINT_DEBUG("raw_lidar_data_folder: %s\n", raw_lidar_data_folder.c_str());
    // PRINT_DEBUG("processed_data_folder: %s\n", processed_data_folder.c_str());

    // PRINT_DEBUG("====== 输出路径配置 ======\n");
    // PRINT_DEBUG("debug_output_path: %s\n", debug_output_path.c_str());
    // PRINT_DEBUG("result_path: %s\n", result_path.c_str());
    // PRINT_DEBUG("log_path: %s\n", log_path.c_str());
  }

  void LoadDataAndPrint(const std::string& yaml_file) {
    PRINT_INFO("====== Loading and printing data paths ======\n");

    if (!CheckYamlFile(yaml_file, "data paths")) {
      return;
    }

    YamlParser::Ptr parser(new YamlParser(yaml_file));

    // 读取data部分
    std::string relative_root_path;
    parser->ParseConfig("data.root_path", relative_root_path);
    data_root_path = GetFullPath("", relative_root_path);  // 转换为绝对路径

    std::string relative_path;
    parser->ParseConfig("data.raw_image_data_folder", relative_path);
    raw_image_data_folder_path = GetFullPath(data_root_path, relative_path);

    parser->ParseConfig("data.semantic_data_folder", relative_path);
    semantic_data_folder_path = GetFullPath(data_root_path, relative_path);

    parser->ParseConfig("data.raw_imu_data_file", relative_path);
    raw_imu_data_file_path = GetFullPath(data_root_path, relative_path);

    parser->ParseConfig("data.ground_truth_data_file", relative_path);
    ground_truth_data_file_path = GetFullPath(data_root_path, relative_path);

    parser->ParseConfig("data.raw_lidar_data_folder", relative_path);
    raw_lidar_data_folder_path = GetFullPath(data_root_path, relative_path);

    parser->ParseConfig("data.semantic_contours_file", relative_path);
    semantic_contours_file_path = GetFullPath(data_root_path, relative_path);

    // 打印加载的数据路径（使用绝对路径）
    PRINT_DEBUG("Root Path: %s\n", data_root_path.c_str());
    PRINT_DEBUG("Raw Image Data Folder: %s\n", raw_image_data_folder_path.c_str());
    PRINT_DEBUG("Semantic Data Folder: %s\n", semantic_data_folder_path.c_str());
    PRINT_DEBUG("Raw IMU Data File: %s\n", raw_imu_data_file_path.c_str());
    PRINT_DEBUG("Ground Truth Data File: %s\n", ground_truth_data_file_path.c_str());
    PRINT_DEBUG("Raw Lidar Data Folder: %s\n", raw_lidar_data_folder_path.c_str());
    PRINT_DEBUG("Semantic Contours File: %s\n", semantic_contours_file_path.c_str());
  }

  // // 获取完整数据路径
  // std::string RawImageFolder() const { return raw_image_data_folder; }
  // std::string SemanticImageFolder() const { return semantic_data_folder; }
  // std::string RawImuFile() const { return raw_imu_data_file; }
  // std::string GroundTruthFile() const { return ground_truth_data_file; }
  // std::string RawLidarFolder() const { return raw_lidar_data_folder; }
};
#endif // VISUAL_SEMANTIC_LOCALIZATION_OPTIONS_H
