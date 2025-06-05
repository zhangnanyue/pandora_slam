#ifndef VISUAL_SEMANTIC_LOCALIZATION_TYPES_H
#define VISUAL_SEMANTIC_LOCALIZATION_TYPES_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <memory>
#include <unordered_map>
#include <vector>
#include <map>



/**
 * @brief IMU数据结构
 */
struct IMUData {
    double timestamp;                    ///< 时间戳（秒）
    Eigen::Vector3d accel;              ///< 加速度计数据 (ax, ay, az)
    Eigen::Vector3d gyro;               ///< 陀螺仪数据 (gx, gy, gz)
    
    IMUData() : timestamp(0.0) {
        accel.setZero();
        gyro.setZero();
    }
};

/**
 * @brief 相机数据结构
 */
struct CameraData {
    double timestamp;                   ///< 时间戳（秒）
    cv::Mat image;                      ///< 图像数据

    CameraData() : timestamp(0.0) {}
};

/**
 * @brief 点云数据结构（模板类）
 * @tparam PointT 点云点类型
 */
template<typename PointT>
struct PointCloudData {
    double timestamp;                   ///< 时间戳（秒）
    typename pcl::PointCloud<PointT>::Ptr cloud;  ///< 点云数据

    PointCloudData() : timestamp(0.0) {
        cloud.reset(new pcl::PointCloud<PointT>);
    }
};

// 常用的点云类型别名
using PointCloudXYZIData = PointCloudData<pcl::PointXYZI>;    ///< XYZI点云数据
using PointCloudXYZData = PointCloudData<pcl::PointXYZ>;      ///< XYZ点云数据
using PointCloudXYZRGBData = PointCloudData<pcl::PointXYZRGB>; ///< XYZRGB点云数据
using PointCloudXYZRGBAData = PointCloudData<pcl::PointXYZRGBA>; ///< XYZRGBA点云数据

/**
 * @brief 位姿数据结构
 */
struct PoseData {
    double timestamp;         ///< 时间戳（秒）
    Eigen::Matrix4d T;        ///< 4x4变换矩阵
    Eigen::Vector3d t;        ///< 位置
    Eigen::Quaterniond q;     ///< 四元数
    Eigen::Matrix3d r;        ///< 旋转矩阵

    PoseData() : timestamp(0.0) {
        T.setIdentity();
        t.setZero();
        q.setIdentity();
        r.setIdentity();
    }
};

/**
 * @brief 语义轮廓数据结构
 */
struct SemanticContoursData {
    double lidar_timestamp;                     ///< 激光雷达时间戳
    double camera_timestamp;                    ///< 相机时间戳
    bool is_valid = false;                      ///< 数据是否有效
    std::unordered_map<std::string, std::vector<cv::Point>> category_contour;  ///< 类别轮廓映射
    
    SemanticContoursData() : lidar_timestamp(0.0), camera_timestamp(0.0) {}
};

/**
 * @brief 传感器数据组结构
 * 用于存储同步的传感器数据
 */
struct DataGroup {
    double timestamp;                                            ///< 数据组时间戳
    std::shared_ptr<cv::Mat> raw_image;                          ///< 原始图像数据
    std::shared_ptr<cv::Mat> semantic_mask_image;                ///< 语义掩码图像数据
    std::shared_ptr<std::map<double, IMUData>> imu_data;         ///< IMU数据
    std::shared_ptr<PoseData> ground_truth;                      ///< 真值位姿数据
    std::shared_ptr<SemanticContoursData> semantic_contours;     ///< 语义轮廓数据
    bool is_complete;                                            ///< 数据是否完整标志
    bool is_init_frame = false;                                  ///< 是否是第一帧
    std::shared_ptr<cv::Mat> raw_image_gray;                     ///< 原始图像灰度图


    // 构造函数
    DataGroup() : timestamp(0.0),
                  raw_image(nullptr),
                  semantic_mask_image(nullptr),
                  imu_data(nullptr),
                  ground_truth(nullptr),
                  semantic_contours(nullptr),
                  is_complete(false) {}

    // 检查数据组是否有效
    bool IsValid() const {
        return is_complete && 
               raw_image != nullptr && 
               semantic_mask_image != nullptr && 
               imu_data != nullptr && 
               ground_truth != nullptr && 
               semantic_contours != nullptr;
    }
};

#endif // VISUAL_SEMANTIC_LOCALIZATION_TYPES_H 