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

#endif // VISUAL_SEMANTIC_LOCALIZATION_TYPES_H 