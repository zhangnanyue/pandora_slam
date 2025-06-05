#ifndef VISUAL_SEMANTIC_LOCALIZATION_CORE_H
#define VISUAL_SEMANTIC_LOCALIZATION_CORE_H

#include <string>
#include <unordered_map>
#include <vector>
#include <opencv2/opencv.hpp>
#include "data_types.h"
#include "visual_semantic_localization_options.h"
#include "ESKF/eskf.h"
#include "common_utils/eigen_types.h"

class VisualSemanticLocalizationCore {
public:
    VisualSemanticLocalizationCore();
    ~VisualSemanticLocalizationCore() = default;

    bool Initialize(const VisualSemanticLocalizationOptions& options);
    void Run(const DataGroup &data_group);

private:
    bool initialized_ = false;
    bool eskf_initialized_ = false;
    bool is_first_frame_ = true;
    double last_frame_timestamp_ = 0.0;
    double last_imu_timestamp_ = 0.0;
    double current_frame_timestamp_ = 0.0;
    double current_frame_time_diff_ = 0.0;
    
    // 激光雷达到IMU坐标系
    Mat4d T_imu_backlidar_;
    Mat3d r_l_to_i_;
    Vec3d t_l_to_i_;

    // 相机到激光雷达坐标系
    Mat4d T_backlidar_camera_;

    // 激光雷达到前激光雷达坐标系
    Mat4d T_backlidar_frontlidar_;

    // IMU到相机坐标系
    Mat4d T_camera_imu_;
    Mat3d r_i_to_c_;
    Vec3d t_i_to_c_;

    // IMU到激光雷达坐标系
    Mat4d T_backlidar_imu_;
    Mat3d r_i_to_l_;
    Vec3d t_i_to_l_;

    // 激光雷达到世界坐标系,真值
    Mat3d r_l_to_w_gt_;
    Vec3d t_l_to_w_gt_;

    // 相机到世界坐标系,真值，由外参数和真值计算得到
    // 在本算法中，统一使用IMU坐标系作为基准坐标系
    // 所以需要将雷达坐标系的真值转换到IMU坐标系下
    Mat3d r_i_to_w_gt_;
    Vec3d t_i_to_w_gt_;


    
    VisualSemanticLocalizationOptions options_;
    ESKF::Ptr eskf_;
};

#endif