#ifndef VISUAL_SEMANTIC_LOCALIZATION_CORE_H
#define VISUAL_SEMANTIC_LOCALIZATION_CORE_H

#include <string>
#include <unordered_map>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "data_types.h"
#include "visual_semantic_localization_options.h"
#include "ESKF/eskf.h"
#include "common_utils/eigen_types.h"

class VisualSemanticLocalizationCore {
public:
    VisualSemanticLocalizationCore();
    ~VisualSemanticLocalizationCore() = default;

    bool Init(const VisualSemanticLocalizationOptions& options);
    void Process(const DataGroup &data_group);

private:

    bool LoadSemanticPointcloudMapData(const std::string& dir_path);

    bool initialized_ = false;
    bool eskf_initialized_ = false;
    bool is_first_frame_ = true;
    double last_frame_timestamp_ = 0.0;
    double last_imu_timestamp_ = 0.0;
    double current_frame_timestamp_ = 0.0;
    double current_frame_time_diff_ = 0.0;

    // 激光雷达到世界坐标系,真值
    Mat3d r_l_to_w_gt_;
    Vec3d t_l_in_w_gt_;

    // 相机到世界坐标系,真值，由外参数和真值计算得到
    // 在本算法中，统一使用IMU坐标系作为基准坐标系
    // 所以需要将雷达坐标系的真值转换到IMU坐标系下
    Mat3d r_i_to_w_gt_;
    Vec3d t_i_in_w_gt_;

    using PointcloudXYZIPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;

    // Semantic pointcloud data
    std::unordered_map<std::string, PointcloudXYZIPtr> semantic_pointcloud_map_;
    
    VisualSemanticLocalizationOptions options_;
    ESKF::Ptr eskf_;
};

#endif