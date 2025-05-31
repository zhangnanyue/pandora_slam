#ifndef COMMON_UTILS_PCL_UTILS_H
#define COMMON_UTILS_PCL_UTILS_H

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief 将输入的点云转换为带颜色的点云。
 *
 * 该函数将一个类型为 `pcl::PointCloud<PointT>` 的点云转换为
 * `pcl::PointCloud<PointRGBT>`， 并为每个点设置指定的 RGB 颜色值。
 *
 * @tparam PointT 输入点云的点类型（例如 `pcl::PointXYZ`）。
 * @tparam PointRGBT 输出点云的点类型（例如 `pcl::PointXYZRGB`）。
 *
 * @param[in] input_cloud 输入的点云。
 * @param[out] output_cloud 输出的带颜色的点云。
 * @param[in] rgb 指定的 RGB 颜色值，使用 `Eigen::Vector3i` 表示。
 */
template <typename PointT, typename PointRGBT>
void ConvertToRgbCloud(const typename pcl::PointCloud<PointT>::Ptr &input_cloud,
                       typename pcl::PointCloud<PointRGBT>::Ptr &output_cloud,
                       const Eigen::Vector3i &rgb) {

  // 清空输出点云
  output_cloud->points.clear();

  // 遍历输入点云中的每个点
  for (const auto &point : input_cloud->points) {
    PointRGBT colored_point;

    // 复制空间坐标
    colored_point.x = point.x;
    colored_point.y = point.y;
    colored_point.z = point.z;

    // 设置颜色，确保颜色值在 0-255 范围内
    colored_point.r = static_cast<uint8_t>(std::clamp(rgb[0], 0, 255));
    colored_point.g = static_cast<uint8_t>(std::clamp(rgb[1], 0, 255));
    colored_point.b = static_cast<uint8_t>(std::clamp(rgb[2], 0, 255));

    // 设置 Alpha 通道为不透明
    colored_point.a = 255;

    // 将彩色点添加到输出点云中
    output_cloud->points.push_back(colored_point);
  }

  // 更新输出点云的宽度、高度和密集属性
  output_cloud->width = static_cast<uint32_t>(output_cloud->points.size());
  output_cloud->height = 1;
  output_cloud->is_dense = true;
}

#endif // COMMON_UTILS_PCL_UTILS_H