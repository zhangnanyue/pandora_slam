#ifndef LIDRAR_VISUAL_ALIGNMENT_VOXEL_UTILS_H
#define LIDRAR_VISUAL_ALIGNMENT_VOXEL_UTILS_H

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>

struct Voxel {
  using Ptr = std::shared_ptr<Voxel>;
  float size;
  Eigen::Vector3d voxel_origin;
  Eigen::Vector3d voxel_color;
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud;

  Voxel(float _size)
      : size(_size), voxel_origin(Eigen::Vector3d::Zero()),
        voxel_color(Eigen::Vector3d::Zero()),
        point_cloud(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()) {}
};

class VoxelPosition {
public:
  int64_t x, y, z;
  VoxelPosition(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}
  bool operator==(const VoxelPosition &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }

  bool operator<(const VoxelPosition &p) const {
    if (x < p.x)
      return true;
    if (x > p.x)
      return false;
    if (y < p.y)
      return true;
    if (y > p.y)
      return false;
    if (z < p.z)
      return true;
  };
};

// namespace std {
// template <> struct hash<VoxelPosition> {
//   std::size_t operator()(const VoxelPosition &s) const {
//     return ((std::hash<int64_t>()(s.x) ^ (std::hash<int64_t>()(s.y) << 1)) >>
//             1) ^
//            (std::hash<int64_t>()(s.z) << 1);
//   }
// };
// } // namespace std

namespace std {
template <> struct hash<VoxelPosition> {
  size_t operator()(const VoxelPosition &s) const {
    return ((std::hash<int64_t>()(s.x) * 73856093) ^
            (std::hash<int64_t>()(s.y) * 19349663) ^
            (std::hash<int64_t>()(s.z) * 83492791));
  }
};
} // namespace std

struct CountPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double intensity = 0.0;
  int count = 0;
};

/**
 * @brief 使用体素栅格过滤对点云进行降采样处理。
 *
 * 此函数采用给定体素大小对点云进行空间分割，并计算各体素内点的平均值，生成降采样后的点云。
 * 体素大小过小时（小于0.01f），将不进行降采样处理。
 *
 * @param point_cloud 输入和输出的点云。传入的点云将被降采样并覆盖。
 * @param voxel_size 体素大小，用于划分点云空间的单位长度。数值应大于等于
 * 0.01f。
 */
void DownsampleVoxelGrid(pcl::PointCloud<pcl::PointXYZI> &point_cloud,
                         const float &voxel_size) {
  if (voxel_size < 0.01f) {
    return;
  }
  std::unordered_map<VoxelPosition, CountPoint> voxel_map;
  voxel_map.reserve(point_cloud.size());

  int point_cloud_size = point_cloud.size();

  for (const auto &p_c : point_cloud) {
    // 使用整数运算避免浮点数精度问题
    int64_t ix = static_cast<int64_t>(std::floor(p_c.x / voxel_size));
    int64_t iy = static_cast<int64_t>(std::floor(p_c.y / voxel_size));
    int64_t iz = static_cast<int64_t>(std::floor(p_c.z / voxel_size));

    VoxelPosition position(ix, iy, iz);
    auto &cp = voxel_map[position];
    cp.x += p_c.x;
    cp.y += p_c.y;
    cp.z += p_c.z;
    cp.intensity += p_c.intensity;
    cp.count += 1;
  }

  pcl::PointCloud<pcl::PointXYZI> downsampled_cloud;
  downsampled_cloud.reserve(voxel_map.size());

  for (const auto &voxel : voxel_map) {
    const auto &cp = voxel.second;
    pcl::PointXYZI point;
    point.x = static_cast<float>(cp.x / cp.count);
    point.y = static_cast<float>(cp.y / cp.count);
    point.z = static_cast<float>(cp.z / cp.count);
    point.intensity = static_cast<float>(cp.intensity / cp.count);
    downsampled_cloud.push_back(point);
  }

  point_cloud = std::move(downsampled_cloud);
  return;
}

#endif // LIDRAR_VISUAL_ALIGNMENT_VOXEL_UTILS_H