#include "lidar_visual_alignment/lidar_visual_alignment.h"
#include <random>

#include "common_utils/math_utils.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <opencv2/core/eigen.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

#include "ceres/ceres.h"

#include <pcl/filters/voxel_grid.h>

// bool comp(int i, int j) { return (curvatures[i] < curvatures[j]); }

LidarVisualAlignment::LidarVisualAlignment(
    const LidarVisualAlignmentOptions &params)
    : params_(params) {
  init_r_l_to_c_ = params_.T_cam_lidar.block<3, 3>(0, 0);
  init_t_l_in_c_ = params_.T_cam_lidar.block<3, 1>(0, 3);
  corrected_r_l_to_c_ = init_r_l_to_c_;
  corrected_t_l_in_c_ = init_t_l_in_c_;
  camera_model_ = std::allocate_shared<PinholeCamera>(
      Eigen::aligned_allocator<PinholeCamera>(), params_.image_width,
      params_.image_height, params_.fx, params_.fy, params_.cx, params_.cy,
      params_.k1, params_.k2, params_.p1, params_.p2, params_.k3, false);
}

void LidarVisualAlignment::Proc(
    const cv::Mat &raw_image,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &raw_point_cloud) {

  if (raw_image.cols != params_.resolution[0] ||
      raw_image.rows != params_.resolution[1]) {
    PRINT_ERROR(RED "Image size %d %d is not consistent with the camera "
                    "parameters %d %d\n" RESET,
                raw_image.cols, raw_image.rows, params_.resolution[0],
                params_.resolution[1]);
    std::exit(EXIT_FAILURE);
  }

  if (raw_image.type() == CV_8UC1) {
    gray_image_ = raw_image;
    cv::cvtColor(raw_image, rgb_image_, cv::COLOR_GRAY2BGR);
    if (params_.downsample_factor > 1) {
      double downsample_factor = 1.0 / params_.downsample_factor;
      cv::resize(gray_image_, gray_image_, cv::Size(), downsample_factor,
                 downsample_factor, cv::INTER_NEAREST);
      cv::resize(rgb_image_, rgb_image_, cv::Size(), downsample_factor,
                 downsample_factor, cv::INTER_NEAREST);
    }
  } else if (raw_image.type() == CV_8UC3) {
    rgb_image_ = raw_image;
    cv::cvtColor(raw_image, gray_image_, cv::COLOR_BGR2GRAY);
    if (params_.downsample_factor > 1) {
      double downsample_factor = 1.0 / params_.downsample_factor;
      cv::resize(gray_image_, gray_image_, cv::Size(), downsample_factor,
                 downsample_factor, cv::INTER_NEAREST);
      cv::resize(rgb_image_, rgb_image_, cv::Size(), downsample_factor,
                 downsample_factor, cv::INTER_NEAREST);
    }
  } else {
    PRINT_ERROR(
        RED "Unsupported image type, please use CV_8UC3 or CV_8UC1\n" RESET);
    std::exit(EXIT_FAILURE);
  }
  //   cv::imshow("raw image", gray_image_);
  //   cv::waitKey();
  cv::imwrite(params_.debug_result_path + "raw_gray_image.png", gray_image_);
  cv::imwrite(params_.debug_result_path + "raw_rgb_image.png", rgb_image_);

  pcl::io::savePCDFileASCII(params_.debug_result_path + "raw_point_cloud.pcd",
                            *raw_point_cloud);

  // 对原生的点云进行进行体素滤波
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_point_cloud =
      PointcloudXYZIPtr(new PointcloudXYZI);
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  // voxel_grid.setLeafSize(params_.voxel_grid_size, params_.voxel_grid_size,
  //                        params_.voxel_grid_size);
  voxel_grid.setLeafSize(0.1, 0.1, 0.1);
  voxel_grid.setInputCloud(raw_point_cloud);
  voxel_grid.filter(*filtered_point_cloud);
  pcl::io::savePCDFileASCII(params_.debug_result_path +
                                "filtered_point_cloud.pcd",
                            *filtered_point_cloud);

  PointcloudXYZIPtr projection_point_cloud =
      PointcloudXYZIPtr(new PointcloudXYZI);
  std::vector<std::vector<std::vector<pcl::PointXYZI>>> image_pts_container;
  // TODO:
  // image_pts_container和projection_point_cloud应该存的是相机坐标系下的点？
  LidarProjectToImage(corrected_r_l_to_c_, corrected_t_l_in_c_,
                      filtered_point_cloud, projection_point_cloud,
                      image_pts_container);
  pcl::io::savePCDFileASCII(params_.debug_result_path +
                                "projection_point_cloud.pcd",
                            *projection_point_cloud);

  PointcloudXYZIPtr fix_projection_point_cloud =
      PointcloudXYZIPtr(new PointcloudXYZI);
  std::vector<std::vector<std::vector<pcl::PointXYZI>>> fix_image_pts_container;
  fix_image_pts_container.resize(
      params_.image_height,
      std::vector<std::vector<pcl::PointXYZI>>(params_.image_width));
  for (int j = 0; j < image_pts_container.size(); j++) {
    for (int i = 0; i < image_pts_container[j].size(); i++) {
      if (image_pts_container[j][i].empty()) {
        continue;
      }

      double min_depth = std::numeric_limits<double>::max();
      pcl::PointXYZI min_depth_pt;

      for (int k = 0; k < image_pts_container[j][i].size(); k++) {
        pcl::PointXYZI p = image_pts_container[j][i][k];
        double depth = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        if (depth < min_depth) {
          min_depth = depth;
          min_depth_pt = p;
        }
      }
      fix_image_pts_container[j][i].push_back(min_depth_pt);
      fix_projection_point_cloud->points.push_back(min_depth_pt);
    }
  }

  fix_projection_point_cloud->width = fix_projection_point_cloud->size();
  fix_projection_point_cloud->height = 1;
  fix_projection_point_cloud->is_dense = true;
  pcl::io::savePCDFileASCII(params_.debug_result_path +
                                "fix_projection_point_cloud.pcd",
                            *fix_projection_point_cloud);

  // 为什么下面要先生成interpolated_image_pts_container，在使用interpolated_image_pts_container
  // 中的值对fix_image_pts_container进行插值？
  // 在插值计算中，是使用某个点周围4个点进行插值的，
  // 如果直接插入原来的容器，后导致后续其他相邻点的计算引入之前计算的插值点，而不是原始点
  std::vector<std::vector<std::vector<pcl::PointXYZI>>>
      interpolated_image_pts_container;
  interpolated_image_pts_container.resize(
      params_.image_height,
      std::vector<std::vector<pcl::PointXYZI>>(params_.image_width));
  for (int j = 0; j < params_.image_height; j++) {
    for (int i = 0; i < params_.image_width; i++) {
      if (fix_image_pts_container[j][i].empty()) {
        // std::cout << "j: " << j << " i: " << i << std::endl;
        interpolated_image_pts_container[j][i].emplace_back(
            InterpolatePoint(fix_image_pts_container, j, i));
      }
    }
  }

  for (int j = 0; j < params_.image_height; j++) {
    for (int i = 0; i < params_.image_width; i++) {
      if (fix_image_pts_container[j][i].empty()) {
        fix_image_pts_container[j][i].emplace_back(
            interpolated_image_pts_container[j][i][0]);
      }
    }
  }

  std::vector<std::vector<std::vector<pcl::PointXYZI>>>
      curvature_image_pts_container;
  curvature_image_pts_container.resize(
      params_.image_height,
      std::vector<std::vector<pcl::PointXYZI>>(params_.image_width));
  // 边缘点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr curvature_point_cloud;
  curvature_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

  // 把图像的高看成线束的数量
  int num_of_scans = params_.image_height;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> scan_point_cloud_vec(
      num_of_scans);
  scan_point_cloud_vec.resize(num_of_scans);
  for (int j = 0; j < params_.image_height; j++) {
    pcl::PointCloud<pcl::PointXYZI> point_cloud;
    for (int i = 0; i < params_.image_width; i++) {
      point_cloud.emplace_back(fix_image_pts_container[j][i][0]);
    }
    point_cloud.width = point_cloud.size();
    point_cloud.height = 1;
    scan_point_cloud_vec[j] = point_cloud;
    PRINT_DEBUG("scan_point_cloud_vec[%d].size(): %d\n", j, point_cloud.size());
    pcl::io::savePCDFile(params_.debug_result_path +
                             "debug/curvature_point_cloud_" +
                             std::to_string(j) + ".pcd",
                         point_cloud);
  }

  std::vector<int> start_id_scans(num_of_scans, 0);
  std::vector<int> end_id_scans(num_of_scans, 0);
  int offset = 5;
  PointcloudXYZIPtr scans_point_cloud = PointcloudXYZIPtr(new PointcloudXYZI);
  for (int i = 0; i < num_of_scans; i++) {
    start_id_scans[i] = scans_point_cloud->size() + offset;
    *scans_point_cloud += scan_point_cloud_vec[i];
    end_id_scans[i] = scans_point_cloud->size() - offset - 1;
    if (i == 0 || i == 2 || i == 20) {
      std::cout << start_id_scans[i] << " " << end_id_scans[i] << std::endl;
    }
  }
  pcl::io::savePCDFile(params_.debug_result_path + "scans_point_cloud.pcd",
                       *scans_point_cloud);

  // 每个点的曲率大小
  std::vector<double> curvatures(scans_point_cloud->size(), 0.0f);
  // 存储按照点的曲率排好序的特征点的ID
  std::vector<int> sort_index(scans_point_cloud->size(), 0);
  // 避免特征点密集分布，每选一个特征点其前后5个点一般就不选取了
  std::vector<int> neighbor_picked(scans_point_cloud->size(), 0);
  // 记录特征点属于那种类型：极大边线点、次极大边线点、极小平面点、次极小平面点
  /*
    Label 2: edge_sharp
    Label 1: edge_less_sharp, 包含Label 2
    Label -1: surface_flat
    Label 0: surface_less_flat， 包含Label -1，因为点太多，最后会降采样
 */
  std::vector<int> labels(scans_point_cloud->size(), 0);

  for (int i = offset; i < scans_point_cloud->size() - offset; i++) {
    double diff_x = 0.0f;
    double diff_y = 0.0f;
    double diff_z = 0.0f;
    for (int interval = -offset; interval <= offset; interval++) {
      if (interval == 0) {
        diff_x -= 2.0f * offset * scans_point_cloud->points[i].x;
        diff_y -= 2.0f * offset * scans_point_cloud->points[i].y;
        diff_z -= 2.0f * offset * scans_point_cloud->points[i].z;
      } else {
        diff_x += scans_point_cloud->points[i + interval].x;
        diff_y += scans_point_cloud->points[i + interval].y;
        diff_z += scans_point_cloud->points[i + interval].z;
      }
    }
    curvatures[i] = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
    sort_index[i] = i;
    neighbor_picked[i] = 0;
    labels[i] = 0;
  }

  PointcloudXYZI egde_points_sharp;      // 极大边线点
  PointcloudXYZI egde_points_less_sharp; // 次极大边线点
  PointcloudXYZI surface_points_flat;    // 极小平面点
  PointcloudXYZI surface_points_less_flat; // 次极小平面点（经过降采样）

  // 对每根扫描线进行操作
  for (int i = 0; i < num_of_scans; i++) {
    if (end_id_scans[i] - start_id_scans[i] < 6) {
      continue;
    }
    // if (i == 1)
    //   break;
    std::cout << "scan: " << i << std::endl;
    // 为了使特征点均匀分布，将一个scan分成interval个扇区
    int interval = 6;
    for (int sector_idx = 0; sector_idx < interval; sector_idx++) {
      int sector_point_num =
          (end_id_scans[i] - start_id_scans[i] + 1) / interval;
      int sector_start_pt_idx =
          start_id_scans[i] + sector_point_num * sector_idx;
      int sector_end_pt_idx =
          start_id_scans[i] + sector_point_num * (sector_idx + 1) - 1;
      std::cout << sector_start_pt_idx << "   " << sector_end_pt_idx << "  "
                << sector_end_pt_idx - sector_start_pt_idx << std::endl;
      std::sort(sort_index.begin() + sector_start_pt_idx,
                sort_index.begin() + sector_end_pt_idx + 1,
                [&curvatures](int m, int n) {
                  return (curvatures[m] < curvatures[n]);
                });
      // 选取极大边线点和次极大边线点
      int largest_picked_number = 0;
      for (int k = sector_end_pt_idx; k >= sector_start_pt_idx; k--) {
        int index = sort_index[k];
        std::cout << "k: " << k << "   index: " << index << std::endl;
        // std::cout << "curvature: " << curvatures[index] << std::endl;
        if (neighbor_picked[index] == 0 && curvatures[index] > 0.1) {
          // std::cout << "curvature: " << curvatures[index] << std::endl;
          largest_picked_number++;
          // 共收集两个极大边线点
          if (largest_picked_number <= 4) {
            labels[index] = 2;
            egde_points_sharp.push_back(scans_point_cloud->points[index]);
            // egde_points_less_sharp.push_back(scans_point_cloud->points[index]);
          } else if (largest_picked_number <= 16) { // 收集18个次大边线点
            labels[index] = 1;
            egde_points_less_sharp.push_back(scans_point_cloud->points[index]);
          } else {
            break;
          }
          neighbor_picked[index] = 1;
          // ID为index的特征点的相邻scan点距离的平方<=0.05的点标记为选择过，避免特征点密集分布
          // 两个条件，该特征点左右的5个点，且距离的平方小于0.05
          // for (int l = 1; l <= 3; l++) {
          //   float diff_x = scans_point_cloud->points[index + l].x -
          //                  scans_point_cloud->points[index + l - 1].x;
          //   float diff_y = scans_point_cloud->points[index + l].y -
          //                  scans_point_cloud->points[index + l - 1].y;
          //   float diff_z = scans_point_cloud->points[index + l].z -
          //                  scans_point_cloud->points[index + l - 1].z;
          //   if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05) {
          //     break;
          //   }
          //   neighbor_picked[index + l] = 1;
          // }
          // for (int l = -1; l >= -3; l--) {
          //   float diff_x = scans_point_cloud->points[index + l].x -
          //                  scans_point_cloud->points[index + l + 1].x;
          //   float diff_y = scans_point_cloud->points[index + l].y -
          //                  scans_point_cloud->points[index + l + 1].y;
          //   float diff_z = scans_point_cloud->points[index + l].z -
          //                  scans_point_cloud->points[index + l + 1].z;
          //   if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05) {
          //     break;
          //   }
          //   neighbor_picked[index + l] = 1;
          // }
        }
      }

      // 选取极小平面点
      int smallest_picked_number = 0;
      for (int k = sector_start_pt_idx; k <= sector_end_pt_idx; k++) {
        int index = sort_index[k];
        if (neighbor_picked[index] == 0 && curvatures[index] < 0.1) {
          smallest_picked_number++;
          labels[index] = -1;
          surface_points_flat.push_back(scans_point_cloud->points[index]);

          // 共收集3个极小平面点
          if (smallest_picked_number >= 4) {
            break;
          }
          neighbor_picked[index] = 1;
          for (int l = 1; l <= 5; l++) {
            float diff_x = scans_point_cloud->points[index + l].x -
                           scans_point_cloud->points[index + l - 1].x;
            float diff_y = scans_point_cloud->points[index + l].y -
                           scans_point_cloud->points[index + l - 1].y;
            float diff_z = scans_point_cloud->points[index + l].z -
                           scans_point_cloud->points[index + l - 1].z;
            if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05) {
              break;
            }
            neighbor_picked[index + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diff_x = scans_point_cloud->points[index + l].x -
                           scans_point_cloud->points[index + l + 1].x;
            float diff_y = scans_point_cloud->points[index + l].y -
                           scans_point_cloud->points[index + l + 1].y;
            float diff_z = scans_point_cloud->points[index + l].z -
                           scans_point_cloud->points[index + l + 1].z;
            if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05) {
              break;
            }
            neighbor_picked[index + l] = 1;
          }
        }
      }

      // 选取次极小平面点
      for (int k = sector_start_pt_idx; k <= sector_end_pt_idx; k++) {
        if (neighbor_picked[k] <= 0) {
          surface_points_less_flat.push_back(scans_point_cloud->points[k]);
        }
      }
    }
    // if (!egde_points_sharp.empty()) {
    //   pcl::io::savePCDFile(params_.debug_result_path + "/debug_1/" +
    //                            "egde_points_sharp_" + std::to_string(i) +
    //                            ".pcd",
    //                        egde_points_sharp);
    // }
    // if (!egde_points_less_sharp.empty()) {
    //   pcl::io::savePCDFile(params_.debug_result_path + "/debug_1/" +
    //                            "egde_points_less_sharp_" + std::to_string(i)
    //                            +
    //                            ".pcd",
    //                        egde_points_less_sharp);
    // }
    // if (!surface_points_flat.empty()) {
    //   pcl::io::savePCDFile(params_.debug_result_path + "/debug_1/" +
    //                            "surface_points_flat_" + std::to_string(i) +
    //                            ".pcd",
    //                        surface_points_flat);
    // }
    // if (!surface_points_less_flat.empty()) {
    //   pcl::io::savePCDFile(params_.debug_result_path + "/debug_1/" +
    //                            "surface_points_less_flat_" +
    //                            std::to_string(i) +
    //                            ".pcd",
    //                        surface_points_less_flat);
    // }
  }
  if (!egde_points_sharp.empty()) {
    pcl::io::savePCDFile(params_.debug_result_path + "egde_points_sharp.pcd",
                         egde_points_sharp);
  }
  if (!egde_points_less_sharp.empty()) {
    pcl::io::savePCDFile(params_.debug_result_path +
                             "egde_points_less_sharp.pcd",
                         egde_points_less_sharp);
  }
  if (!surface_points_flat.empty()) {
    pcl::io::savePCDFile(params_.debug_result_path + "surface_points_flat.pcd",
                         surface_points_flat);
  }
  if (!surface_points_less_flat.empty()) {
    pcl::io::savePCDFile(params_.debug_result_path +
                             "surface_points_less_flat.pcd",
                         surface_points_less_flat);
  }

  /**
  for (int j = 0; j < params_.image_height; j++) {
    for (int i = offset_x; i < params_.image_width - offset_x; i++) {
      if (fix_image_pts_container[j][i].empty()) {
        exit(EXIT_FAILURE);
      }
      double diff_x = 0.0f;
      double diff_y = 0.0f;
      double diff_z = 0.0f;
      for (int offset = -offset_x; offset <= offset_x; offset++) {
        if (offset == 0) {
          diff_x -= 2.0f * offset_x * fix_image_pts_container[j][i][0].x;
          diff_y -= 2.0f * offset_x * fix_image_pts_container[j][i][0].y;
          diff_z -= 2.0f * offset_x * fix_image_pts_container[j][i][0].z;
        } else {
          diff_x += fix_image_pts_container[j][i + offset][0].x;
          diff_y += fix_image_pts_container[j][i + offset][0].y;
          diff_z += fix_image_pts_container[j][i + offset][0].z;
        }
      }
      double curvature = diff_x * diff_x + diff_y * diff_y + diff_z *
  diff_z; pcl::PointXYZI curvature_point; curvature_point =
  fix_image_pts_container[j][i][0]; if (curvature > 0.01f) {
        curvature_point.intensity = 100;
      } else {
        curvature_point.intensity = 20;
      }
      curvature_image_pts_container[j][i].emplace_back(curvature_point);
      curvature_point_cloud->points.emplace_back(curvature_point);
      // std::cout << curvature_image_pts_container[j][i][0].intensity
      //           << std::endl;
    }
  }
  curvature_point_cloud->width = curvature_point_cloud->points.size();
  curvature_point_cloud->height = 1;
  curvature_point_cloud->is_dense = true;
  pcl::io::savePCDFile(params_.debug_result_path +
  "curvature_point_cloud.pcd", *curvature_point_cloud);
  **/

  // for (int j = 5; j < params_.image_height - 5; j++) {
  //   for (int i = 0; i < params_.image_width; i++) {
  //     if (fix_image_pts_container[j][i].empty()) {
  //       exit(EXIT_FAILURE);
  //     }

  //     double diff_x = fix_image_pts_container[j - 5][i][0].x +
  //                     fix_image_pts_container[j - 4][i][0].x +
  //                     fix_image_pts_container[j - 3][i][0].x +
  //                     fix_image_pts_container[j - 2][i][0].x +
  //                     fix_image_pts_container[j - 1][i][0].x -
  //                     10 * fix_image_pts_container[j][i][0].x +
  //                     fix_image_pts_container[j + 1][i][0].x +
  //                     fix_image_pts_container[j + 2][i][0].x +
  //                     fix_image_pts_container[j + 3][i][0].x +
  //                     fix_image_pts_container[j + 4][i][0].x +
  //                     fix_image_pts_container[j + 5][i][0].x;

  //     double diff_y = fix_image_pts_container[j - 5][i][0].y +
  //                     fix_image_pts_container[j - 4][i][0].y +
  //                     fix_image_pts_container[j - 3][i][0].y +
  //                     fix_image_pts_container[j - 2][i][0].y +
  //                     fix_image_pts_container[j - 1][i][0].y -
  //                     10 * fix_image_pts_container[j][i][0].y +
  //                     fix_image_pts_container[j + 1][i][0].y +
  //                     fix_image_pts_container[j + 2][i][0].y +
  //                     fix_image_pts_container[j + 3][i][0].y +
  //                     fix_image_pts_container[j + 4][i][0].y +
  //                     fix_image_pts_container[j + 5][i][0].y;

  //     double diff_z = fix_image_pts_container[j - 5][i][0].z +
  //                     fix_image_pts_container[j - 4][i][0].z +
  //                     fix_image_pts_container[j - 3][i][0].z +
  //                     fix_image_pts_container[j - 2][i][0].z +
  //                     fix_image_pts_container[j - 1][i][0].z -
  //                     10 * fix_image_pts_container[j][i][0].z +
  //                     fix_image_pts_container[j + 1][i][0].z +
  //                     fix_image_pts_container[j + 2][i][0].z +
  //                     fix_image_pts_container[j + 3][i][0].z +
  //                     fix_image_pts_container[j + 4][i][0].z +
  //                     fix_image_pts_container[j + 5][i][0].z;

  //     double curvature =
  //         sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

  //     pcl::PointXYZI curvature_point;
  //     curvature_point = fix_image_pts_container[j][i][0];
  //     curvature_point.intensity = curvature;
  //     curvature_image_pts_container[j][i].emplace_back(curvature_point);
  //     curvature_point_cloud->points.emplace_back(curvature_point);

  //     std::cout << curvature_image_pts_container[j][i][0].intensity
  //               << std::endl;
  //   }
  // }

  // // Loop over rows (image_height)
  // for (int j = 5; j < params_.image_height - 5; j++) {
  //   // Loop over columns (image_width)
  //   for (int i = 5; i < params_.image_width - 5; i++) {
  //     if (fix_image_pts_container[j][i].empty()) {
  //       exit(EXIT_FAILURE);
  //     }

  //     // Calculate differences in the x, y, z directions in both i
  //     (horizontal)
  //     // and j (vertical) directions
  //     double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0;

  //     // Horizontal neighbors (i direction)
  //     for (int dx = -5; dx <= 5; ++dx) {
  //       if (i + dx >= 0 && i + dx < params_.image_width) {
  //         diff_x +=
  //             fix_image_pts_container[j][i + dx][0].x * (5 -
  //             std::abs(dx));
  //         diff_y +=
  //             fix_image_pts_container[j][i + dx][0].y * (5 -
  //             std::abs(dx));
  //         diff_z +=
  //             fix_image_pts_container[j][i + dx][0].z * (5 -
  //             std::abs(dx));
  //       }
  //     }

  //     // Vertical neighbors (j direction)
  //     for (int dy = -5; dy <= 5; ++dy) {
  //       if (j + dy >= 0 && j + dy < params_.image_height) {
  //         diff_x +=
  //             fix_image_pts_container[j + dy][i][0].x * (5 -
  //             std::abs(dy));
  //         diff_y +=
  //             fix_image_pts_container[j + dy][i][0].y * (5 -
  //             std::abs(dy));
  //         diff_z +=
  //             fix_image_pts_container[j + dy][i][0].z * (5 -
  //             std::abs(dy));
  //       }
  //     }

  //     // Subtract the current point to center the curvature calculation
  //     diff_x -= 10 * fix_image_pts_container[j][i][0].x;
  //     diff_y -= 10 * fix_image_pts_container[j][i][0].y;
  //     diff_z -= 10 * fix_image_pts_container[j][i][0].z;

  //     // Calculate curvature
  //     double curvature =
  //         sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

  //     // Create the point with curvature intensity
  //     pcl::PointXYZI curvature_point;
  //     curvature_point = fix_image_pts_container[j][i][0];
  //     curvature_point.intensity = curvature;

  //     // Store the point in the container and point cloud
  //     curvature_image_pts_container[j][i].emplace_back(curvature_point);
  //     curvature_point_cloud->points.emplace_back(curvature_point);

  //     std::cout << "Curvature at (" << j << ", " << i
  //               << "): " <<
  //               curvature_image_pts_container[j][i][0].intensity
  //               << std::endl;
  //   }
  // }

  // 在主函数中调用插值函数
  // fix_image_pts_container = interpolate_points(fix_image_pts_container);
  // cv::imwrite(params_.debug_result_path + "depath_image_8u.png",
  //             depath_image_8u);

  // cv::Mat interpolated_depath_image_8u = cv::Mat::zeros(
  //     image_pts_container.size(), image_pts_container[0].size(),
  //     CV_8UC3);

  // for (int j = 0; j < depath_image_8u.rows; ++j) {
  //   for (int i = 0; i < depath_image_8u.cols; ++i) {
  //     cv::Vec3b gray_value = depath_image_8u.at<cv::Vec3b>(j, i);
  //     if (gray_value[0] != 0 || gray_value[1] != 0 || gray_value[2] != 0)
  //     {
  //       std::cout << "Pixel at (" << j << ", " << i << ") = "
  //                 << "B: " << static_cast<int>(gray_value[0]) << ", "
  //                 << "G: " << static_cast<int>(gray_value[1]) << ", "
  //                 << "R: " << static_cast<int>(gray_value[2]) <<
  //                 std::endl;
  //       continue;
  //     }
  //     std::cout << "Pixel at (" << j << ", " << i << ") = "
  //               << "B: " << static_cast<int>(gray_value[0]) << ", "
  //               << "G: " << static_cast<int>(gray_value[1]) << ", "
  //               << "R: " << static_cast<int>(gray_value[2]) << ", ";

  //     // RGB每个通道进行插值
  //     uchar interpolated_x = InterpolateMat<uchar>(depath_image_8u, i, j,
  //     0); uchar interpolated_y = InterpolateMat<uchar>(depath_image_8u,
  //     i, j, 1); uchar interpolated_z =
  //     InterpolateMat<uchar>(depath_image_8u, i, j, 2);

  //     // // 将插值结果写回到图像
  //     // depath_image_8u.at<cv::Vec3b>(j, i)[0] =
  //     //     static_cast<uchar>(interpolated_x);
  //     // depath_image_8u.at<cv::Vec3b>(j, i)[1] =
  //     //     static_cast<uchar>(interpolated_y);
  //     // depath_image_8u.at<cv::Vec3b>(j, i)[2] =
  //     //     static_cast<uchar>(interpolated_z);

  //     std::cout << "fix: "
  //               << "R: " <<
  //               static_cast<int>(static_cast<uchar>(interpolated_x))
  //               << ", "
  //               << "G: " <<
  //               static_cast<int>(static_cast<uchar>(interpolated_y))
  //               << ", "
  //               << "B: " <<
  //               static_cast<int>(static_cast<uchar>(interpolated_z))
  //               << std::endl;
  //   }
  // }
  // cv::imwrite(params_.debug_result_path + "fix_depth_image_8u.png",
  //             depath_image_8u);

  // for (int j = 0; j < depath_image_8u.rows; ++j) {
  //   for (int i = 0; i < depath_image_8u.cols; ++i) {
  //     // 当前像素的灰度值
  //     uchar gray_value = depath_image_8u.at<uchar>(i, j);
  //     if (gray_value == 0) {
  //       continue; // 跳过背景
  //     }
  //   }
  // }

  // for (int j = 0; j < 1; ++j) {
  //   for (int i = 0; i < depth_image.cols; ++i) {
  //     // 访问当前像素的灰度值
  //     uchar gray_value = depth_image.at<uchar>(j, i);

  //     // 打印出灰度值
  //     std::cout << "Pixel at (" << j << ", " << i
  //               << ") = " << static_cast<int>(gray_value) << std::endl;
  //   }
  // }

  // for (auto &pts : image_pts_container) {
  //   for (auto &pt : pts) {
  //     PRINT_DEBUG(BLUE "pt size: %d. " RESET, pt.size());
  //     if (pt.size() == 0) {
  //       std::cout << std::endl;
  //       continue;
  //     }
  //     for (auto &p : pt) {
  //       double depth = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  //       // PRINT_DEBUG(BLUE "depth: %f. " RESET, depth);
  //       std::cout << depth << ", ";
  //     }
  //     std::cout << std::endl;
  //   }
  // }

  // for (auto &pts : image_pts_container) {

  // pcl::PointCloud<pcl::PointXYZI>::Ptr projection_depth_point_cloud;
  // LidarProjectToImage(corrected_r_l_to_c_, corrected_t_l_in_c_,
  //                     ProjectionType::DEPTH, false, raw_point_cloud,
  //                     rgb_image_, lidar_depth_image_,
  //                     projection_depth_point_cloud);
  // cv::imwrite(params_.debug_result_path + "lidar_depth_image.png",
  //             lidar_depth_image_);
  // pcl::io::savePCDFileASCII(params_.debug_result_path +
  //                               "projection_depth_point_cloud.pcd",
  //                           *projection_depth_point_cloud);

  // // LidarProjectToImage(corrected_r_l_to_c_, corrected_t_l_in_c_,
  // //                     ProjectionType::INTENSITY, false,
  // raw_point_cloud,
  // //                     rgb_image_, lidar_intensity_image_);
  // // cv::imwrite(params_.debug_result_path + "lidar_intensity_image.png",
  // //             lidar_intensity_image_);

  // // camera edge extract
  // ImageContoursDetection(params_.gaussian_blur_ksize,
  // params_.canny_threshold,
  //                        gray_image_, camera_contours_);
  // EdgeExtractionByContours(camera_contours_,
  //                          params_.edge_line_contour_min_length_threshold,
  //                          camera_edge_image_, camera_edge_point_cloud_);
  // cv::imwrite(params_.debug_result_path + "camera_edge_image.png",
  //             camera_edge_image_);
  // pcl::io::savePCDFileASCII(params_.debug_result_path +
  //                               "camera_edge_point_cloud.pcd",
  //                           *camera_edge_point_cloud_);

  // // lidar depth edge extract
  // ImageContoursDetection(params_.gaussian_blur_ksize,
  // params_.canny_threshold,
  //                        lidar_depth_image_, lidar_depth_contours_);
  // EdgeExtractionByContours(
  //     lidar_depth_contours_,
  //     params_.edge_line_contour_min_length_threshold,
  //     lidar_depth_edge_image_, lidar_depth_edge_point_cloud_);
  // cv::imwrite(params_.debug_result_path + "lidar_depth_edge_image.png",
  //             lidar_depth_edge_image_);
  // pcl::io::savePCDFileASCII(params_.debug_result_path +
  //                               "lidar_depth_edge_point_cloud.pcd",
  //                           *lidar_depth_edge_point_cloud_);

  // lidar intensity edge extract
  // ImageContoursDetection(params_.gaussian_blur_ksize,
  // params_.canny_threshold,
  //                        lidar_intensity_image_,
  //                        lidar_intensity_contours_);
  // EdgeExtractionByContours(
  //     lidar_intensity_contours_,
  //     params_.edge_line_contour_min_length_threshold,
  //     lidar_intensity_edge_image_, lidar_intensity_edge_point_cloud_);
  // cv::imwrite(params_.debug_result_path +
  // "lidar_intensity_edge_image.png",
  //             lidar_intensity_edge_image_);
  // pcl::io::savePCDFileASCII(params_.debug_result_path +
  //                               "lidar_intensity_edge_point_cloud.pcd",
  //                           *lidar_intensity_edge_point_cloud_);

  // lidar_edge_point_cloud_ = PointcloudXYZPtr(new PointcloudXYZ);

  // for (const auto &point : lidar_depth_edge_point_cloud_->points) {
  //   lidar_edge_point_cloud_->points.emplace_back(point);
  // }
  // for (const auto &point : lidar_intensity_edge_point_cloud_->points) {
  //   lidar_edge_point_cloud_->points.emplace_back(point);
  // }
  // lidar_edge_point_cloud_->width =
  // lidar_edge_point_cloud_->points.size(); lidar_edge_point_cloud_->height
  // = 1; lidar_edge_point_cloud_->is_dense = true;
  // pcl::io::savePCDFileASCII(params_.debug_result_path +
  //                               "lidar_edge_point_cloud.pcd",
  //                           *lidar_edge_point_cloud_);

  exit(0);

  ImageEdgeLineGenerator(params_.gaussian_blur_ksize, params_.canny_threshold,
                         params_.edge_line_contour_min_length_threshold,
                         gray_image_, edge_line_contour_image_,
                         image_edge_line_point_cloud_);
  pcl::io::savePCDFileASCII(params_.debug_result_path +
                                "image_edge_line_point_cloud.pcd",
                            *image_edge_line_point_cloud_);

  pcl::io::savePCDFileASCII(params_.debug_result_path + "raw_point_cloud.pcd",
                            *raw_point_cloud);
  std::unordered_map<VoxelPosition, Voxel::Ptr> voxel_map;
  BuildVoxelMapFromRawPointCloud(raw_point_cloud, params_.voxel_size,
                                 voxel_map);

  cv::Mat init_projection_image;
  GetProjectionImage(init_r_l_to_c_, init_t_l_in_c_, ProjectionType ::INTENSITY,
                     params_.colored_point_cloud_min_depth_threshold,
                     params_.colored_point_cloud_max_depth_threshold, false,
                     rgb_image_, raw_point_cloud, init_projection_image);
  cv::imwrite(params_.debug_result_path + "init_projection_image.png",
              init_projection_image);

  // DownsampleVoxelMap(voxel_map);

  LidarEdgeLineExtractor(voxel_map, params_.plane_ransac_distance_threshold,
                         params_.plane_point_size_threshold,
                         lidar_depth_continuous_edge_line_point_cloud_);
  pcl::io::savePCDFileASCII(
      params_.debug_result_path +
          "lidar_depth_continuous_edge_line_point_cloud.pcd",
      *lidar_depth_continuous_edge_line_point_cloud_);

  // exit(0);

  // std::vector<PnPData> pnp_data_vec;
  std::vector<DirectionPnPData> direction_pnp_data_vec;
  int distance_threshold = 30;
  int iter = 0;
  bool opt_flag = true;
  for (distance_threshold = 30; distance_threshold > 10;
       distance_threshold -= 1) {
    // For each distance, do twice optimization
    for (int cnt = 0; cnt < 2; cnt++) {
      BuildDirectionalVectorPnP(corrected_r_l_to_c_, corrected_t_l_in_c_,
                                distance_threshold,
                                image_edge_line_point_cloud_,
                                lidar_depth_continuous_edge_line_point_cloud_,
                                lidar_depth_continuous_edge_line_number_, true,
                                direction_pnp_data_vec);
      PRINT_INFO(GREEN "iteration:%d, distance:%d, cnt:%d, direction pnp data "
                       "size: %d\n" RESET,
                 iter++, distance_threshold, cnt,
                 direction_pnp_data_vec.size());

      cv::Mat projection_image;
      GetProjectionImage(corrected_r_l_to_c_, corrected_t_l_in_c_,
                         ProjectionType ::INTENSITY,
                         params_.colored_point_cloud_min_depth_threshold,
                         params_.colored_point_cloud_max_depth_threshold, false,
                         rgb_image_, raw_point_cloud, projection_image);
      cv::imwrite(params_.debug_result_path + "projection_image_" +
                      std::to_string(distance_threshold) + "_" +
                      std::to_string(cnt) + ".png",
                  projection_image);
      cv::imshow("Optimization", projection_image);
      cv::waitKey(100);
      // Eigen::Vector3d euler_angle(corrected_r_l_to_c_.eulerAngles(2, 1,
      // 0)); Eigen::Matrix3d opt_init_R; opt_init_R =
      // Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitZ()) *
      //              Eigen::AngleAxisd(euler_angle[1],
      //              Eigen::Vector3d::UnitY()) *
      //              Eigen::AngleAxisd(euler_angle[2],
      //              Eigen::Vector3d::UnitX());
      // // Eigen::Quaterniond q_l_to_c_init = Eigen::Quaterniond(opt_init_R);
      // Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      //     corrected_r_l_to_c_, Eigen::ComputeFullU | Eigen::ComputeFullV);
      // Eigen::Matrix3d orthogonal_R = svd.matrixU() *
      // svd.matrixV().transpose(); Eigen::Quaterniond
      // q_l_to_c_init(orthogonal_R); std::cout << "----------opt_init_R: " <<
      // std::endl
      //           << opt_init_R << std::endl;
      // std::cout << "----------corrected_r_l_to_c_: " << std::endl
      //           << corrected_r_l_to_c_ << std::endl;
      // double difference = (opt_init_R - corrected_r_l_to_c_).norm();
      // std::cout << "Difference between matrices: " << difference <<
      // std::endl;

      Eigen::Quaterniond corrected_q_l_to_c(corrected_r_l_to_c_);
      double extrinsic[7];
      extrinsic[0] = corrected_q_l_to_c.x();
      extrinsic[1] = corrected_q_l_to_c.y();
      extrinsic[2] = corrected_q_l_to_c.z();
      extrinsic[3] = corrected_q_l_to_c.w();
      extrinsic[4] = corrected_t_l_in_c_[0];
      extrinsic[5] = corrected_t_l_in_c_[1];
      extrinsic[6] = corrected_t_l_in_c_[2];

      // Eigen::Map<Eigen::Quaterniond> m_q =
      //     Eigen::Map<Eigen::Quaterniond>(extrinsic);
      // Eigen::Map<Eigen::Vector3d> m_t =
      //     Eigen::Map<Eigen::Vector3d>(extrinsic + 4);

      // ceres::Manifold *q_parameterization = new
      // ceres::QuaternionManifold();
      ceres::LocalParameterization *q_parameterization =
          new ceres::EigenQuaternionParameterization();
      ceres::Problem problem;
      problem.AddParameterBlock(extrinsic, 4, q_parameterization);
      problem.AddParameterBlock(extrinsic + 4, 3);

      for (auto data : direction_pnp_data_vec) {
        ceres::CostFunction *cost_function;
        cost_function = DirectionPnpDataCostFunction::Create(data);
        problem.AddResidualBlock(cost_function, NULL, extrinsic, extrinsic + 4);
      }

      ceres::Solver::Options options;
      options.preconditioner_type = ceres::JACOBI;
      options.linear_solver_type = ceres::SPARSE_SCHUR;
      options.minimizer_progress_to_stdout = true; // 关闭控制台输出
      options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
      options.logging_type = ceres::PER_MINIMIZER_ITERATION;

      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      // std::cout << summary.BriefReport() << std::endl;
      PRINT_DEBUG(summary.BriefReport().c_str());
      // std::cout << summary.FullReport() << std::endl;

      Eigen::Matrix3d r_l_to_c_opt =
          Eigen::Map<Eigen::Quaterniond>(extrinsic).toRotationMatrix();
      Eigen::Vector3d t_l_in_c_opt = Eigen::Map<Eigen::Vector3d>(extrinsic + 4);

      Eigen::JacobiSVD<Eigen::Matrix3d> svd(
          r_l_to_c_opt, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix3d r_l_to_c_opt_orthogonal =
          svd.matrixU() * svd.matrixV().transpose();

      PRINT_INFO(GREEN "r_distance(deg):%f, t_distance:%f\n" RESET,
                 RAD2DEG(Eigen::Quaterniond(r_l_to_c_opt_orthogonal)
                             .angularDistance(corrected_q_l_to_c)),
                 (t_l_in_c_opt - corrected_t_l_in_c_).norm());
      PRINT_INFO(
          GREEN "r_distance_init:%f, t_distance_init:%ff\n" RESET,
          RAD2DEG(Eigen::Quaterniond(r_l_to_c_opt_orthogonal)
                      .angularDistance(Eigen::Quaterniond(init_r_l_to_c_))),
          (corrected_t_l_in_c_ - init_t_l_in_c_).norm())

      corrected_r_l_to_c_ = r_l_to_c_opt_orthogonal;
      corrected_t_l_in_c_ = t_l_in_c_opt;

      // if (RAD2DEG(Eigen::Quaterniond(corrected_r_l_to_c_)
      //                 .angularDistance(q_l_to_c_init)) < 0.01 &&
      //     (corrected_t_l_in_c_ - t_l_in_c_init).norm() < 0.002) {
      //   opt_flag = false;
      // }
    }
    // if (!opt_flag)
    //   break;
  }

  // ColorRawPointCloudByRgbImage(
  //     init_r_l_to_c_, init_t_l_in_c_,
  //     params_.colored_point_cloud_dense_threshold,
  //     params_.colored_point_cloud_min_intensity_threshold,
  //     params_.colored_point_cloud_min_depth_threshold,
  //     params_.colored_point_cloud_max_depth_threshold, rgb_image_,
  //     raw_point_cloud, colored_point_cloud_);

  // pcl::io::savePCDFileASCII(params_.debug_result_path +
  //                               "colored_point_cloud.pcd",
  //                           *colored_point_cloud_);

  // cv::Mat init_projection_image;
  // GetProjectionImage(init_r_l_to_c_, init_t_l_in_c_, ProjectionType
  // ::INTENSITY,
  //                    params_.colored_point_cloud_min_depth_threshold,
  //                    params_.colored_point_cloud_max_depth_threshold,
  //                    false, rgb_image_, raw_point_cloud,
  //                    init_projection_image);
  // cv::imwrite(params_.debug_result_path +
  // "merged_init_projection_image.png",
  //             init_projection_image);

  return;
}

LidarVisualAlignment::~LidarVisualAlignment() {}

void LidarVisualAlignment::ImageEdgeLineGenerator(
    const int &gaussian_blur_ksize, const int &canny_threshold,
    const int &edge_line_contour_min_length_threshold, const cv::Mat &src_img,
    cv::Mat &edge_line_contour_image,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &image_edge_line_point_cloud) {
  PRINT_INFO(GREEN "Generating image edge line point_cloud...\n" RESET);

  // 第四和第五个参数：高斯核在 X 和 Y
  // 方向的标准差，0表示根据核大小自动计算。
  cv::GaussianBlur(src_img, src_img,
                   cv::Size(gaussian_blur_ksize, gaussian_blur_ksize), 0, 0);
  //   cv::imshow("gaussian blur image", src_img);
  //   cv::waitKey();
  cv::imwrite(params_.debug_result_path + "gaussian_blur_image.png", src_img);
  cv::Mat canny_result =
      cv::Mat::zeros(params_.image_height, params_.image_width, CV_8UC1);

  // 第三个参数：边缘检测的阈值，越大越容易检测到边缘，但是也会漏掉一些边缘。
  // 第四个参数：边缘检测的突变值，越大越容易检测到边缘，但是也会漏掉一些边缘。
  // 第五个参数：Sobel内核大小，这里设置为 3，表示 3x3 大小的 Sobel 内核。
  // 第六个参数：是否使用 L2 范数来计算梯度，这里设置为 true，表示使用 L2
  // 范数。
  cv::Canny(src_img, canny_result, canny_threshold, canny_threshold * 3, 3,
            true);
  //   cv::imshow("canny image", canny_result);
  //   cv::waitKey();
  cv::imwrite(params_.debug_result_path + "canny_image.png", canny_result);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  // 第一个参数：输入图像（Canny 边缘检测结果）。
  // 第二个参数：输出的轮廓（点集）。
  // 第三个参数：输出的层级（每个轮廓对应的父轮廓）。
  // 第四个参数：轮廓检索模式，这里设置为
  // RETR_EXTERNAL，表示只检索最外层轮廓。
  // 第五个参数：轮廓近似方法，这里设置为
  // CHAIN_APPROX_NONE，表示不进行轮廓近似。 第六个参数：偏移量，这里设置为
  // (0,0)，表示从图像左上角开始计数，不偏移
  cv::findContours(canny_result, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
  image_edge_line_point_cloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  edge_line_contour_image =
      cv::Mat::zeros(params_.image_height, params_.image_width, CV_8UC1);
  for (int i = 0; i < contours.size(); i++) {
    if (contours[i].size() < edge_line_contour_min_length_threshold)
      continue;
    for (int j = 0; j < contours[i].size(); j++) {
      pcl::PointXYZ point;
      point.x = contours[i][j].x;
      point.y = -contours[i][j].y;
      point.z = 0;
      image_edge_line_point_cloud->points.push_back(point);
      edge_line_contour_image.at<uchar>(contours[i][j].y, contours[i][j].x) =
          255;
    }
  }

  image_edge_line_point_cloud->width =
      image_edge_line_point_cloud->points.size();
  image_edge_line_point_cloud->height = 1;
  pcl::io::savePCDFileASCII(params_.debug_result_path +
                                "image_edge_line_point_cloud.pcd",
                            *image_edge_line_point_cloud);
  cv::imwrite(params_.debug_result_path + "image_edge_line_contour.png",
              edge_line_contour_image);
  PRINT_INFO(GREEN "Successfully generate image edge line point_cloud, point "
                   "size:%d\n" RESET,
             image_edge_line_point_cloud->size());
  return;
}

void LidarVisualAlignment::BuildVoxelMapFromRawPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_point_cloud,
    const float &voxel_size,
    std::unordered_map<VoxelPosition, Voxel::Ptr> &voxel_map) {

  PRINT_INFO(GREEN "Building voxel map from raw point cloud...\n" RESET);

  pcl::PointCloud<pcl::PointXYZRGB> test_point_cloud;
  test_point_cloud.reserve(input_point_cloud->size());

  std::mt19937 rng(std::random_device{}());
  std::uniform_int_distribution<int> dist(0, 255);

  for (const auto &p_c : *input_point_cloud) {
    // 使用整数运算(向下取整)避免浮点数精度问题
    int64_t ix = static_cast<int64_t>(std::floor(p_c.x / voxel_size));
    int64_t iy = static_cast<int64_t>(std::floor(p_c.y / voxel_size));
    int64_t iz = static_cast<int64_t>(std::floor(p_c.z / voxel_size));
    VoxelPosition position(ix, iy, iz);
    auto iter = voxel_map.find(position);
    if (iter == voxel_map.end()) {
      Voxel::Ptr voxel = std::make_shared<Voxel>(voxel_size);
      voxel->voxel_origin << ix * voxel_size, iy * voxel_size, iz * voxel_size;
      voxel->voxel_color << dist(rng), dist(rng), dist(rng);
      voxel->point_cloud->emplace_back(p_c);
      voxel_map.emplace(position, voxel);
    } else {
      iter->second->point_cloud->emplace_back(p_c);

      pcl::PointXYZRGB p_rgb;
      p_rgb.x = p_c.x;
      p_rgb.y = p_c.y;
      p_rgb.z = p_c.z;
      p_rgb.r = static_cast<uint8_t>(iter->second->voxel_color.x());
      p_rgb.g = static_cast<uint8_t>(iter->second->voxel_color.y());
      p_rgb.b = static_cast<uint8_t>(iter->second->voxel_color.z());
      test_point_cloud.emplace_back(p_rgb);
    }
  }
  test_point_cloud.reserve(test_point_cloud.size());
  pcl::io::savePCDFileASCII(params_.debug_result_path + "voxel_map.pcd",
                            test_point_cloud);
  PRINT_INFO(GREEN "Successfully build voxel map from raw point cloud, voxel "
                   "map size: %d, total point cloud size: %d\n" RESET,
             voxel_map.size(), input_point_cloud->size());
  return;
}

void LidarVisualAlignment::DownsampleVoxelMap(
    std::unordered_map<VoxelPosition, Voxel::Ptr> &voxel_map) {
  PRINT_INFO(GREEN "Downsampling voxel map...\n" RESET);
  int total_num_points_before = 0;
  int total_num_points_after = 0;
#pragma omp parallel for
  for (auto it = voxel_map.begin(); it != voxel_map.end(); ++it) {
    if (it->second->point_cloud->size() > 20) {
      int num_points_before = it->second->point_cloud->size();
      total_num_points_before += num_points_before;
      DownsampleVoxelGrid(*(it->second->point_cloud), 0.02f);
      int num_points_after = it->second->point_cloud->size();
      total_num_points_after += num_points_after;
      // PRINT_DEBUG(BLUE "Voxel ID [%lld, %lld, %lld]: downsampled point
      // cloud
      // "
      //                  "from %d to %d points.\n" RESET,
      //             it->first.x, it->first.y, it->first.z,
      //             num_points_before, num_points_after);
    } else {
      total_num_points_before += it->second->point_cloud->size();
    }
  }
  PRINT_INFO(GREEN "Successfully downsample voxel map, voxel map size: %d, "
                   "total downsampled point cloud from %d to %d points\n" RESET,
             voxel_map.size(), total_num_points_before, total_num_points_after);
  return;
}

// 1. For each voxel, we repeatedly use RANSAC to fit and extract planes
//    contained in the voxel.
// 2. Then, we retain plane pairs that are connected and form an angle
// within a
//    certain range (e.g., [30°, 150°]) and solve for the plane intersection
//    lines (i.e., the depth-continuous edge).
void LidarVisualAlignment::LidarEdgeLineExtractor(
    const std::unordered_map<VoxelPosition, Voxel::Ptr> &voxel_map,
    const float &plane_ransac_distance_threshold,
    const int &plane_point_size_threshold,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_edge_line_point_cloud) {
  PRINT_INFO(GREEN
             "Building lidar edge line point cloud from voxel map...\n" RESET);
  lidar_edge_line_point_cloud = PointcloudXYZIPtr(new PointcloudXYZI);

  int voxel_num = 0;
  PointcloudXYZRGBPtr test_lidar_plane_point_cloud(new PointcloudXYZRGB);
  for (const auto &voxel_pair : voxel_map) {
    voxel_num++;
    if (voxel_pair.second->point_cloud->size() <= 50)
      continue;
    // 定义平面集合
    std::vector<Plane> plane_list;

    PointcloudXYZIPtr filtered_point_cloud(new PointcloudXYZI);
    pcl::copyPointCloud(*(voxel_pair.second->point_cloud),
                        *filtered_point_cloud);

    // 设置平面分割的参数
    // 创建一个分割器
    pcl::SACSegmentation<pcl::PointXYZI> segmentation;
    // Optional,设置结果平面展示的点是分割掉的点还是分割剩下的点
    segmentation.setOptimizeCoefficients(true);
    // Mandatory-设置目标几何形状，这里是平面
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    // 分割方法：随机采样法
    segmentation.setMethodType(pcl::SAC_RANSAC);
    // 设置误差容忍范围，也就是阈值
    segmentation.setDistanceThreshold(plane_ransac_distance_threshold);
    // 设置最大迭代次数
    segmentation.setMaxIterations(500);

    PointcloudXYZRGB colored_plane_point_cloud;
    int plane_index = 0;

    // 不断分割平面，直到剩余点云过少
    while (filtered_point_cloud->points.size() > 10) {
      // 创建一个模型参数对象，用于记录结果
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      // inliers表示误差能容忍的点，记录点云序号
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

      segmentation.setInputCloud(filtered_point_cloud);
      segmentation.segment(*inliers, *coefficients);

      if (inliers->indices.empty()) {
        PRINT_ERROR(RED "Could not estimate a planner model for the given "
                        "dataset\n" RESET);
        break;
      }

      // 提取当前平面内的点云
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      extract.setIndices(inliers);
      extract.setInputCloud(filtered_point_cloud);

      pcl::PointCloud<pcl::PointXYZI> plane_cloud;
      extract.filter(plane_cloud);

      // 如果平面内点数量大于阈值，保存平面信息
      if (plane_cloud.size() > plane_point_size_threshold) {
        // pcl::PointCloud<pcl::PointXYZRGB> color_cloud;

        uint8_t r = static_cast<uint8_t>(rand() % 256);
        uint8_t g = static_cast<uint8_t>(rand() % 256);
        uint8_t b = static_cast<uint8_t>(rand() % 256);

        pcl::PointXYZ center_point(0.0f, 0.0f, 0.0f);
        for (const auto &point : plane_cloud.points) {
          pcl::PointXYZRGB colored_point;
          colored_point.x = point.x;
          colored_point.y = point.y;
          colored_point.z = point.z;
          center_point.x += point.x;
          center_point.y += point.y;
          center_point.z += point.z;
          colored_point.r = r;
          colored_point.g = g;
          colored_point.b = b;
          // color_cloud.push_back(colored_point);
          colored_plane_point_cloud.push_back(colored_point);
          test_lidar_plane_point_cloud->push_back(colored_point);
        }
        center_point.x /= plane_cloud.size();
        center_point.y /= plane_cloud.size();
        center_point.z /= plane_cloud.size();

        Plane plane;
        plane.point_cloud = plane_cloud;
        plane.point_center = center_point;
        plane.normal << coefficients->values[0], coefficients->values[1],
            coefficients->values[2];
        plane.index = plane_index;
        plane_list.push_back(plane);
        plane_index++;
      }

      // 从点云中移除当前平面点,将剩下的点云重复进行提取，直到不满足条件
      extract.setNegative(true);
      pcl::PointCloud<pcl::PointXYZI> remaining_cloud;
      extract.filter(remaining_cloud);
      *filtered_point_cloud = remaining_cloud;

      // if (colored_plane_point_cloud.size() > 0) {
      //   pcl::io::savePCDFileASCII(params_.debug_result_path +
      //                                 "/corlored_plane_point_cloud/"
      //                                 "corlored_plane_point_cloud_" +
      //                                 std::to_string(voxel_num) + "_" +
      //                                 std::to_string(plane_index) +
      //                                 ".pcd",
      //                             colored_plane_point_cloud);
      // }
    }

    // 如果找到两个以上的平面，计算交线
    if (plane_list.size() < params_.plane_min_size_threshold ||
        plane_list.size() > params_.plane_max_size_threshold)
      continue;

    std::vector<pcl::PointCloud<pcl::PointXYZI>> line_point_clouds;
    CalculateLineFromPlaneIntersection(plane_list, params_.voxel_size,
                                       voxel_pair.second->voxel_origin,
                                       line_point_clouds);
    // PRINT_DEBUG(BLUE "Voxel ID [%lld, %lld, %lld]: lidar plane size: %d,
    // edge
    // "
    //                  "line size: % d\n" RESET,
    //             voxel_pair.first.x, voxel_pair.first.y, voxel_pair.first.z,
    //             plane_list.size(), line_point_clouds.size());

    if (line_point_clouds.empty() || line_point_clouds.size() > 8)
      continue;

    for (size_t i = 0; i < line_point_clouds.size(); i++) {
      for (size_t j = 0; j < line_point_clouds[i].size(); j++) {
        pcl::PointXYZI p = line_point_clouds[i].points[j];
        lidar_edge_line_point_cloud->points.push_back(p);
        lidar_depth_continuous_edge_line_number_.push_back(
            lidar_depth_continuous_edge_line_number_total_);
      }
      lidar_depth_continuous_edge_line_number_total_++;
    }
  }
  lidar_edge_line_point_cloud->width =
      lidar_edge_line_point_cloud->points.size();
  lidar_edge_line_point_cloud->height = 1;
  pcl::io::savePCDFileASCII(params_.debug_result_path +
                                "lidar_plane_point_cloud.pcd",
                            *test_lidar_plane_point_cloud);
  PRINT_INFO(
      GREEN
      "Successfully generate lidar edge line point cloud, size: %d\n" RESET,
      lidar_edge_line_point_cloud->points.size());
  return;
}

// 计算平面交线的函数
void LidarVisualAlignment::CalculateLineFromPlaneIntersection(
    const std::vector<Plane> &plane_list, const float &voxel_size,
    const Eigen::Vector3d &voxel_origin,
    std::vector<pcl::PointCloud<pcl::PointXYZI>> &line_point_clouds) {

  size_t plane_size = plane_list.size();
  // 预构建所有平面的 KdTree
  std::vector<pcl::search::KdTree<pcl::PointXYZI>::Ptr> kdtree_list;
  kdtree_list.reserve(plane_size);
  for (const auto &plane : plane_list) {
    auto kdtree = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
    kdtree->setInputCloud(plane.point_cloud.makeShared());
    kdtree_list.push_back(kdtree);
  }

  // 遍历所有平面对，双指针
#pragma omp parallel for schedule(dynamic) collapse(2)
  for (size_t i = 0; i < plane_list.size() - 1; ++i) {
    for (size_t j = i + 1; j < plane_list.size(); ++j) {
      const Plane &plane1 = plane_list[i];
      const Plane &plane2 = plane_list[j];

      if (plane1.point_cloud.empty() || plane2.point_cloud.empty()) {
        continue;
      }

      // 计算法向量的内积，判断平面是否相交
      float dot_product = plane1.normal.dot(plane2.normal);

      // if (dot_product <= theta_max_ || dot_product >= theta_min_)
      //   continue;

      // 如果找到两个以上的平面，计算交线
      if (dot_product >= params_.plane_connection_min_cos_threshold ||
          dot_product <= params_.plane_connection_max_cos_threshold)
        continue;

      // // 构造线性方程组，求解平面交线与体素边界面的交点
      // Eigen::Matrix<double, 3, 4> matrix;
      // matrix(0, 0) = plane1.normal[0];
      // matrix(0, 1) = plane1.normal[1];
      // matrix(0, 2) = plane1.normal[2];
      // matrix(0, 3) = plane1.normal.dot(Eigen::Vector3d(
      //     plane1.point_center.x, plane1.point_center.y,
      //     plane1.point_center.z));

      // matrix(1, 0) = plane2.normal[0];
      // matrix(1, 1) = plane2.normal[1];
      // matrix(1, 2) = plane2.normal[2];
      // matrix(1, 3) = plane2.normal.dot(Eigen::Vector3d(
      //     plane2.point_center.x, plane2.point_center.y,
      //     plane2.point_center.z));

      // std::vector<Eigen::Vector3d> intersections;
      // Eigen::Vector3d point;
      // double boundaries[3] = {voxel_origin[0], voxel_origin[1],
      //                         voxel_origin[2]};

      // //
      // 如果体素内两个平面存在交线，那么该交线一定穿过两个体素的两个平面，形成交点
      // // 所以该点满足三个条件：
      // // 1. 在平面1上（方程一）；
      // // 2. 在平面2上（方程二）；
      // // 3. 在voxel的某个面上（方程三）；
      // for (int axis = 0; axis < 3; ++axis) {
      //   matrix(2, 0) = (axis == 0) ? 1.0f : 0.0f;
      //   matrix(2, 1) = (axis == 1) ? 1.0f : 0.0f;
      //   matrix(2, 2) = (axis == 2) ? 1.0f : 0.0f;
      //   matrix(2, 3) = boundaries[axis];
      //   if (!CramerSolve(matrix.block<3, 3>(0, 0).eval(),
      //   matrix.col(3).eval(),
      //                    point))
      //     continue;
      //   // Eigen::Vector3d test_point_1;
      //   // SolveLinearSystem(matrix, test_point_1);
      //   // PRINT_DEBUG("test solver 1:\n%s\n",
      //   //             EigenMatrixToString(test_point_1).c_str());
      //   // PRINT_DEBUG("solver 1:\n%s\n",
      //   EigenMatrixToString(point).c_str());

      //   if ((point.array() >= voxel_origin.array() - 0.0f).all() &&
      //       (point.array() <= voxel_origin.array() + voxel_size +
      //       0.0f).all()) {
      //     intersections.push_back(point);
      //   }

      //   matrix(2, 3) = boundaries[axis] + voxel_size;
      //   if (!CramerSolve(matrix.block<3, 3>(0, 0).eval(),
      //   matrix.col(3).eval(),
      //                    point))
      //     continue;

      //   // Eigen::Vector3d test_point_2;
      //   // SolveLinearSystem(matrix, test_point_2);
      //   // PRINT_DEBUG("test solver 2:\n%s\n",
      //   //             EigenMatrixToString(test_point_2).c_str());
      //   // PRINT_DEBUG("solver 2:\n%s\n",
      //   EigenMatrixToString(point).c_str()); if ((point.array() >=
      //   voxel_origin.array() - 0.0f).all() &&
      //       (point.array() <= voxel_origin.array() + voxel_size +
      //       0.0f).all()) {
      //     intersections.push_back(point);
      //   }
      // }

      Eigen::Matrix<double, 3, 3> A;
      Eigen::Vector3d b;

      A.row(0) = plane1.normal;
      b(0) = plane1.normal.dot(Eigen::Vector3d(
          plane1.point_center.x, plane1.point_center.y, plane1.point_center.z));

      A.row(1) = plane2.normal;
      b(1) = plane2.normal.dot(Eigen::Vector3d(
          plane2.point_center.x, plane2.point_center.y, plane2.point_center.z));

      std::vector<Eigen::Vector3d> intersections;

      // For each boundary plane of the voxel
      for (int axis = 0; axis < 3; ++axis) {
        for (int offset = 0; offset <= 1; ++offset) {
          A.row(2).setZero();
          A(2, axis) = 1.0;
          b(2) = voxel_origin[axis] + offset * voxel_size;

          // Solve the linear system
          Eigen::Vector3d point;
          if (!SolveLinearSystemCramer(A, b, point))
            continue;

          // Check if the point is within the voxel
          if ((point.array() >= voxel_origin.array()).all() &&
              (point.array() <= (voxel_origin.array() + voxel_size)).all()) {
            intersections.push_back(point);
          }
        }
      }

      // PRINT_DEBUG(BLUE "intersections size: %d\n" RESET,
      // intersections.size())

      // 如果points中恰好有两个点，表示成功找到了两平面的交点，代码将这两个点连接成一条直线。
      // 计算这条直线的长度，并沿直线以步长 0.005 进行采样，生成多个中间点。
      // 对每个采样点，使用 KdTree
      // 进行最近邻搜索，检查该点是否靠近两个平面的点云数据。
      // 满足距离阈值条件的点被添加到 line_cloud 中。
      if (intersections.size() != 2)
        continue;

      Eigen::Vector3d &p1 = intersections[0];
      Eigen::Vector3d &p2 = intersections[1];
      float length = (p2 - p1).norm();

      if (length == 0.0f)
        continue;

      float inv_length = 1.0f / length;
      float dx = (p2[0] - p1[0]) * inv_length;
      float dy = (p2[1] - p1[1]) * inv_length;
      float dz = (p2[2] - p1[2]) * inv_length;

      PointcloudXYZI line_point_cloud;
      line_point_cloud.reserve(static_cast<size_t>(length / 0.005f) * 2);

      // pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree1(
      //     new pcl::search::KdTree<pcl::PointXYZI>());
      // pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree2(
      //     new pcl::search::KdTree<pcl::PointXYZI>());

      // kdtree1->setInputCloud(plane1.point_cloud.makeShared());
      // kdtree2->setInputCloud(plane2.point_cloud.makeShared());

      auto kdtree1 = kdtree_list[i];
      auto kdtree2 = kdtree_list[j];

      std::vector<int> indices1(1), indices2(1);
      std::vector<float> sqr_distances1(1), sqr_distances2(1);

      for (float t = 0.0f; t <= length; t += 0.005f) {
        pcl::PointXYZI p;
        p.x = p1[0] + dx * t;
        p.y = p1[1] + dy * t;
        p.z = p1[2] + dz * t;
        p.intensity = 100;

        if (kdtree1->nearestKSearch(p, 1, indices1, sqr_distances1) > 0 &&
            kdtree2->nearestKSearch(p, 1, indices2, sqr_distances2) > 0) {

          // float dist1 = (p.x - plane1.point_cloud.points[indices1[0]].x)
          // *
          //                   (p.x -
          //                   plane1.point_cloud.points[indices1[0]].x)
          //                   +
          //               (p.y - plane1.point_cloud.points[indices1[0]].y)
          //               *
          //                   (p.y -
          //                   plane1.point_cloud.points[indices1[0]].y)
          //                   +
          //               (p.z - plane1.point_cloud.points[indices1[0]].z)
          //               *
          //                   (p.z -
          //                   plane1.point_cloud.points[indices1[0]].z);
          // PRINT_DEBUG(GREEN "dist1: %f, sqr_distances1: %f\n" RESET,
          // dist1,
          //             sqr_distances1[0])
          float dist1 = sqr_distances1[0];

          // float dist2 = (p.x - plane2.point_cloud.points[indices2[0]].x)
          // *
          //                   (p.x -
          //                   plane2.point_cloud.points[indices2[0]].x)
          //                   +
          //               (p.y - plane2.point_cloud.points[indices2[0]].y)
          //               *
          //                   (p.y -
          //                   plane2.point_cloud.points[indices2[0]].y)
          //                   +
          //               (p.z - plane2.point_cloud.points[indices2[0]].z)
          //               *
          //                   (p.z -
          //                   plane2.point_cloud.points[indices2[0]].z);
          // PRINT_DEBUG(GREEN "dist2: %f, sqr_distances2: %f\n" RESET,
          // dist2,
          //             sqr_distances2[0])
          float dist2 = sqr_distances2[0];

          // 1. 确保点p到第一个平面的距离非常小（在最小阈值内），
          //    同时到第二个平面的距离不超过最大阈值。
          // 2. 确保点 p 到第二个平面的距离非常小，
          //    同时到第一个平面的距离不超过最大阈值
          if ((dist1 < params_.point_to_plane_min_distance_threshold_sqaured &&
               dist2 < params_.point_to_plane_max_distance_threshold_sqaured) ||
              (dist2 < params_.point_to_plane_min_distance_threshold_sqaured &&
               dist1 < params_.point_to_plane_max_distance_threshold_sqaured)) {
            line_point_cloud.push_back(p);
          }
        }
      }

      if (line_point_cloud.size() > 10) {
#pragma omp critical
        { line_point_clouds.push_back(line_point_cloud); }
      }
    }
  }
  return;
}

void LidarVisualAlignment::BuildDirectionalVectorPnP(
    const Eigen::Matrix3d &r_l_to_c, const Eigen::Vector3d &t_l_in_c,
    const int &distance_threshold,
    const PointcloudXYZPtr &image_edge_line_point_cloud,
    const PointcloudXYZIPtr &lidar_edge_line_point_cloud,
    const std::vector<int> &lidar_edge_line_number,
    const bool show_residual_image,
    std::vector<DirectionPnPData> &direction_pnp_data_vec) {

  PRINT_INFO(GREEN "Building directional vector PnP...\n" RESET);
  if (!direction_pnp_data_vec.empty())
    direction_pnp_data_vec.clear();

  std::vector<std::vector<std::vector<pcl::PointXYZI>>> image_pts_container;
  for (int y = 0; y < params_.image_height; y++) {
    std::vector<std::vector<pcl::PointXYZI>> row_pts_container;
    for (int x = 0; x < params_.image_width; x++) {
      std::vector<pcl::PointXYZI> col_pts_container;
      row_pts_container.push_back(col_pts_container);
    }
    image_pts_container.push_back(row_pts_container);
  }

  // 2. 3D点投影到图像平面得到2D点
  std::vector<cv::Point3f> lidar_edge_line_points_3d;
  std::vector<cv::Point2f> projected_image_edge_line_points_2d;
  projected_image_edge_line_points_2d.reserve(
      lidar_edge_line_point_cloud->size());
  for (const auto &point_3d : lidar_edge_line_point_cloud->points) {
    Vec3d p_l = Vec3d(point_3d.x, point_3d.y, point_3d.z);
    Vec3d p_c = r_l_to_c * p_l + t_l_in_c;
    if (p_c(2) < 0)
      continue;
    Vec2d uv = camera_model_->Camera2Pixel(p_c);
    if (!camera_model_->isInFrame(uv, 1))
      continue;
    lidar_edge_line_points_3d.emplace_back(
        cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
    projected_image_edge_line_points_2d.emplace_back(cv::Point2f(uv(0), uv(1)));
  }

  // cv::Mat r_vec;
  // cv::eigen2cv(r_l_to_c, r_vec);

  // cv::Mat t_vec =
  //     (cv::Mat_<double>(3, 1) << t_l_in_c(0), t_l_in_c(1), t_l_in_c(2));

  // cv::Mat camera_intrinsics =
  //     (cv::Mat_<double>(3, 3) << params_.fx, 0.0, params_.cx, 0.0,
  //     params_.fy,
  //      params_.cy, 0.0, 0.0, 1.0);

  // cv::Mat distortion_coeffs = (cv::Mat_<double>(1, 5) << params_.k1,
  // params_.k2,
  //                              params_.p1, params_.p2, params_.k3);

  // cv::projectPoints(lidar_edge_line_points_3d, r_vec, t_vec,
  // camera_intrinsics,
  //                   distortion_coeffs,
  //                   projected_image_edge_line_points_2d);
  PRINT_DEBUG(BLUE "projected image edge line points size: %d\n" RESET,
              projected_image_edge_line_points_2d.size())

  // 3. 构建投影后的2D点云
  PointcloudXYZPtr projected_image_edge_line_point_cloud(new PointcloudXYZ);
  // 属于哪跟线
  std::vector<int> projected_image_edge_line_number;
  for (size_t i = 0; i < projected_image_edge_line_points_2d.size(); i++) {
    const cv::Point2f &pt_2d = projected_image_edge_line_points_2d[i];
    const cv::Point3f &pt_3d = lidar_edge_line_points_3d[i];
    pcl::PointXYZ p_2d;
    p_2d.x = pt_2d.x;
    p_2d.y = -pt_2d.y;
    p_2d.z = 0;
    pcl::PointXYZI p_3d;
    p_3d.x = pt_3d.x;
    p_3d.y = pt_3d.y;
    p_3d.z = pt_3d.z;
    p_3d.intensity = 1;
    // if (pt_2d.x <= 0 || pt_2d.x >= params_.image_width || pt_2d.y <= 0 ||
    //     pt_2d.y >= params_.image_height)
    if (!CheckFov(pt_2d))
      continue;
    // TODO：是否需要line_edge_cloud_2d和line_edge_cloud_2d_number
    if (image_pts_container[pt_2d.y][pt_2d.x].size() == 0) {
      projected_image_edge_line_point_cloud->points.push_back(p_2d);
      projected_image_edge_line_number.push_back(1);
      // projected_image_edge_line_number.push_back(lidar_edge_line_number[i]);
      image_pts_container[pt_2d.y][pt_2d.x].push_back(p_3d);
    } else {
      image_pts_container[pt_2d.y][pt_2d.x].push_back(p_3d);
    }
  }

  PRINT_DEBUG(BLUE "projected image edge line point cloud size: %d\n" RESET,
              projected_image_edge_line_point_cloud->points.size());

  if (show_residual_image) {
    cv::Mat residual_img =
        GetResidualImage(distance_threshold, image_edge_line_point_cloud,
                         projected_image_edge_line_point_cloud);
    cv::imwrite(params_.debug_result_path + "residual_image_" +
                    std::to_string(distance_threshold) + ".png",
                residual_img);
    // cv::imshow("residual", residual_img);
    // cv::waitKey();
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_image(
      new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_projected_image(
      new pcl::search::KdTree<pcl::PointXYZ>());
  kdtree_image->setInputCloud(image_edge_line_point_cloud);
  kdtree_projected_image->setInputCloud(projected_image_edge_line_point_cloud);

  // 之所以指定5个，是需要计算image和projected_image平面的方向向量
  int k = 5;
  std::vector<int> point_idx_knn_search_image(k);
  std::vector<float> point_knn_squared_distance_image(k);
  std::vector<int> point_idx_knn_search_projected_image(k);
  std::vector<float> point_knn_squared_distance_projected_image(k);

  std::vector<cv::Point2d> projected_image_point_2d_vec;
  std::vector<cv::Point2d> image_point_2d_vec;
  std::vector<Eigen::Vector2d> image_direction_vec;
  std::vector<Eigen::Vector2d> projected_image_direction_vec;
  std::vector<int> projected_image_point_2d_line_number;

  for (size_t i = 0; i < projected_image_edge_line_point_cloud->points.size();
       i++) {
    const pcl::PointXYZ &search_point =
        projected_image_edge_line_point_cloud->points[i];
    if ((kdtree_image->nearestKSearch(search_point, k,
                                      point_idx_knn_search_image,
                                      point_knn_squared_distance_image) > 0) &&
        // 该搜索用于方向估计
        (kdtree_projected_image->nearestKSearch(
             search_point, k, point_idx_knn_search_projected_image,
             point_knn_squared_distance_projected_image) > 0)) {
      bool distance_check = true;
      for (int j = 0; j < k; j++) {
        float distance = std::sqrt(point_knn_squared_distance_image[j]);
        if (distance > distance_threshold) {
          distance_check = false;
          break;
        }
      }
      if (!distance_check)
        continue;

      // PRINT_DEBUG(BLUE "serch image point size: %d\n" RESET,
      //             point_idx_knn_search_image.size())

      std::vector<Eigen::Vector2d> search_image_points;
      search_image_points.reserve(point_idx_knn_search_image.size());
      for (size_t index = 0; index < point_idx_knn_search_image.size();
           index++) {
        const pcl::PointXYZ &pt =
            image_edge_line_point_cloud
                ->points[point_idx_knn_search_image[index]];
        search_image_points.emplace_back(Eigen::Vector2d(pt.x, -pt.y));
      }
      Eigen::Vector2d image_direction(0, 0);
      CalculatePrincipalDirection(search_image_points, image_direction);

      std::vector<Eigen::Vector2d> search_projected_image_points;
      search_projected_image_points.reserve(
          point_idx_knn_search_projected_image.size());
      for (size_t index = 0;
           index < point_idx_knn_search_projected_image.size(); index++) {
        const pcl::PointXYZ &pt =
            projected_image_edge_line_point_cloud
                ->points[point_idx_knn_search_projected_image[index]];
        search_projected_image_points.emplace_back(
            Eigen::Vector2d(pt.x, -pt.y));
      }
      Eigen::Vector2d projected_image_direction(0, 0);
      CalculatePrincipalDirection(search_projected_image_points,
                                  projected_image_direction);

      cv::Point2d projected_image_point_2d(search_point.x, -search_point.y);
      // 对当前点projected_image_point来说，只需要找最近的image点即可，这里的序号为0。
      cv::Point2d image_point_2d(
          image_edge_line_point_cloud->points[point_idx_knn_search_image[0]].x,
          -image_edge_line_point_cloud->points[point_idx_knn_search_image[0]]
               .y);
      if (!CheckFov(projected_image_point_2d)) {
        PRINT_DEBUG(RED "x: %f, y: %f, out of fov\n" RESET,
                    projected_image_point_2d.x, projected_image_point_2d.y);
        continue;
      }
      projected_image_point_2d_vec.emplace_back(projected_image_point_2d);
      image_point_2d_vec.emplace_back(image_point_2d);
      image_direction_vec.emplace_back(image_direction);
      projected_image_direction_vec.emplace_back(projected_image_direction);
      projected_image_point_2d_line_number.emplace_back(
          projected_image_edge_line_number[i]);
    }
  }

  // 构建VPnPData列表
  for (size_t i = 0; i < projected_image_point_2d_vec.size(); i++) {
    int y = projected_image_point_2d_vec[i].y;
    int x = projected_image_point_2d_vec[i].x;
    if (image_pts_container[y][x].empty())
      continue;

    int pixel_points_size = image_pts_container[y][x].size();
    DirectionPnPData data;

    data.image_direction = image_direction_vec[i];
    data.projected_image_direction = projected_image_direction_vec[i];
    float cos_theta = data.image_direction.dot(data.projected_image_direction);
    if (cos_theta <= params_.plane_connection_min_cos_threshold &&
        cos_theta >= params_.plane_connection_max_cos_threshold)
      continue;
    data.x = 0;
    data.y = 0;
    data.z = 0;
    data.u = image_point_2d_vec[i].x;
    data.v = image_point_2d_vec[i].y;
    for (size_t j = 0; j < pixel_points_size; j++) {
      data.x += image_pts_container[y][x][j].x;
      data.y += image_pts_container[y][x][j].y;
      data.z += image_pts_container[y][x][j].z;
    }
    data.x = data.x / pixel_points_size;
    data.y = data.y / pixel_points_size;
    data.z = data.z / pixel_points_size;

    data.line_number = projected_image_point_2d_line_number[i];
    direction_pnp_data_vec.push_back(data);
  }
  // PRINT_INFO("Successfully build direction pnp data, size: %d\n",
  //            direction_pnp_data_vec.size());
  return;
}

void LidarVisualAlignment::ColorRawPointCloudByRgbImage(
    const Eigen::Matrix3d &r_l_to_c, const Eigen::Vector3d &t_l_in_c,
    const int &density, const int &min_intensity_threshold,
    const float &min_depth_threshold, const float &max_depth_threshold,
    const cv::Mat &rgb_image,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &raw_point_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_point_cloud) {

  PRINT_INFO("Coloring raw point cloud by RGB image...\n");

  std::vector<cv::Point3f> selected_lidar_points_3d;
  selected_lidar_points_3d.reserve(raw_point_cloud->size() / density);
  for (size_t i = 0; i < raw_point_cloud->size(); i += density) {
    const auto &point = raw_point_cloud->points[i];
    float depth =
        std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (depth > min_depth_threshold && depth < max_depth_threshold &&
        point.intensity >= min_intensity_threshold) {
      selected_lidar_points_3d.emplace_back(
          cv::Point3f(point.x, point.y, point.z));
    }
  }

  cv::Mat r_vec;
  cv::eigen2cv(r_l_to_c, r_vec);

  // const的数据无法使用下面的方法将Eigen矩阵转换为cv::Mat
  // cv::Mat r_vec(3, 3, CV_64F, r_l_to_c.data());

  cv::Mat t_vec =
      (cv::Mat_<double>(3, 1) << t_l_in_c(0), t_l_in_c(1), t_l_in_c(2));

  // 相机内参和畸变系数
  // cv::Mat camera_intrinsics =
  //     (cv::Mat_<double>(3, 3) << params_.fx, 0.0, params_.cx, 0.0,
  //     params_.fy,
  //      params_.cy, 0.0, 0.0, 1.0);
  cv::Mat camera_intrinsics(3, 3, CV_64F, params_.camere_intrinsics.data());

  // cv::Mat distortion_coeffs = (cv::Mat_<double>(1, 5) << params_.k1,
  // params_.k2,
  //                              params_.p1, params_.p2, params_.k3);
  cv::Mat distortion_coeffs = cv::Mat(params_.distortion_coeffs).clone();

  // 7. 将3D点投影到2D图像平面
  std::vector<cv::Point2f> projected_image_points_2d;
  cv::projectPoints(selected_lidar_points_3d, r_vec, t_vec, camera_intrinsics,
                    distortion_coeffs, projected_image_points_2d);

  colored_point_cloud = PointcloudXYZRGBPtr(new PointcloudXYZRGB);
  colored_point_cloud->points.reserve(selected_lidar_points_3d.size());

  // 获取图像数据指针，提升像素访问效率
  const uchar *image_data = rgb_image.ptr<uchar>(0);
  const int step = rgb_image.step;

  // 遍历所有投影点，着色
  for (size_t i = 0; i < projected_image_points_2d.size(); ++i) {
    const cv::Point2f &pt = projected_image_points_2d[i];
    int x = static_cast<int>(pt.x);
    int y = static_cast<int>(pt.y);

    // 边界检查
    if (x < 0 || x >= params_.image_width || y < 0 || y >= params_.image_height)
      continue;

    const uchar *color = image_data + y * step + x * 3;

    if (color[0] == 0 && color[1] == 0 && color[2] == 0)
      continue;

    const cv::Point3f &point_3d = selected_lidar_points_3d[i];

    // TODO:需要根据雷达向前的方向，调整该参数
    if (point_3d.x > 100.0f)
      continue;

    // 创建带颜色的点
    pcl::PointXYZRGB colored_point;
    colored_point.x = point_3d.x;
    colored_point.y = point_3d.y;
    colored_point.z = point_3d.z;
    colored_point.b = color[0];
    colored_point.g = color[1];
    colored_point.r = color[2];

    // 添加到颜色点云中
    colored_point_cloud->points.emplace_back(colored_point);
  }
  colored_point_cloud->width = colored_point_cloud->points.size();
  colored_point_cloud->height = 1;
  colored_point_cloud->is_dense = false;
  PRINT_DEBUG("Successfully colored point cloudsize: %d\n",
              colored_point_cloud->size());
  return;
}

void LidarVisualAlignment::GetProjectionImage(
    const Eigen::Matrix3d &r_l_to_c, const Eigen::Vector3d &t_l_in_c,
    const ProjectionType &projection_type, const float &min_depth_threshold,
    const float &max_depth_threshold, const bool is_fill_image,
    const cv::Mat &rgb_image,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &raw_point_cloud,
    cv::Mat &projection_image) {
  // PRINT_INFO(GREEN "Getting projection image...\n" RESET);

  // 预分配点云数据、强度列表、深度列表
  std::vector<cv::Point3f> selected_lidar_points_3d;
  std::vector<cv::Point2f> projected_image_points_2d;
  std::vector<float> intensity_list;
  std::vector<float> depth_list;
  selected_lidar_points_3d.reserve(raw_point_cloud->size());
  projected_image_points_2d.reserve(raw_point_cloud->size());
  intensity_list.reserve(raw_point_cloud->size());
  depth_list.reserve(raw_point_cloud->size());

  for (const auto &point_3d : raw_point_cloud->points) {
    float depth = std::sqrt(point_3d.x * point_3d.x + point_3d.y * point_3d.y +
                            point_3d.z * point_3d.z);
    if (depth <= min_depth_threshold || depth >= max_depth_threshold)
      continue;
    Vec3d p_l = Vec3d(point_3d.x, point_3d.y, point_3d.z);
    Vec3d p_c = r_l_to_c * p_l + t_l_in_c;
    if (p_c(2) < 0)
      continue;
    Vec2d uv = camera_model_->Camera2Pixel(p_c);
    if (!camera_model_->isInFrame(uv, 1))
      continue;
    selected_lidar_points_3d.emplace_back(
        cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
    projected_image_points_2d.emplace_back(uv[0], uv[1]);
    intensity_list.emplace_back(point_3d.intensity);
    depth_list.emplace_back(depth);
    cv::circle(rgb_image_, cv::Point(uv(0), uv(1)), 2, cv::Scalar(0, 255, 0),
               -1, 6);
  }
  cv::imwrite(params_.debug_result_path + "projection_image_circle_debug.png",
              rgb_image_);

  // 将旋转矩阵转换为旋转向量（Rodrigues 形式）
  // cv::Mat r_mat_cv;
  // cv::eigen2cv(r_l_to_c, r_mat_cv); // 将 Eigen::Matrix3d 转换为 cv::Mat
  // cv::Mat r_vec;
  // cv::Rodrigues(r_mat_cv, r_vec); // 使用 Rodrigues

  // cv::Mat r_vec;
  // cv::eigen2cv(r_l_to_c, r_vec);

  // cv::Mat t_vec =
  //     (cv::Mat_<double>(3, 1) << t_l_in_c(0), t_l_in_c(1), t_l_in_c(2));

  // cv::Mat camera_intrinsics =
  //     (cv::Mat_<double>(3, 3) << params_.fx, 0.0, params_.cx, 0.0,
  //     params_.fy,
  //      params_.cy, 0.0, 0.0, 1.0);

  // cv::Mat distortion_coeffs = (cv::Mat_<double>(1, 5) << params_.k1,
  // params_.k2,
  //                              params_.p1, params_.p2, params_.k3);

  // std::vector<cv::Point2f> projected_image_points_2d;
  // cv::projectPoints(selected_lidar_points_3d, r_vec, t_vec,
  // camera_intrinsics,
  //                   distortion_coeffs, projected_image_points_2d);

  // 初始化投影图像
  cv::Mat image_project =
      cv::Mat::zeros(params_.image_height, params_.image_width, CV_16UC1);
  cv::Mat rgb_image_project =
      cv::Mat::zeros(params_.image_height, params_.image_width, CV_8UC3);

  // // 获取图像数据指针
  // ushort *image_ptr = image_project.ptr<ushort>(0); // ushort两个字节
  // uchar *rgb_ptr = rgb_image_project.ptr<uchar>(0); // uchar一个字节

  // 预计算常量
  // TODO：根据实际雷达强度值的范围来决定是否选择150.0f
  const float intensity_scale = 255.0f / 150.0f;
  const float intensity_scale_65535 = 65535.0f / 150.0f;
  const float depth_scale = 255.0f / max_depth_threshold;

  for (size_t i = 0; i < projected_image_points_2d.size(); ++i) {
    const cv::Point2f &pt = projected_image_points_2d[i];
    int x = static_cast<int>(pt.x);
    int y = static_cast<int>(pt.y);

    // 上面选择点的时候，已经判断过了
    // if (x < 0 || x >= params_.image_width || y < 0 || y >=
    // params_.image_height)
    //   continue;
    // // PRINT_DEBUG("projected_image_points_2d: %d, %d\n", x, y);

    float depth = depth_list[i];
    float intensity = intensity_list[i];

    if (projection_type == ProjectionType::DEPTH) {
      const float depth_weight = 1.0f; // 当前深度权重固定为1
      float gray_value =
          depth_weight * (depth / max_depth_threshold) * 65535.0f +
          (1.0f - depth_weight) * (intensity / 150.0f) * 65535.0f;

      ushort &current_depth = image_project.at<ushort>(y, x);
      if (current_depth == 0 || depth < current_depth) {
        current_depth = static_cast<ushort>(gray_value);
        rgb_image_project.at<cv::Vec3b>(y, x)[0] =
            static_cast<uchar>(depth * depth_scale);
        rgb_image_project.at<cv::Vec3b>(y, x)[1] =
            intensity > 150.0f
                ? 255
                : static_cast<uchar>(intensity * intensity_scale);
      }
    } else if (projection_type == ProjectionType::INTENSITY) {
      intensity =
          (intensity > 100.0f) ? 65535 : intensity * intensity_scale_65535;
      image_project.at<ushort>(y, x) = intensity;
    }
  }

  image_project.convertTo(image_project, CV_8UC1, 1 / 256.0);
  if (is_fill_image) {
    for (int i = 0; i < 5; ++i) {
      image_project = FillImage(image_project);
      // rgb_image_project = fillImg(rgb_image_project);
      cv::imwrite(params_.debug_result_path +
                      "intensity_projection_image_filled.png",
                  image_project);
    }
  } else {
    cv::imwrite(params_.debug_result_path + "intensity_projection_image.png",
                image_project);
  }
  // projection_image = image_project.clone();

  cv::Mat map_img =
      cv::Mat::zeros(params_.image_height, params_.image_width, CV_8UC3);
  for (int x = 0; x < map_img.cols; x++) {
    for (int y = 0; y < map_img.rows; y++) {
      uint8_t r, g, b;
      float norm = image_project.at<uchar>(y, x) / 256.0;
      MapJet(norm, 0, 1, r, g, b);
      map_img.at<cv::Vec3b>(y, x)[0] = b;
      map_img.at<cv::Vec3b>(y, x)[1] = g;
      map_img.at<cv::Vec3b>(y, x)[2] = r;
    }
  }
  cv::Mat merge_img;
  if (rgb_image.type() == CV_8UC3) {
    merge_img = 0.5 * map_img + 0.8 * rgb_image;
  } else {
    cv::Mat src_rgb;
    cv::cvtColor(rgb_image, src_rgb, cv::COLOR_GRAY2BGR);
    merge_img = 0.5 * map_img + 0.8 * src_rgb;
  }

  projection_image = merge_img.clone();
  return;
}

cv::Mat LidarVisualAlignment::GetResidualImage(
    const int &distance_threshold,
    const PointcloudXYZPtr &image_edge_line_point_cloud,
    const PointcloudXYZPtr &lidar_projection_edge_line_point_cloud) {
  // PRINT_DEBUG("Getting residual image...\n");
  cv::Mat residual_image =
      cv::Mat::zeros(params_.image_height, params_.image_width, CV_8UC3);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  PointcloudXYZPtr selected_lidar_projection_edge_line_point_cloud =
      PointcloudXYZPtr(new PointcloudXYZ);
  PointcloudXYZPtr tree_cloud = PointcloudXYZPtr(new PointcloudXYZ);

  // for (size_t i = 0; i <
  // lidar_projection_edge_line_point_cloud->points.size();
  //      i++) {
  //   const pcl::PointXYZ &pt =
  //   lidar_projection_edge_line_point_cloud->points[i]; cv::Point2d
  //   pt_2d(pt.x, -pt.y); if (pt_2d.x <= 0 || pt_2d.x >= params_.image_width
  //   || pt_2d.y <= 0 ||
  //       pt_2d.y >= params_.image_height)
  //     continue;
  //   selected_lidar_projection_edge_line_point_cloud->points.push_back(pt);
  // }

  int line_count = 0;
  kdtree->setInputCloud(image_edge_line_point_cloud);
  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  for (size_t i = 0; i < lidar_projection_edge_line_point_cloud->points.size();
       i++) {
    const pcl::PointXYZ &pt = lidar_projection_edge_line_point_cloud->points[i];
    if (!CheckFov(cv::Point2d(pt.x, -pt.y))) {
      PRINT_DEBUG(RED "x: %f, y: %f, out of fov!!!!!!\n" RESET, pt.x, -pt.y);
      continue;
    }
    if (kdtree->nearestKSearch(pt, K, pointIdxNKNSearch,
                               pointNKNSquaredDistance) > 0) {
      for (int j = 0; j < K; j++) {
        float distance = std::sqrt(pointNKNSquaredDistance[j]);
        if (distance >= distance_threshold)
          continue;
        line_count++;
        // 每隔3个点，画一条线，避免线太密集
        if ((line_count % 3) != 0)
          continue;
        const pcl::PointXYZ &pt_image =
            image_edge_line_point_cloud->points[pointIdxNKNSearch[j]];
        cv::line(residual_image, cv::Point(pt.x, -pt.y),
                 cv::Point(pt_image.x, -pt_image.y), cv::Scalar(0, 255, 0), 1);
      }
    }
  }

  for (size_t i = 0; i < image_edge_line_point_cloud->size(); i++) {
    const pcl::PointXYZ &pt = image_edge_line_point_cloud->points[i];
    residual_image.at<cv::Vec3b>(-pt.y, pt.x)[0] = 255;
    residual_image.at<cv::Vec3b>(-pt.y, pt.x)[1] = 0;
    residual_image.at<cv::Vec3b>(-pt.y, pt.x)[2] = 0;
  }

  for (size_t i = 0; i < lidar_projection_edge_line_point_cloud->size(); i++) {
    const pcl::PointXYZ &pt = lidar_projection_edge_line_point_cloud->points[i];
    residual_image.at<cv::Vec3b>(-pt.y, pt.x)[0] = 0;
    residual_image.at<cv::Vec3b>(-pt.y, pt.x)[1] = 0;
    residual_image.at<cv::Vec3b>(-pt.y, pt.x)[2] = 255;
  }

  // PRINT_DEBUG(BLUE "Successfully get residual image\n" RESET);
  return residual_image;
}

bool LidarVisualAlignment::CheckFov(const cv::Point2d &p) {
  if (p.x <= 0.0f || p.x >= params_.image_width || p.y <= 0.0f ||
      p.y >= params_.image_height) {
    return false;
  }
  return true;
}

void LidarVisualAlignment::LidarProjectToImage(
    const Eigen::Matrix3d &r_l_to_c, const Eigen::Vector3d &t_l_in_c,
    const ProjectionType &projection_type, const bool &is_fill_image,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &raw_point_cloud,
    const cv::Mat &rgb_image, cv::Mat &projection_image,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &projection_point_cloud) {
  if (projection_type == ProjectionType::INTENSITY) {
    PRINT_INFO(GREEN
               "Projecting lidar points to image by intensity...\n" RESET);
  } else if (projection_type == ProjectionType::DEPTH) {
    PRINT_INFO(GREEN "Projecting lidar points to image by depth...\n" RESET);
  } else if (projection_type == ProjectionType::BOTH) {
    PRINT_INFO(GREEN "Projecting lidar points to image by both intensity and "
                     "depth...\n" RESET);
  }
  if (raw_point_cloud->empty()) {
    PRINT_ERROR(RED "raw_point_cloud is empty!\n" RESET);
    return;
  }

  projection_point_cloud = PointcloudXYZIPtr(new PointcloudXYZI);

  // 变量初始化
  std::vector<cv::Point3f> selected_lidar_points_3d;
  std::vector<cv::Point2f> projected_image_points_2d;
  std::vector<float> intensity_values;
  std::vector<float> depth_values;

  selected_lidar_points_3d.reserve(raw_point_cloud->size());
  projected_image_points_2d.reserve(raw_point_cloud->size());
  intensity_values.reserve(raw_point_cloud->size());
  depth_values.reserve(raw_point_cloud->size());

  for (const auto &point_3d : raw_point_cloud->points) {
    float depth = std::sqrt(point_3d.x * point_3d.x + point_3d.y * point_3d.y +
                            point_3d.z * point_3d.z);
    Vec3d p_l = Vec3d(point_3d.x, point_3d.y, point_3d.z);
    Vec3d p_c = r_l_to_c * p_l + t_l_in_c;
    if (p_c(2) < 0)
      continue;
    Vec2d uv = camera_model_->Camera2Pixel(p_c);
    if (!camera_model_->isInFrame(uv, 1))
      continue;
    selected_lidar_points_3d.emplace_back(
        cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
    projected_image_points_2d.emplace_back(cv::Point2f(uv[0], uv[1]));
    intensity_values.emplace_back(point_3d.intensity);
    depth_values.emplace_back(depth);
  }

  cv::Mat image_project =
      cv::Mat::zeros(params_.image_height, params_.image_width, CV_16UC1);
  // cv::Mat rgb_image_project =
  //     cv::Mat::zeros(params_.image_height, params_.image_width, CV_8UC3);

  // 深度值越小，投影图像中相应的像素灰度值越接近黑色。
  // 强度值越小，投影图像中相应的像素灰度值越接近黑色。

  // // TODO:自适应的intensity和depth
  float max_intensity = 100.0f;
  // if (projection_type == ProjectionType::INTENSITY ||
  //     projection_type == ProjectionType::BOTH) {
  //   // 计算自适应阈值（以 95th 百分位为例）
  //   if (!intensity_values.empty()) {
  //     std::vector<float> intensity_values_tmp = intensity_values;
  //     size_t n = static_cast<size_t>(intensity_values_tmp.size() * 0.95);
  //     if (n == 0)
  //       n = 1; // 确保至少有一个元素
  //     // 使用 std::nth_element 找到第 n 个元素
  //     std::nth_element(intensity_values_tmp.begin(),
  //                      intensity_values_tmp.begin() + n,
  //                      intensity_values_tmp.end());
  //     max_intensity = intensity_values_tmp[n];
  //     // 可选：限制阈值的上下限，防止极端值
  //     max_intensity = std::min(std::max(max_intensity, 2.0f),
  //     max_intensity); PRINT_DEBUG(BLUE "adaptive_max_intensity: %1f\n"
  //     RESET, max_intensity);
  //   }
  // }

  // float max_intensity =
  //     *std::max_element(intensity_values.begin(), intensity_values.end());
  // for (float &intensity : intensity_values) {
  //   intensity =
  //       std::log(1.0f + intensity) / std::log(1.0f + max_intensity) *
  //       65535.0f;
  // }

  // 计算强度值的均值和标准差
  float mean_intensity =
      std::accumulate(intensity_values.begin(), intensity_values.end(), 0.0f) /
      intensity_values.size();
  float variance_intensity = 0.0f;
  for (float intensity : intensity_values) {
    variance_intensity +=
        (intensity - mean_intensity) * (intensity - mean_intensity);
  }
  variance_intensity /= intensity_values.size();
  float stddev_intensity = std::sqrt(variance_intensity);

  // Z-score 标准化，并重新映射到 [0, 65535] 区间
  for (float &intensity : intensity_values) {
    // 中心化到 [0, 65535]
    intensity =
        ((intensity - mean_intensity) / stddev_intensity) * 32767.5f + 32767.5f;
    intensity = std::min(std::max(intensity, 0.0f), 65535.0f);
  }

  // 计算深度值的均值和标准差
  float min_depth = 1.0f;
  float max_depth = 80.0f;
  if (projection_type == ProjectionType::DEPTH ||
      projection_type == ProjectionType::BOTH) {
    if (!depth_values.empty()) {
      std::vector<float> depth_values_tmp = depth_values;
      size_t n = static_cast<size_t>(depth_values_tmp.size() * 0.95);
      if (n == 0) {
        n = 1; // 确保至少有一个元素
      }
      // 使用 std::nth_element 找到第 n 个元素
      std::nth_element(depth_values_tmp.begin(), depth_values_tmp.begin() + n,
                       depth_values_tmp.end());
      max_depth = depth_values_tmp[n];
      max_depth = std::min(std::max(max_depth, 2.0f), max_depth);
      PRINT_DEBUG(BLUE "adaptive_max_depth: %1f\n" RESET, max_depth);
    }
  }
  float mean_depth =
      std::accumulate(depth_values.begin(), depth_values.end(), 0.0f) /
      depth_values.size();
  float variance_depth = 0.0f;
  for (float depth : depth_values) {
    variance_depth += (depth - mean_depth) * (depth - mean_depth);
  }
  variance_depth /= depth_values.size();
  float stddev_depth = std::sqrt(variance_depth);

  // Z-score 标准化，并重新映射到 [0, 65535] 区间
  for (float &depth : depth_values) {
    // 中心化到 [0, 65535]
    depth = ((depth - mean_depth) / stddev_depth) * 32767.5f + 32767.5f;
    depth = std::min(std::max(depth, 0.0f), 65535.0f);
  }

  // // 选择增强方法：例如对数缩放
  // for (float &depth : depth_values) {
  //   if (depth > min_depth) {
  //     depth = std::log1p(depth - min_depth) /
  //             std::log1p(max_depth - min_depth) * 65535.0f;
  //   } else {
  //     depth = 0.0f;
  //   }
  //   depth = std::min(std::max(depth, 0.0f), 65535.0f);
  // }
  // 另一种方法：直方图均衡化（取消注释以下代码）
  // cv::Mat depth_image(depth_values.size(), 1, CV_32F, depth_values.data());
  // cv::Mat depth_normalized;
  // depth_image.convertTo(depth_normalized, CV_8U, 255.0 / 65535.0);
  // cv::Mat depth_equalized;
  // cv::equalizeHist(depth_normalized, depth_equalized);
  // for (size_t i = 0; i < depth_values.size(); ++i) {
  //   depth_values[i] =
  //       static_cast<float>(depth_equalized.at<uchar>(i)) * 65535.0f /
  //       255.0f;
  // }

  // // 找到max depth
  // float min_depth = 1.0f;
  // float max_depth = 80.0f;
  // if (projection_type == ProjectionType::DEPTH ||
  //     projection_type == ProjectionType::BOTH) {
  //   if (!depth_values.empty()) {
  //     std::vector<float> depth_values_tmp = depth_values;
  //     size_t n = static_cast<size_t>(depth_values_tmp.size() * 0.95);
  //     if (n == 0) {
  //       n = 1; // 确保至少有一个元素
  //     }
  //     // 使用 std::nth_element 找到第 n 个元素
  //     std::nth_element(depth_values_tmp.begin(), depth_values_tmp.begin() +
  //     n,
  //                      depth_values_tmp.end());
  //     max_depth = depth_values_tmp[n];
  //     max_depth = std::min(std::max(max_depth, 2.0f), max_depth);
  //     PRINT_DEBUG(BLUE "adaptive_max_depth: %1f\n" RESET, max_depth);
  //   }
  // }

  for (size_t i = 0; i < projected_image_points_2d.size(); ++i) {
    const cv::Point2f &pt = projected_image_points_2d[i];
    int u = static_cast<int>(pt.x);
    int v = static_cast<int>(pt.y);

    float depth = depth_values[i];
    float intensity = intensity_values[i];

    if (projection_type == ProjectionType::INTENSITY) {
      // intensity = (intensity > max_intensity)
      //                 ? 65535.0f
      //                 : intensity / max_intensity * 65535.0f;
      image_project.at<ushort>(v, u) = intensity;

    } else if (projection_type == ProjectionType::DEPTH) {
      // depth = (depth > max_depth) ? 65535.0f
      //                             : (depth - min_depth) / max_depth *
      //                             65535.0f;
      // if (image_project.at<ushort>(v, u) == 0 ||
      //     depth < image_project.at<ushort>(v, u)) {
      //   image_project.at<ushort>(v, u) = depth;
      // }
      // 反转深度映射：近处物体亮，远处物体暗
      // depth = (depth > max_depth) ? 0.0f // 远处的深度值设置为0
      //                             : (1.0f - (depth - min_depth) /
      //                             max_depth)
      //                             *
      //                                   65535.0f; // 越远越小的值

      // 如果当前像素没有值或者新的深度值小于当前深度值，更新深度值
      // if (image_project.at<ushort>(v, u) == 0 ||
      //     depth < image_project.at<ushort>(v, u)) {
      //   image_project.at<ushort>(v, u) = depth;
      // }
      // 反转深度值，使得越近越亮
      // float inverted_depth = 65535.0f - depth;

      // //
      // 如果当前像素没有值或者新的反转深度值更大（即距离更近），则更新像素值
      // if (image_project.at<ushort>(v, u) == 0 ||
      //     inverted_depth > image_project.at<ushort>(v, u)) {
      //   image_project.at<ushort>(v, u) =
      //   static_cast<ushort>(inverted_depth);
      // }
      // float gray_value = (depth - min_depth) / max_depth * 65535.0f;
      // gray_value = std::min(std::max(gray_value, 0.0f), 65535.0f); //
      // // 保证在有效范围内
      // image_project.at<ushort>(v, u) = static_cast<ushort>(gray_value);
      pcl::PointXYZI point;
      point.x = selected_lidar_points_3d[i].x;
      point.y = selected_lidar_points_3d[i].y;
      point.z = selected_lidar_points_3d[i].z;
      point.intensity = depth;
      projection_point_cloud->emplace_back(point);

    } else if (projection_type == ProjectionType::BOTH) {
      const float depth_weight = 0.3f; // 当前深度权重固定为1
      float gray = depth_weight * (1.0f - depth / max_depth) * 65535.0f +
                   (1.0f - depth_weight) * intensity;
      image_project.at<ushort>(v, u) = gray;
      // rgb_image_project.at<cv::Vec3b>(v, u)[0] = (depth / max_depth) *
      // 255.0f; rgb_image_project.at<cv::Vec3b>(v, u)[1] =
      //     (intensity / max_intensity) * 255.0f;
    } else {
      PRINT_ERROR(RED "projection_type is not supported!\n" RESET);
    }
  }

  projection_point_cloud->width = projection_point_cloud->size();
  projection_point_cloud->height = 1;
  projection_point_cloud->is_dense = true;

  cv::Mat filled_img = image_project.clone();
  // filled_img = FillImage(image_project);
  if (is_fill_image) {
    for (size_t x = 0; x < image_project.cols; x++) {
      for (size_t y = 0; y < image_project.rows; y++) {
        if (image_project.at<ushort>(y, x) == 0) {
          std::vector<ushort> temp_depth;
          for (int x_inc = 0; x_inc < 4; x_inc++) {
            for (int y_inc = 0; y_inc < 4; y_inc++) {
              int x_near = x + pow(-1, x_inc) * int((x_inc + 2) / 2);
              int y_near = y + pow(-1, y_inc) * int((y_inc + 2) / 2);
              if (x_near > 0 && y_near > 0 &&
                  image_project.at<ushort>(y_near, x_near) != 0) {
                temp_depth.push_back(image_project.at<ushort>(y_near, x_near));
              }
            }
            if (temp_depth.size() > 1) {
              float mean_depth = 0;
              for (size_t i = 0; i < temp_depth.size(); i++) {
                mean_depth += temp_depth[i];
              }
              filled_img.at<ushort>(y, x) = mean_depth / temp_depth.size();
            }
          }
        }
      }
    }
  }

  cv::Mat test;
  filled_img.convertTo(test, CV_8U, 1.0f / 256.0f);
  // if (projection_type == ProjectionType::INTENSITY) {
  //   cv::imwrite(params_.debug_result_path +
  //   "projection_image_intensity.png",
  //               test);
  // } else if (projection_type == ProjectionType::DEPTH) {
  //   cv::imwrite(params_.debug_result_path + "projection_image_depth.png",
  //   test);
  // } else if (projection_type == ProjectionType::BOTH) {
  //   // cv::Mat gray_image_projection;
  //   // cv::cvtColor(rgb_image_project, gray_image_projection,
  //   // cv::COLOR_BGR2GRAY); cv::imwrite(params_.debug_result_path +
  //   //                 "test_projection_image_both_gray.png",
  //   //             gray_image_projection);
  //   cv::imwrite(params_.debug_result_path + "projection_image_both.png",
  //   test);
  // }

  projection_image = test;
  return;
}

void LidarVisualAlignment::LidarProjectToImage(
    const Eigen::Matrix3d &r_l_to_c, const Eigen::Vector3d &t_l_in_c,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &raw_point_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &projection_point_cloud,
    std::vector<std::vector<std::vector<pcl::PointXYZI>>>
        &image_pts_container) {

  PRINT_INFO(GREEN "Projecting lidar points to image...\n" RESET);

  if (raw_point_cloud->empty()) {
    PRINT_ERROR(RED "raw_point_cloud is empty!\n" RESET);
    return;
  }

  projection_point_cloud = PointcloudXYZIPtr(new PointcloudXYZI);

  // 用于存储每个像素位置上对应的LiDAR 3D点。结构为[y][x][points]
  image_pts_container.resize(
      params_.image_height,
      std::vector<std::vector<pcl::PointXYZI>>(params_.image_width));

  // 变量初始化
  std::vector<pcl::PointXYZI> selected_lidar_points_3d;
  std::vector<cv::Point2f> projected_image_points_2d;

  selected_lidar_points_3d.reserve(raw_point_cloud->size());
  projected_image_points_2d.reserve(raw_point_cloud->size());

  for (const auto &point_3d : raw_point_cloud->points) {
    Vec3d p_l = Vec3d(point_3d.x, point_3d.y, point_3d.z);
    Vec3d p_c = r_l_to_c * p_l + t_l_in_c;
    pcl::PointXYZI point_c(p_c[0], p_c[1], p_c[2], point_3d.intensity);
    if (p_c(2) < 0)
      continue;
    Vec2d uv = camera_model_->Camera2Pixel(p_c);
    if (!camera_model_->isInFrame(uv, 1))
      continue;
    selected_lidar_points_3d.emplace_back(point_3d);
    projected_image_points_2d.emplace_back(cv::Point2f(uv[0], uv[1]));
    int v = static_cast<int>(uv[1]);
    int u = static_cast<int>(uv[0]);
    image_pts_container[v][u].emplace_back(point_c);
    projection_point_cloud->points.emplace_back(point_c);
  }

  if (projection_point_cloud->empty()) {
    PRINT_ERROR(RED "projection_point_cloud is empty!\n" RESET);
    return;
  }

  projection_point_cloud->width = projection_point_cloud->size();
  projection_point_cloud->height = 1;
  projection_point_cloud->is_dense = true;

  if (image_pts_container.empty()) {
    PRINT_ERROR(RED "image_pts_container is empty!\n" RESET);
    return;
  }

  PRINT_INFO(GREEN "Successfully project lidar points to image\n" RESET);

  return;
}

void LidarVisualAlignment::ImageContoursDetection(
    const int &gaussian_blur_ksize, const int &canny_threshold,
    const cv::Mat &src_img, std::vector<std::vector<cv::Point>> &contours) {
  PRINT_INFO(GREEN "Detecting image contours...\n" RESET);

  // 第四和第五个参数：高斯核在 X 和 Y
  // 方向的标准差，0表示根据核大小自动计算。
  cv::GaussianBlur(src_img, src_img,
                   cv::Size(gaussian_blur_ksize, gaussian_blur_ksize), 0, 0);
  // cv::blur(src_img, src_img, cv::Size(gaussian_blur_ksize,
  // gaussian_blur_ksize),
  //          cv::Point(-1, -1));
  //   cv::imshow("gaussian blur image", src_img);
  //   cv::waitKey();
  cv::imwrite(params_.debug_result_path + "gaussian_blur_image.png", src_img);
  cv::Mat canny_result =
      cv::Mat::zeros(params_.image_height, params_.image_width, CV_8UC1);

  // 第三个参数：边缘检测的阈值，越大越容易检测到边缘，但是也会漏掉一些边缘。
  // 第四个参数：边缘检测的突变值，越大越容易检测到边缘，但是也会漏掉一些边缘。
  // 第五个参数：Sobel内核大小，这里设置为 3，表示 3x3 大小的 Sobel 内核。
  // 第六个参数：是否使用 L2 范数来计算梯度，这里设置为 true，表示使用 L2
  // 范数。
  cv::Canny(src_img, canny_result, canny_threshold, canny_threshold * 3, 3,
            true);
  //   cv::imshow("canny image", canny_result);
  //   cv::waitKey();
  cv::imwrite(params_.debug_result_path + "canny_image.png", canny_result);

  std::vector<cv::Vec4i> hierarchy;
  // 第一个参数：输入图像（Canny 边缘检测结果）。
  // 第二个参数：输出的轮廓（点集）。
  // 第三个参数：输出的层级（每个轮廓对应的父轮廓）。
  // 第四个参数：轮廓检索模式，这里设置为RETR_EXTERNAL，表示只检索最外层轮廓。
  // 第五个参数：轮廓近似方法，这里设置为CHAIN_APPROX_NONE，表示不进行轮廓近似。
  // 第六个参数：偏移量，这里设置为 (0,0)，表示从图像左上角开始计数，不偏移
  cv::findContours(canny_result, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
  PRINT_INFO(GREEN
             "Successfully detected image edge!, contours size:%d\n" RESET,
             contours.size());
  return;
}

void LidarVisualAlignment::EdgeExtractionByContours(
    const std::vector<std::vector<cv::Point>> &contours,
    const int &min_length_threshold, cv::Mat &edge_image,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_point_cloud) {

  edge_point_cloud = PointcloudXYZPtr(new PointcloudXYZ);

  edge_image =
      cv::Mat::zeros(params_.image_height, params_.image_width, CV_8UC1);

  for (size_t i = 0; i < contours.size(); i++) {
    if (contours[i].size() < min_length_threshold)
      continue;
    for (size_t j = 0; j < contours[i].size(); j++) {
      pcl::PointXYZ p;
      p.x = contours[i][j].x;
      p.y = -contours[i][j].y;
      p.z = 0;
      edge_point_cloud->points.push_back(p);
      edge_image.at<uchar>(contours[i][j].y, contours[i][j].x) = 255;
    }
  }

  edge_point_cloud->width = edge_point_cloud->size();
  edge_point_cloud->height = 1;
  edge_point_cloud->is_dense = true;

  return;
}

pcl::PointXYZI LidarVisualAlignment::InterpolatePoint(
    const std::vector<std::vector<std::vector<pcl::PointXYZI>>> &pts_container,
    int j, int i) {

  // 默认值为 (100, 100, 100, 100)
  pcl::PointXYZI interpolated_pt(100.0f, 100.0f, 100.0f, 100.0f);
  int valid_points = 0;
  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f, sum_i = 0.0f;
  std::vector<std::pair<int, int>> neighbors = {
      {j - 1, i}, {j + 1, i}, {j, i - 1}, {j, i + 1}};

  // 遍历上下左右四个方向，检查是否在范围内，并且是有效点
  for (const auto &neighbor : neighbors) {
    int nj = neighbor.first;
    int ni = neighbor.second;

    // 检查是否越界
    if (nj >= 0 && nj < params_.image_height && ni >= 0 &&
        ni < params_.image_width && !pts_container[nj][ni].empty()) {

      // 取第一个有效的点
      pcl::PointXYZI p = pts_container[nj][ni][0];
      sum_x += p.x;
      sum_y += p.y;
      sum_z += p.z;
      sum_i += p.intensity;
      valid_points++;
    }
  }

  // 如果有足够的有效邻居点，计算均值
  if (valid_points >= 2) {
    interpolated_pt.x = sum_x / valid_points;
    interpolated_pt.y = sum_y / valid_points;
    interpolated_pt.z = sum_z / valid_points;
    interpolated_pt.intensity = sum_i / valid_points;
  }

  return interpolated_pt;
}
