#ifndef LIDRAR_VISUAL_ALIGNMENT_OPTIONS_H
#define LIDRAR_VISUAL_ALIGNMENT_OPTIONS_H

#include "common_utils/opencv_yaml_parse.h"
#include "common_utils/print.h"
#include "common_utils/utils.h"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

struct LidarVisualAlignmentOptions {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // base camera params
  std::string camera_model = "pinhole";
  std::string distortion_model = "radtan";

  // (width, height)
  std::vector<int> resolution = {1, 1};
  int image_width = 1;
  int image_height = 1;

  // camera intrinsics K
  Eigen::Matrix3d camere_intrinsics = Eigen::Matrix3d::Identity();
  double fx, fy, cx, cy;

  // distortion coefficients
  std::vector<double> distortion_coeffs;
  double k1, k2, p1, p2, k3;

  // Extrinsic T_cam_lidar
  Eigen::Matrix4d T_cam_lidar = Eigen::Matrix4d::Identity();

  // downsample_factor
  int downsample_factor = 1;

  void LoadCameraBaseParamsAndPrint(const YamlParser::Ptr &parser = nullptr) {
    if (parser != nullptr) {
      parser->ParseConfig("camera.camera_model", camera_model);
      parser->ParseConfig("camera.distortion_model", distortion_model);
      parser->ParseConfig("camera.resolution", resolution);
      parser->ParseConfig("camera.intrinsics", camere_intrinsics);
      parser->ParseConfig("camera.distortion_coeffs", distortion_coeffs);
      parser->ParseConfig("camera.T_cam_lidar", T_cam_lidar);
      parser->ParseConfig("camera.downsample_factor", downsample_factor);
      image_width = resolution[0] / downsample_factor;
      image_height = resolution[1] / downsample_factor;
      fx = camere_intrinsics(0, 0) / downsample_factor;
      fy = camere_intrinsics(1, 1) / downsample_factor;
      cx = camere_intrinsics(0, 2) / downsample_factor;
      cy = camere_intrinsics(1, 2) / downsample_factor;
      k1 = distortion_coeffs[0];
      k2 = distortion_coeffs[1];
      p1 = distortion_coeffs[2];
      p2 = distortion_coeffs[3];
      k3 = distortion_coeffs[4];
    }

    PRINT_DEBUG("camera.camera_model: %s\n", camera_model.c_str());
    PRINT_DEBUG("camera.distortion_model: %s\n", distortion_model.c_str());
    PRINT_DEBUG("camera.downsample_factor: %d\n", downsample_factor);
    PRINT_DEBUG("camera.resolution: %d, %d\n", resolution[0], resolution[1]);
    PRINT_DEBUG("image_width, image_height: %d, %d\n", image_width,
                image_height);
    PRINT_DEBUG("camera.intrinsics:\n%s\n",
                EigenMatrixToString(camere_intrinsics).c_str());
    PRINT_DEBUG("fx, fy, cx, cy: %f, %f, %f, %f\n", fx, fy, cx, cy);
    PRINT_DEBUG("camera.distortion_coeffs(k1, k2, p1, p2, k3): %s\n",
                VectorToString(distortion_coeffs).c_str());
    PRINT_DEBUG("k1, k2, p1, p2, k3: %f, %f, %f, %f, %f\n", k1, k2, p1, p2, k3)
    PRINT_DEBUG("camera.T_cam_lidar:\n%s\n",
                EigenMatrixToString(T_cam_lidar).c_str());
  }

  // common params
  std::string debug_result_path = "./result/";
  void LoadCommonParamsAndPrint(const YamlParser::Ptr &parser = nullptr) {
    if (parser != nullptr) {
      parser->ParseConfig("common.debug_result_path", debug_result_path);
    }
    PRINT_DEBUG("debug_result_path: %s\n", debug_result_path.c_str());
  };

  // params for Canny Edge Extraction
  int gaussian_blur_ksize = 5;
  int canny_threshold = 20;
  int edge_line_contour_min_length_threshold = 200;
  float voxel_size = 1.0f;

  // params for Lidar Edge line extract
  float plane_ransac_distance_threshold = 0.02;
  int plane_point_size_threshold = 60;
  int plane_max_size_threshold = 5;
  int plane_min_size_threshold = 2;

  int plane_connection_min_angle_threshold = 30;
  int plane_connection_max_angle_threshold = 150;

  double plane_connection_min_cos_threshold = 0.0f;
  double plane_connection_max_cos_threshold = 1.0f;

  float point_to_plane_min_distance_threshold = 0.03;
  float point_to_plane_max_distance_threshold = 0.06;

  float point_to_plane_min_distance_threshold_sqaured = 0.0009;
  float point_to_plane_max_distance_threshold_sqaured = 0.0036;

  int colored_point_cloud_dense_threshold = 3;
  int colored_point_cloud_min_intensity_threshold = 10;

  float colored_point_cloud_min_depth_threshold = 1.0f;
  float colored_point_cloud_max_depth_threshold = 80.0f;

  void LoadCoreParamsAndPrint(const YamlParser::Ptr &parser = nullptr) {
    if (parser != nullptr) {
      parser->ParseConfig("gaussian_blur_ksize", gaussian_blur_ksize);
      parser->ParseConfig("canny_threshold", canny_threshold);
      parser->ParseConfig("edge_line_contour_min_length_threshold",
                          edge_line_contour_min_length_threshold);
      parser->ParseConfig("voxel_size", voxel_size);
      parser->ParseConfig("plane_ransac_distance_threshold",
                          plane_ransac_distance_threshold);
      parser->ParseConfig("plane_point_size_threshold",
                          plane_point_size_threshold);
      parser->ParseConfig("plane_max_size_threshold", plane_max_size_threshold);
      parser->ParseConfig("plane_min_size_threshold", plane_min_size_threshold);

      parser->ParseConfig("plane_connection_min_angle_threshold",
                          plane_connection_min_angle_threshold);
      parser->ParseConfig("plane_connection_max_angle_threshold",
                          plane_connection_max_angle_threshold);

      parser->ParseConfig("point_to_plane_min_distance_threshold",
                          point_to_plane_min_distance_threshold);
      parser->ParseConfig("point_to_plane_max_distance_threshold",
                          point_to_plane_max_distance_threshold);
      parser->ParseConfig("colored_point_cloud_dense_threshold",
                          colored_point_cloud_dense_threshold);
      parser->ParseConfig("colored_point_cloud_min_intensity_threshold",
                          colored_point_cloud_min_intensity_threshold);
      parser->ParseConfig("colored_point_cloud_min_depth_threshold",
                          colored_point_cloud_min_depth_threshold);
      parser->ParseConfig("colored_point_cloud_max_depth_threshold",
                          colored_point_cloud_max_depth_threshold);
    }
    // PRINT_DEBUG(ss.str().c_str());
    PRINT_DEBUG("gaussian_blur_ksize: %d\n", gaussian_blur_ksize);
    PRINT_DEBUG("canny_threshold: %d\n", canny_threshold);
    PRINT_DEBUG("edge_line_contour_min_length_threshold: %d\n",
                edge_line_contour_min_length_threshold);
    PRINT_DEBUG("voxel_size: %f\n", voxel_size);
    PRINT_DEBUG("plane_ransac_distance_threshold: %f\n",
                plane_ransac_distance_threshold);
    PRINT_DEBUG("plane_point_size_threshold: %d\n", plane_point_size_threshold);
    PRINT_DEBUG("plane_max_size_threshold: %d\n", plane_max_size_threshold);
    PRINT_DEBUG("plane_min_size_threshold: %d\n", plane_min_size_threshold);
    PRINT_DEBUG("plane_connection_min_angle_threshold: %d\n",
                plane_connection_min_angle_threshold);
    PRINT_DEBUG("plane_connection_max_angle_threshold: %d\n",
                plane_connection_max_angle_threshold);

    plane_connection_min_cos_threshold =
        cos(DEG2RAD(plane_connection_min_angle_threshold));
    plane_connection_max_cos_threshold =
        cos(DEG2RAD(plane_connection_max_angle_threshold));
    PRINT_DEBUG("plane_connection_min_cos_threshold: %f\n",
                plane_connection_min_cos_threshold);
    PRINT_DEBUG("plane_connection_max_cos_threshold: %f\n",
                plane_connection_max_cos_threshold);

    PRINT_DEBUG("point_to_plane_min_distance_threshold: %f\n",
                point_to_plane_min_distance_threshold);
    PRINT_DEBUG("point_to_plane_max_distance_threshold: %f\n",
                point_to_plane_max_distance_threshold);
    point_to_plane_min_distance_threshold_sqaured =
        point_to_plane_min_distance_threshold *
        point_to_plane_min_distance_threshold;
    point_to_plane_max_distance_threshold_sqaured =
        point_to_plane_max_distance_threshold *
        point_to_plane_max_distance_threshold;
    PRINT_DEBUG("point_to_plane_min_distance_threshold_sqaured: %f\n",
                point_to_plane_min_distance_threshold_sqaured);
    PRINT_DEBUG("point_to_plane_max_distance_threshold_sqaured: %f\n",
                point_to_plane_max_distance_threshold_sqaured);

    PRINT_DEBUG("colored_point_cloud_dense_threshold: %d\n",
                colored_point_cloud_dense_threshold);
    PRINT_DEBUG("colored_point_cloud_min_intensity_threshold: %d\n",
                colored_point_cloud_min_intensity_threshold);
    PRINT_DEBUG("colored_point_cloud_min_depth_threshold: %f\n",
                colored_point_cloud_min_depth_threshold);
    PRINT_DEBUG("colored_point_cloud_max_depth_threshold: %f\n",
                colored_point_cloud_max_depth_threshold);
  };
};
#endif // LIDRAR_VISUAL_ALIGNMENT_OPTIONS_H