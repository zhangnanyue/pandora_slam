#ifndef LIDRAR_VISUAL_ALIGNMENT_H
#define LIDRAR_VISUAL_ALIGNMENT_H

#include "camera_model/pinhole_camera.h"
#include "lidar_visual_alignment_options.h"
#include "lidar_visual_alignment_utils.h"
#include "voxel_utils.h"
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

// #include <pcl/common/io.h>
// #include <pcl/common/transforms.h>
// #include <pcl/point_types.h>

class LidarVisualAlignment {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<const LidarVisualAlignment> ConstPtr;
  typedef std::shared_ptr<LidarVisualAlignment> Ptr;

  using PointcloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
  using PointcloudXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
  using PointcloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
  using PointcloudXYZIPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;
  using PointcloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;
  using PointcloudXYZINPtr = pcl::PointCloud<pcl::PointXYZINormal>::Ptr;
  using PointcloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
  using PointcloudXYZRGBPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;

  using Vec2d = Eigen::Matrix<double, 2, 1>;
  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec3f = Eigen::Matrix<float, 3, 1>;
  using Vec4d = Eigen::Matrix<double, 4, 1>;
  using Vec4f = Eigen::Matrix<float, 4, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using Mat4d = Eigen::Matrix<double, 4, 4>;

  LidarVisualAlignment(const LidarVisualAlignmentOptions &params);
  ~LidarVisualAlignment();

  void Proc(const cv::Mat &raw_image,
            const pcl::PointCloud<pcl::PointXYZI>::Ptr &raw_point_cloud);

  // void set_r_i_to_l(const Eigen::Matrix<double, 3, 3> &r_i_to_l) {
  //   r_i_to_l_ = r_i_to_l;
  // }
  // void set_t_i_in_l(const Eigen::Vector3d &t_i_in_l) { t_i_in_l_ =
  // t_i_in_l;
  // }

private:
  enum ProjectionType { DEPTH, INTENSITY, BOTH };

  struct PnPData {
    double x, y, z, u, v;
  };

  struct DirectionPnPData {
    double x, y, z, u, v;
    Eigen::Vector2d image_direction;
    Eigen::Vector2d projected_image_direction;
    int line_number;
  };

  // // image base params
  // int image_width_;
  // int image_height_;
  // double fx_, fy_, cx_, cy_;
  // double k1_, k2_, p1_, p2_, k3_;

  CameraModel::Ptr camera_model_;
  LidarVisualAlignmentOptions params_;

  // 存储图像边缘轮廓点云
  PointcloudXYZPtr image_edge_line_point_cloud_;

  // 存储雷达点云平面交接处的点云
  PointcloudXYZIPtr lidar_depth_continuous_edge_line_point_cloud_;
  std::vector<int> lidar_depth_continuous_edge_line_number_;
  int lidar_depth_continuous_edge_line_number_total_;

  // 利用外参投影获得的彩色点云
  PointcloudXYZRGBPtr colored_point_cloud_;

  // cv::Mat raw_image_;
  cv::Mat gray_image_;
  cv::Mat rgb_image_;

  cv::Mat edge_line_contour_image_;

  cv::Mat lidar_depth_image_;
  cv::Mat lidar_intensity_image_;

  std::vector<std::vector<cv::Point>> camera_contours_;
  std::vector<std::vector<cv::Point>> lidar_depth_contours_;
  std::vector<std::vector<cv::Point>> lidar_intensity_contours_;

  // cv::Mat contours_image_;
  // cv::Mat contours_projection_depth_image_;
  // cv::Mat contours_projection_intensity_image_;

  cv::Mat camera_edge_image_;
  cv::Mat lidar_depth_edge_image_;
  cv::Mat lidar_intensity_edge_image_;

  PointcloudXYZPtr camera_edge_point_cloud_;
  PointcloudXYZPtr lidar_edge_point_cloud_;
  PointcloudXYZPtr lidar_depth_edge_point_cloud_;
  PointcloudXYZPtr lidar_intensity_edge_point_cloud_;

  void ImageEdgeLineGenerator(
      const int &gaussian_blur_ksize, const int &canny_threshold,
      const int &edge_line_contour_min_length_threshold, const cv::Mat &src_img,
      cv::Mat &edge_line_contour_image,
      pcl::PointCloud<pcl::PointXYZ>::Ptr &image_edge_line_point_cloud);

  void BuildVoxelMapFromRawPointCloud(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_point_cloud,
      const float &voxel_size,
      std::unordered_map<VoxelPosition, Voxel::Ptr> &voxel_map);

  void
  DownsampleVoxelMap(std::unordered_map<VoxelPosition, Voxel::Ptr> &voxel_map);

  void LidarEdgeLineExtractor(
      const std::unordered_map<VoxelPosition, Voxel::Ptr> &voxel_map,
      const float &plane_ransac_distance_threshold,
      const int &plane_point_size_threshold,
      pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_line_point_cloud);

  void CalculateLineFromPlaneIntersection(
      const std::vector<Plane> &plane_list, const float &voxel_size,
      const Eigen::Vector3d &voxel_origin,
      std::vector<pcl::PointCloud<pcl::PointXYZI>> &line_point_clouds);

  void BuildDirectionalVectorPnP(
      const Eigen::Matrix3d &r_l_to_c, const Eigen::Vector3d &t_l_in_c,
      const int &distance_threshold,
      const PointcloudXYZPtr &image_edge_line_point_cloud,
      const PointcloudXYZIPtr &lidar_edge_line_point_cloud,
      const std::vector<int> &lidar_edge_line_number,
      const bool show_residual_image,
      std::vector<DirectionPnPData> &direction_pnp_data_vec);

  cv::Mat GetResidualImage(
      const int &distance_threshold,
      const PointcloudXYZPtr &image_edge_line_point_cloud,
      const PointcloudXYZPtr &lidar_projection_edge_line_point_cloud);

  void ColorRawPointCloudByRgbImage(
      const Eigen::Matrix3d &r_l_to_c, const Eigen::Vector3d &t_c_in_l,
      const int &density, const int &min_intensity_threshold,
      const float &min_distance_threshold, const float &max_distance_threshold,
      const cv::Mat &rgb_image,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &raw_point_cloud,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_point_cloud);

  void GetProjectionImage(
      const Eigen::Matrix3d &r_l_to_c, const Eigen::Vector3d &t_c_in_l,
      const ProjectionType &projection_type, const float &min_depth_threshold,
      const float &max_depth_threshold, const bool is_fill_image,
      const cv::Mat &rgb_image,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &raw_point_cloud,
      cv::Mat &projection_image);

  bool CheckFov(const cv::Point2d &p);

  void LidarProjectToImage(
      const Eigen::Matrix3d &r_l_to_c, const Eigen::Vector3d &t_l_in_c,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &raw_point_cloud,
      pcl::PointCloud<pcl::PointXYZI>::Ptr &projection_point_cloud,
      std::vector<std::vector<std::vector<pcl::PointXYZI>>>
          &image_pts_container);

  pcl::PointXYZI
  InterpolatePoint(const std::vector<std::vector<std::vector<pcl::PointXYZI>>>
                       &pts_container,
                   int j, int i);

  /**
   * @brief 将激光雷达点云投影到图像上。
   *
   * 此函数根据给定的投影类型（强度、深度或二者）将激光雷达点云投影到图像中。
   * 投影过程包括坐标变换、深度和强度值的归一化以及图像填充（可选）。最后，结果保存为调试图像。
   *
   * @param[in] r_l_to_c 激光雷达坐标系到相机坐标系的旋转矩阵。
   * @param[in] t_l_in_c 激光雷达坐标系到相机坐标系的平移向量。
   * @param[in] projection_type
   * 投影类型，决定使用强度、深度还是两者混合进行投影。
   * @param[in] is_fill_image 是否对空白区域进行填充。
   * @param[in] raw_point_cloud 输入的激光雷达点云数据。
   * @param[in] rgb_image 原始图像，未在本函数中使用，但可用于调试或其他处理。
   * @param[out] projection_image
   * 输出的投影图像，依据投影类型进行处理（强度、深度或两者）。
   *
   * @note
   * - 本函数会根据投影类型分别处理强度值和深度值：
   *   - 强度投影时，强度值被归一化并映射到灰度值；
   *   - 深度投影时，深度值被映射到灰度值，较小的深度值对应较浅的灰度。
   *   - 若投影类型为“二者”，则深度和强度加权平均后生成图像。
   * - 支持自适应的强度值和深度值的归一化，使用百分位数法确定最大值。
   *
   * @return 无返回值，结果直接保存为图像文件。
   */
  void LidarProjectToImage(
      const Eigen::Matrix3d &r_l_to_c, const Eigen::Vector3d &t_l_in_c,
      const ProjectionType &projection_type, const bool &is_fill_image,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &raw_point_cloud,
      const cv::Mat &rgb_image, cv::Mat &projection_image,
      pcl::PointCloud<pcl::PointXYZI>::Ptr &projection_point_cloud);

  /**
   * @brief 图像轮廓检测函数
   *
   * 该函数使用高斯模糊和Canny边缘检测来处理输入图像，并使用findContours提取图像中的轮廓。
   * 经过处理后，轮廓信息保存在传入的 contours_image 参数中。
   *
   * @param gaussian_blur_ksize 高斯模糊核的大小，必须为奇数。
   * @param canny_threshold
   * Canny边缘检测的低阈值，通常需要调整以获得较好的边缘检测效果。
   * @param src_img 输入的源图像。
   * @param contours_image 输出的轮廓信息，存储图像中的轮廓点集。
   */
  void
  ImageContoursDetection(const int &gaussian_blur_ksize,
                         const int &canny_threshold, const cv::Mat &src_img,
                         std::vector<std::vector<cv::Point>> &contours_image);

  /**
   * @brief 使用轮廓信息提取边缘点并生成点云与边缘图像。
   *
   * 此函数根据输入的轮廓信息提取符合长度要求的边缘点，并将其存储为点云和二值化边缘图像。
   * 主要用于从图像轮廓中提取具有几何意义的边缘信息。
   *
   * @param[in] contours 输入的轮廓点集，每个轮廓由点集表示。
   * @param[in] min_length_threshold
   * 轮廓的最小长度阈值，小于该阈值的轮廓将被忽略。
   * @param[out] edge_image 输出的二值化边缘图像，边缘像素值为255，其他区域为0。
   * @param[out] edge_point_cloud 输出的边缘点云，包含边缘点的三维坐标。
   *
   * @note
   * -
   * 边缘点云的Z值默认为0，X和Y值分别对应像素点的坐标，其中Y坐标进行了符号翻转。
   * - 符合条件的边缘点将同时在点云和边缘图像中表示。
   * - 输出的边缘图像大小与输入图像一致。
   *
   * @return 无返回值，结果通过参数引用返回。
   */

  /**
   * @brief 使用轮廓信息提取边缘点并生成点云与边缘图像。
   *
   * 此函数根据输入的轮廓信息提取符合长度要求的边缘点，并将其存储为点云和二值化边缘图像。
   * 主要用于从图像轮廓中提取具有几何意义的边缘信息。
   *
   * @param[in] contours 输入的轮廓点集，每个轮廓由点集表示。
   * @param[in] min_length_threshold
   * 轮廓的最小长度阈值，小于该阈值的轮廓将被忽略。
   * @param[out] edge_image 输出的二值化边缘图像，边缘像素值为255，其他区域为0。
   * @param[out] edge_point_cloud 输出的边缘点云，包含边缘点的三维坐标。
   *
   * @note
   * -
   * 边缘点云的Z值默认为0，X和Y值分别对应像素点的坐标，其中Y坐标进行了符号翻转。
   * - 符合条件的边缘点将同时在点云和边缘图像中表示。
   * - 输出的边缘图像大小与输入图像一致。
   *
   * @return 无返回值，结果通过参数引用返回。
   */
  void EdgeExtractionByContours(
      const std::vector<std::vector<cv::Point>> &contours,
      const int &min_length_threshold, cv::Mat &edge_image,
      pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_point_cloud);

  Mat3d corrected_r_l_to_c_ = Eigen::Matrix3d::Identity();
  Vec3d corrected_t_l_in_c_ = Eigen::Vector3d::Zero();

  Mat3d init_r_l_to_c_ = Eigen::Matrix3d::Identity();
  Vec3d init_t_l_in_c_ = Eigen::Vector3d::Zero();

  void LoadVPnPData(std::vector<DirectionPnPData> &direction_pnp_data_vec,
                    int distance_threshold, int cnt);

  class DirectionPnpDataCostFunction {

  public:
    DirectionPnpDataCostFunction(const DirectionPnPData &data) : data_(data) {}

    template <typename T>
    bool operator()(const T *_q, const T *_t, T *residual) const {
      const T &fx = T(1364.45);
      const T &fy = T(1366.46);
      const T &cx = T(958.327);
      const T &cy = T(535.074);
      const T &k1 = T(0.0958277);
      const T &k2 = T(-0.198233);
      const T &p1 = T(-0.000147133);
      const T &p2 = T(-0.000430056);
      const T &k3 = T(0.0);

      Eigen::Quaternion<T> q{_q[3], _q[0], _q[1], _q[2]};
      Eigen::Matrix<T, 3, 1> t{_t[0], _t[1], _t[2]};

      Eigen::Matrix<T, 3, 1> p_lidar(T(data_.x), T(data_.y), T(data_.z));
      Eigen::Matrix<T, 3, 1> p_camera = q.toRotationMatrix() * p_lidar + t;

      T x_normal = T(p_camera[0]) / T(p_camera[2] + T(1e-9));
      T y_normal = T(p_camera[1]) / T(p_camera[2] + T(1e-9));
      T r2 = x_normal * x_normal + y_normal * y_normal;
      T r4 = r2 * r2;
      T r6 = r4 * r2;
      T distortion = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
      T x_distorted = x_normal * distortion + 2.0 * p1 * x_normal * y_normal +
                      p2 * (r2 + 2.0 * x_normal * x_normal);
      T y_distorted = y_normal * distortion +
                      p1 * (r2 + 2.0 * y_normal * y_normal) +
                      2.0 * p2 * x_normal * y_normal;
      T u = fx * x_distorted + cx;
      T v = fy * y_distorted + cy;

      const T epsilon = T(1e-8);
      if (ceres::abs(data_.image_direction(0)) < epsilon &&
          ceres::abs(data_.image_direction(1)) < epsilon) {
        residual[0] = u - T(data_.u);
        residual[1] = v - T(data_.v);
      } else {
        residual[0] = u - T(data_.u);
        residual[1] = v - T(data_.v);
        Eigen::Matrix<T, 2, 2> I =
            Eigen::Matrix<float, 2, 2>::Identity().cast<T>();
        Eigen::Matrix<T, 2, 1> n = data_.image_direction.cast<T>();
        Eigen::Matrix<T, 1, 2> nt = data_.image_direction.transpose().cast<T>();
        Eigen::Matrix<T, 2, 2> V = I - n * nt;
        Eigen::Matrix<T, 2, 1> R(residual[0], residual[1]);

        R = V * R;
        residual[0] = R(0);
        residual[1] = R(1);
      }

      return true;
    }

    static ceres::CostFunction *Create(const DirectionPnPData &data) {
      return (new ceres::AutoDiffCostFunction<DirectionPnpDataCostFunction, 2,
                                              4, 3>(
          new DirectionPnpDataCostFunction(data)));
    }

  private:
    DirectionPnPData data_;
    LidarVisualAlignmentOptions params_;
  };
};

#endif // LIDRAR_VISUAL_ALIGNMENT_H