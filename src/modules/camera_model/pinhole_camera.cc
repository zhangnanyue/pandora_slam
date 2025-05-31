#include "pinhole_camera.h"

PinholeCamera::PinholeCamera(const double &width, const double &height,
                             const double &fx, const double &fy,
                             const double &cx, const double &cy,
                             const double &k1, const double &k2,
                             const double &p1, const double &p2,
                             const double &k3, bool is_distortion)
    : CameraModel(width, height), fx_(fx), fy_(fy), cx_(cx), cy_(cy),
      distortion_(is_distortion), undist_map1_(height_, width_, CV_16SC2),
      undist_map2_(height_, width_, CV_16SC2) {
  k1_ = k1;
  k2_ = k2;
  p1_ = p1;
  p2_ = p2;
  k3_ = k3;
  K_cv_ =
      (cv::Mat_<float>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);

  D_cv_ = (cv::Mat_<float>(1, 5) << k1_, k2_, p1_, p2_, k3_);

  cv::initUndistortRectifyMap(K_cv_, D_cv_, cv::Mat_<double>::eye(3, 3), K_cv_,
                              cv::Size(width_, height_), CV_16SC2, undist_map1_,
                              undist_map2_);
  K_ << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0;
  K_inv_ = K_.inverse();
}

PinholeCamera::~PinholeCamera() {}

Eigen::Vector3d PinholeCamera::Pixel2WorldUnit(const double &u,
                                               const double &v) const {
  return Pixel2Normal(u, v).normalized();
}

Eigen::Vector3d
PinholeCamera::Pixel2WorldUnit(const Eigen::Vector2d &uv) const {
  return Pixel2WorldUnit(uv[0], uv[1]);
}

Eigen::Vector3d PinholeCamera::Pixel2Normal(const double &u,
                                            const double &v) const {
  Eigen::Vector3d xyz;
  if (!distortion_) {
    xyz[0] = (u - cx_) / fx_;
    xyz[1] = (v - cy_) / fy_;
    xyz[2] = 1.0;
  } else {
    cv::Point2f uv(u, v), px;
    const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
    cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
    cv::undistortPoints(src_pt, dst_pt, K_cv_, D_cv_);
    xyz[0] = px.x;
    xyz[1] = px.y;
    xyz[2] = 1.0;
  }
  return xyz;
}

Eigen::Vector3d PinholeCamera::Pixel2Normal(const Eigen::Vector2d &uv) const {
  return Pixel2Normal(uv[0], uv[1]);
}

Eigen::Vector2d PinholeCamera::Camera2Pixel(const Eigen::Vector3d &xyz) const {
  return Camera2Pixel(Eigen::Vector2d(xyz.head<2>() / xyz[2]));
}

Eigen::Vector2d PinholeCamera::Camera2Pixel(const Eigen::Vector2d &xy) const {
  Eigen::Vector2d uv;
  if (!distortion_) {
    uv[0] = fx_ * xy[0] + cx_;
    uv[1] = fy_ * xy[1] + cy_;
  } else {
    double x, y, r2, r4, r6;
    x = xy[0];
    y = xy[1];
    r2 = x * x + y * y;
    r4 = r2 * r2;
    r6 = r4 * r2;
    double x_distorted = x * (1 + k1_ * r2 + k2_ * r4 + k3_ * r6) +
                         2 * p1_ * x * y + p2_ * (r2 + 2 * x * x);
    double y_distorted = y * (1 + k1_ * r2 + k2_ * r4 + k3_ * r6) +
                         p1_ * (r2 + 2 * y * y) + 2 * p2_ * x * y;
    uv[0] = x_distorted * fx_ + cx_;
    uv[1] = y_distorted * fy_ + cy_;
  }
  return uv;
}

void PinholeCamera::UndistortImage(const cv::Mat &raw, cv::Mat &rectified) {
  if (distortion_)
    cv::remap(raw, rectified, undist_map1_, undist_map2_, cv::INTER_LINEAR);
  else
    rectified = raw.clone();
}

// Eigen::Vector3d PinholeCamera::Pixel2WorldUnit(const double &u,
//                                                const double &v) const {
//   Eigen::Vector3d xyz;
//   if (!distortion_) {
//     xyz[0] = (u - cx_) / fx_;
//     xyz[1] = (v - cy_) / fy_;
//     xyz[2] = 1.0;
//   } else {
//     cv::Point2f uv(u, v), px;
//     const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
//     cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
//     cv::undistortPoints(src_pt, dst_pt, K_cv_, D_cv_);
//     xyz[0] = px.x;
//     xyz[1] = px.y;
//     xyz[2] = 1.0;
//   }
//   return xyz.normalized();
// }
