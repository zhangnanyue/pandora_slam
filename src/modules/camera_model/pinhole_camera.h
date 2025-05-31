#ifndef CAMERA_MODEL_PINHOLE_CAMERA_H
#define CAMERA_MODEL_PINHOLE_CAMERA_H

#include <opencv2/opencv.hpp>

#include "camera_model.h"

class PinholeCamera : public CameraModel {
private:
  double fx_, fy_, cx_, cy_;
  bool distortion_;
  double k1_, k2_, p1_, p2_, k3_;
  cv::Mat K_cv_, D_cv_;
  cv::Mat undist_map1_, undist_map2_;
  bool use_optimization_;
  Eigen::Matrix3d K_;
  Eigen::Matrix3d K_inv_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PinholeCamera(const double &width, const double &height, const double &fx,
                const double &fy, const double &cx, const double &cy,
                const double &k1, const double &k2, const double &p1,
                const double &p2, const double &k3, bool is_distortion = true);
  ~PinholeCamera();

  void initUnistortionMap();

  Eigen::Vector3d Pixel2WorldUnit(const double &u,
                                  const double &v) const override;
  Eigen::Vector3d Pixel2WorldUnit(const Eigen::Vector2d &uv) const override;

  Eigen::Vector3d Pixel2Normal(const double &u, const double &v) const override;
  Eigen::Vector3d Pixel2Normal(const Eigen::Vector2d &uv) const override;

  Eigen::Vector2d Camera2Pixel(const Eigen::Vector3d &xyz) const override;
  Eigen::Vector2d Camera2Pixel(const Eigen::Vector2d &uv) const override;

  double ErrorMultiplier2() const override { return fabs(fx_); }
  double ErrorMultiplier() const override { return fabs(4.0 * fx_ * fy_); }

  inline const Eigen::Matrix3d &K() const { return K_; };
  inline const Eigen::Matrix3d &K_inv() const { return K_inv_; };

  inline double fx() const { return fx_; };
  inline double fy() const { return fy_; };
  inline double cx() const { return cx_; };
  inline double cy() const { return cy_; };
  inline double k1() const { return k1_; };
  inline double k2() const { return k2_; };
  inline double p1() const { return p1_; };
  inline double p2() const { return p2_; };
  inline double k3() const { return k3_; };

  void UndistortImage(const cv::Mat &raw, cv::Mat &rectified);
};

#endif // CAMERA_MODEL_PINHOLE_CAMERA_H