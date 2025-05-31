#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

#include <Eigen/Core>
#include <Eigen/Dense>

class CameraModel {
protected:
  int height_;
  int width_;

public:
  typedef std::shared_ptr<const CameraModel> ConstPtr;
  typedef std::shared_ptr<CameraModel> Ptr;

  CameraModel(){};
  CameraModel(int width, int height) : width_(width), height_(height){};
  virtual ~CameraModel(){};

  // from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Eigen::Vector3d Pixel2WorldUnit(const double &u,
                                          const double &v) const = 0;
  virtual Eigen::Vector3d Pixel2WorldUnit(const Eigen::Vector2d &uv) const = 0;

  virtual Eigen::Vector3d Pixel2Normal(const double &u,
                                       const double &v) const = 0;
  virtual Eigen::Vector3d Pixel2Normal(const Eigen::Vector2d &uv) const = 0;

  virtual Eigen::Vector2d Camera2Pixel(const Eigen::Vector3d &xyz) const = 0;
  virtual Eigen::Vector2d Camera2Pixel(const Eigen::Vector2d &uv) const = 0;

  virtual double ErrorMultiplier2() const = 0;
  virtual double ErrorMultiplier() const = 0;

  inline int width() const { return width_; }
  inline int height() const { return height_; }

  inline bool isInFrame(const Eigen::Vector2d &obs, int boundary = 0) const {
    if (obs[0] >= boundary && obs[0] < width() - boundary &&
        obs[1] >= boundary && obs[1] < height() - boundary)
      return true;
    return false;
  }

  inline bool isInFrame(const Eigen::Vector2d &obs, int boundary,
                        int level) const {
    if (obs[0] >= boundary && obs[0] < width() / (1 << level) - boundary &&
        obs[1] >= boundary && obs[1] < height() / (1 << level) - boundary)
      return true;
    return false;
  }
};

#endif // CAMERA_MODEL_H