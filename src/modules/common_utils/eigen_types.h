#ifndef COMMON_UTILS_EIGEN_TYPES_H
#define COMMON_UTILS_EIGEN_TYPES_H

#include <Eigen/Core>

using Vec3d = Eigen::Matrix<double, 3, 1>;
using Vec2d = Eigen::Matrix<double, 2, 1>;
using Vec2f = Eigen::Matrix<float, 2, 1>;
using Vec4d = Eigen::Matrix<double, 4, 1>;
using Mat3d = Eigen::Matrix<double, 3, 3>;
using Mat4d = Eigen::Matrix<double, 4, 4>;
using Mat18d = Eigen::Matrix<double, 18, 18>;
using VecXd = Eigen::VectorXd;
using MatXd = Eigen::MatrixXd;
using Mat12d = Eigen::Matrix<double, 1, 2>;
using Mat14d = Eigen::Matrix<double, 1, 4>;
using Mat13d = Eigen::Matrix<double, 1, 3>;
using Mat16d = Eigen::Matrix<double, 1, 6>;
using Mat26d = Eigen::Matrix<double, 2, 6>;
using Mat23d = Eigen::Matrix<double, 2, 3>;
using Mat43d = Eigen::Matrix<double, 4, 3>;
using Mat46d = Eigen::Matrix<double, 4, 6>;
using Mat63d = Eigen::Matrix<double, 6, 3>;

#endif // COMMON_UTILS_EIGEN_TYPES_H