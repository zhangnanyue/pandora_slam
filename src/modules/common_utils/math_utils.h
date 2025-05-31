#ifndef COMMON_UTILS_MATH_UTILS_H
#define COMMON_UTILS_MATH_UTILS_H

#include <Eigen/Dense>
#include <iostream>
#include <limits>
#include <vector>

/**
 * @file math_utils.h
 * @brief 提供通用的数学工具函数。
 */

/**
 * @brief 使用克拉默法则求解线性方程组 Ax = b。
 *
 * 此函数使用克拉默法则求解线性方程组 Ax = b，返回求解后的向量 x。
 * 如果系数矩阵的行列式为零，则方程组无唯一解，返回 false。
 *
 * @tparam EigenMatrixType Eigen 矩阵类型，要求为方阵。
 * @tparam EigenVectorType Eigen 向量类型。
 * @param A 系数矩阵。
 * @param b 常数项向量。
 * @param x 求解后的未知数向量。
 * @return 如果方程组有唯一解，返回 true，否则返回 false。
 */
template <typename EigenMatrixType, typename EigenVectorType>
bool SolveLinearSystemCramer(const EigenMatrixType &A, const EigenVectorType &b,
                             EigenVectorType &x) {
  typedef typename EigenMatrixType::Scalar Scalar;
  const Scalar detA = A.determinant();

  // 检查行列式是否为零
  if (std::abs(detA) < std::numeric_limits<Scalar>::epsilon()) {
    std::cerr << "系数矩阵的行列式为零，方程组无唯一解。" << std::endl;
    return false;
  }
  const int n = A.cols();
  x.resize(n);
  for (int i = 0; i < n; ++i) {
    EigenMatrixType Ai = A;
    Ai.col(i) = b;
    Scalar detAi = Ai.determinant();
    x(i) = detAi / detA;
  }
  return true;
}

/**
 * @brief 计算二维点集的主要方向。
 *
 * 此函数通过计算点集的均值点，构建协方差矩阵，进行特征值分解，
 * 并提取最大特征值对应的特征向量作为点集的主要方向。
 *
 * @tparam T 标量类型（例如，float，double）。
 * @param points 包含二维点的向量，类型为 std::vector<Eigen::Matrix<T, 2, 1>>。
 * @param direction 输出参数，用于存储计算得到的主要方向，类型为
 * Eigen::Matrix<T, 2, 1>。
 */
template <typename T>
void CalculatePrincipalDirection(
    const std::vector<Eigen::Matrix<T, 2, 1>> &points,
    Eigen::Matrix<T, 2, 1> &direction) {
  // 计算均值点
  Eigen::Matrix<T, 2, 1> mean_point = Eigen::Matrix<T, 2, 1>::Zero();
  for (const auto &point : points) {
    mean_point += point;
  }
  mean_point /= static_cast<T>(points.size());

  // 构建协方差矩阵
  Eigen::Matrix<T, 2, 2> covariance_matrix = Eigen::Matrix<T, 2, 2>::Zero();
  for (const auto &point : points) {
    Eigen::Matrix<T, 2, 1> diff = point - mean_point;
    covariance_matrix += diff * diff.transpose();
  }

  // 进行特征值分解
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 2, 2>> eigen_solver(
      covariance_matrix);
  if (eigen_solver.info() != Eigen::Success) {
    // 如果特征值分解失败，将方向设置为零向量
    direction.setZero();
    return;
  }

  // 找到最大特征值的索引
  int max_eigenvalue_index;
  eigen_solver.eigenvalues().maxCoeff(&max_eigenvalue_index);

  // 提取对应的特征向量作为主要方向
  direction = eigen_solver.eigenvectors().col(max_eigenvalue_index);
}

/**
 * @brief 解一个 3x3 的线性方程组 Ax = b。
 *
 * 此函数通过逆矩阵法解 3x3 线性方程组 Ax = b。如果系数矩阵不可逆，
 * 则方程组无唯一解，返回 false。
 *
 * @param matrix 包含系数矩阵和常数项向量的 3x4 矩阵，其中前 3 列为系数矩阵 A，
 *               第 4 列为常数项向量 b。
 * @param solution 存储求解后的 3x1 向量 x。
 * @return 如果方程组有唯一解，返回 true，否则返回 false。
 */
bool Solve3x3LinearSystem(const Eigen::Matrix<double, 3, 4> &matrix,
                          Eigen::Vector3d &solution) {
  Eigen::Matrix3d A;
  Eigen::Vector3d b;

  for (int i = 0; i < 3; ++i) {
    A(i, 0) = matrix(i, 0);
    A(i, 1) = matrix(i, 1);
    A(i, 2) = matrix(i, 2);
    b(i) = matrix(i, 3);
  }

  Eigen::FullPivLU<Eigen::Matrix3d> lu_decomp(A);
  if (lu_decomp.isInvertible()) {
    solution = lu_decomp.solve(b);
    return true;
  } else {
    return false;
  }
}

#endif // COMMON_UTILS_MATH_UTILS_H
