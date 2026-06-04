#ifndef COMMON_UTILS_MATH_UTILS_H
#define COMMON_UTILS_MATH_UTILS_H

#include <Eigen/Dense>
#include <iostream>
#include <limits>
#include <vector>
#include <map>

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

/**
 * @brief 通用线性插值函数
 * @tparam T 数据类型
 * @param start 起始值
 * @param end 结束值
 * @param alpha 插值系数 [0,1]
 * @return 插值结果
 */
template <typename T>
T LinearInterpolate(const T& start, const T& end, double alpha) {
    return start + alpha * (end - start);
}

/**
 * @brief 通用向量插值函数（用于加速度和角速度）
 * @tparam T 数据类型
 * @param start 起始向量
 * @param end 结束向量
 * @param alpha 插值系数 [0,1]
 * @return 插值后的向量
 */
template <typename T>
Eigen::Matrix<T, 3, 1> VectorInterpolate(const Eigen::Matrix<T, 3, 1>& start,
                                        const Eigen::Matrix<T, 3, 1>& end,
                                        double alpha) {
    return LinearInterpolate(start, end, alpha);
}

/**
 * @brief 通用时间序列插值函数
 * @tparam T 数据类型
 * @param time_series 时间序列数据，key为时间戳，value为对应的向量值
 * @param timestamp 需要插值的时间戳
 * @return 插值后的向量，如果时间戳超出范围则返回零向量
 */
template <typename T>
Eigen::Matrix<T, 3, 1> InterpolateTimeSeries(
    const std::map<double, Eigen::Matrix<T, 3, 1>>& time_series,
    double timestamp) {
    
    auto it_next = time_series.upper_bound(timestamp);
    if (it_next == time_series.end() || it_next == time_series.begin()) {
        return Eigen::Matrix<T, 3, 1>::Zero();
    }
    
    auto it_prev = std::prev(it_next);
    double alpha = (timestamp - it_prev->first) / (it_next->first - it_prev->first);
    
    return VectorInterpolate(it_prev->second, it_next->second, alpha);
}

/**
 * @brief 计算三维空间中两点之间的欧氏距离
 *
 * 该函数使用 std::hypot 计算点 \p a 和点 \p b 之间的三维欧氏距离，
 * 相较于直接使用 std::sqrt(dx*dx + dy*dy + dz*dz)，
 * std::hypot 在数值上更加稳定，可减少极端情况下的溢出或下溢风险。
 *
 * @param[in] a 第一个三维坐标点，类型为 Eigen::Vector3d
 * @param[in] b 第二个三维坐标点，类型为 Eigen::Vector3d
 * @return 两点之间的欧氏距离（double 型）
 */
inline double ComputeEuclideanDistance3D(const Eigen::Vector3d &a,
                                          const Eigen::Vector3d &b) {
    return std::hypot(b(0) - a(0),
                      b(1) - a(1),
                      b(2) - a(2));
}



#endif // COMMON_UTILS_MATH_UTILS_H
