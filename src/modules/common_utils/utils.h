#ifndef COMMON_UTILS_UTILS_H
#define COMMON_UTILS_UTILS_H

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

/**
 * @brief 根据指定的分隔符将字符串拆分成多个子字符串。
 *
 * 此函数将输入字符串 `str` 根据分隔符字符串 `delimiter`
 * 拆分成一个子字符串向量。 它可以处理多字符分隔符。
 *
 * @param str 要拆分的字符串。
 * @param delimiter 用作分隔符的字符串。
 * @return
 * 包含拆分后子字符串的向量。如果分隔符为空，则整个字符串作为一个元素返回。
 */
inline std::vector<std::string> SplitString(const std::string &str,
                                            const std::string &delimiter) {
  std::vector<std::string> tokens;
  if (delimiter.empty()) {
    tokens.emplace_back(str);
    return tokens;
  }

  size_t start = 0;
  size_t end = 0;
  size_t delimiter_length = delimiter.length();

  // 预估分割次数，使用 delimiter 的第一个字符出现次数作为近似值
  tokens.reserve(std::count(str.begin(), str.end(), delimiter[0]));

  while ((end = str.find(delimiter, start)) != std::string::npos) {
    tokens.emplace_back(str.substr(start, end - start));
    start = end + delimiter_length;
  }
  tokens.emplace_back(str.substr(start));

  return tokens;
}

/**
 * @brief 根据指定的字符分隔符将字符串拆分成多个子字符串。
 *
 * 此函数将输入字符串 `str` 根据字符分隔符 `delimiter` 拆分成一个子字符串向量。
 * 每次遇到 `delimiter` 时，字符串都会被分割。
 *
 * @param str 要拆分的字符串。
 * @param delimiter 用作分隔符的字符。
 * @return
 * 包含拆分后子字符串的向量。如果字符串中不包含分隔符，则整个字符串作为一个元素返回。
 */
inline std::vector<std::string> SplitString(const std::string &str,
                                            char delimiter) {
  std::vector<std::string> tokens; // 用于存储拆分后的子字符串

  size_t start = 0; // 当前子字符串的起始位置
  size_t end = 0;   // 分隔符在字符串中的位置

  // 预估分割次数，使用 delimiter 出现的次数作为向量预分配空间的近似值
  tokens.reserve(std::count(str.begin(), str.end(), delimiter));

  // 使用分隔符来分割字符串
  while ((end = str.find(delimiter, start)) != std::string::npos) {
    tokens.emplace_back(
        str.substr(start, end - start)); // 提取子字符串并添加到向量中
    start = end + 1; // 更新子字符串起始位置（因为是字符分隔符，所以加 1）
  }

  // 添加最后一个子字符串（从最后一个分隔符到字符串末尾）
  tokens.emplace_back(str.substr(start));

  return tokens; // 返回拆分后的子字符串向量
}

/**
 * @brief 去除字符串两端的空白字符。
 *
 * 此函数会从输入字符串 `s` 的开始和结束位置去除空白字符。
 * 空白字符包括空格、制表符、回车符和换行符等。
 * 如果输入字符串仅包含空白字符，则返回空字符串。
 *
 * @param s 要处理的输入字符串。
 * @return
 * 去除两端空白字符后的字符串。如果字符串中仅包含空白字符，则返回空字符串。
 */
static inline std::string Trim(const std::string &s) {
  size_t start = s.find_first_not_of(" \t\r\n"); // 找到第一个非空白字符的位置
  size_t end = s.find_last_not_of(" \t\r\n"); // 找到最后一个非空白字符的位置
  return (start == std::string::npos)
             ? ""
             : s.substr(start, end - start + 1); // 返回去除空白字符的子字符串
}

/**
 * @brief 将Eigen::MatrixXd矩阵转换为字符串。
 *
 * 此函数会将输入的Eigen::MatrixXd矩阵转换为一个格式化的字符串表示，
 * 每一行对应矩阵的一行，元素之间用空格分隔。
 *
 * @param matrix 要转换的Eigen::MatrixXd矩阵。
 * @return
 * 矩阵的字符串表示，每行对应矩阵的一行，元素之间用空格分隔。
 */
inline std::string EigenMatrixToString(const Eigen::MatrixXd &matrix) {
  std::stringstream ss;
  ss << matrix;
  return ss.str();
}

/**
 * @brief 将std::vector容器转换为字符串。
 *
 * 此模板函数会将输入的std::vector容器中的元素转换为一个空格分隔的字符串。
 * 支持任何可以被输出流操作符<<处理的类型T。
 *
 * @tparam T 向量中元素的类型。
 * @param vec 要转换的std::vector容器。
 * @return
 * 向量的字符串表示，元素之间用空格分隔。如果向量为空，则返回空字符串。
 */
template <typename T>
static inline std::string VectorToString(const std::vector<T> &vec) {
  std::stringstream ss;
  for (const auto &item : vec) {
    ss << item << " ";
  }
  std::string result = ss.str();
  // 去除末尾多余的空格
  if (!result.empty()) {
    result.pop_back();
  }
  return result;
}

/**
 * @brief 双线性插值函数，从图像中指定坐标获取插值结果
 *
 * 该函数从指定的图像中，根据给定的浮动坐标 (u, v)，进行双线性插值。
 * 适用于任意类型的图像数据
 * (例如，8位、32位等)，并能够根据需要处理不同类型的图像。
 *
 * @param mat 输入图像，类型为 `cv::Mat`，支持任意单通道类型。
 * @param u 水平方向的浮动坐标（即目标像素的横坐标）。
 * @param v 垂直方向的浮动坐标（即目标像素的纵坐标）。
 * @return 插值结果，类型为 `double`，代表目标位置的像素值。
 */
template <typename T>
double InterpolateMat(const cv::Mat &mat, double u, double v) {
  int x = floor(u);
  int y = floor(v);
  double subpix_x = u - x;
  double subpix_y = v - y;

  double w00 = (1.0f - subpix_x) * (1.0f - subpix_y);
  double w01 = (1.0f - subpix_x) * subpix_y;
  double w10 = subpix_x * (1.0f - subpix_y);
  double w11 = 1.0f - w00 - w01 - w10;

  const int stride = mat.step.p[0];
  unsigned char *ptr = mat.data + y * stride + x;
  T pixel00 = *(reinterpret_cast<T *>(ptr));
  T pixel01 = *(reinterpret_cast<T *>(ptr + stride));
  T pixel10 = *(reinterpret_cast<T *>(ptr + sizeof(T)));
  T pixel11 = *(reinterpret_cast<T *>(ptr + stride + sizeof(T)));

  return w00 * pixel00 + w01 * pixel01 + w10 * pixel10 + w11 * pixel11;
}

#endif // COMMON_UTILS_UTILS_H
