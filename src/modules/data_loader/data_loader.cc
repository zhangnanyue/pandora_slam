#include "data_loader.h"

// 标准库头文件
#include <filesystem>
#include <algorithm>
#include <fstream>
#include <unistd.h>
#include <regex>
#include <optional>

namespace fs = std::filesystem;

/**
 * @brief 加载单个图像文件
 * 
 * 使用OpenCV的imread函数加载图像，保持原始格式。
 * 如果加载失败，会打印错误信息。
 * 
 * @param file_path 图像文件路径
 * @param image 输出图像数据
 * @return 加载成功返回true，否则返回false
 */
bool DataLoaderApp::LoadImageFile(const std::string &file_path,
                                  cv::Mat &image) {
  // 使用IMREAD_UNCHANGED标志读取图像，保持原始格式
  image = cv::imread(file_path, cv::IMREAD_UNCHANGED);

  // 检查图像是否成功读取
  if (image.empty()) {
    PRINT_ERROR(RED, "Failed to load image from %s" RESET, file_path.c_str());
    return false;
  }
  return true;
}

/**
 * @brief 加载文本文件并逐行处理
 * 
 * 打开文本文件并逐行读取，对每一行调用回调函数进行处理。
 * 如果某行处理失败，会打印警告信息但继续处理后续行。
 * 
 * @param file_path 文本文件路径
 * @param parse_line_callback 行处理回调函数
 * @return 加载成功返回true，否则返回false
 */
bool DataLoaderApp::LoadTxtFile(const std::string &file_path,
                                const ParseLineCallback &parse_line_callback) {
  // 检查文件是否可读
  if (access(file_path.c_str(), R_OK) != 0) {
    PRINT_ERROR(RED, "Txt file is missing or not readable: %s\n" RESET,
                file_path.c_str());
    return false;
  }

  std::ifstream file(file_path);
  if (!file.is_open()) {
    PRINT_ERROR(RED, "Failed to load txt file from %s\n" RESET,
                file_path.c_str());
    return false;
  }

  std::string line;
  int line_number = 0;

  // 逐行读取文件内容
  while (std::getline(file, line)) {
    line_number++;
    if (!parse_line_callback(line)) {
      PRINT_WARNING(YELLOW
                    "Parsing error on line %d: %s. Skipping...\n" RESET,
                    line_number++, line.c_str());
    }
  }

  file.close();
  return true;
}

/**
 * @brief 从目录加载所有图像文件（懒加载版本）
 * 
 * 遍历指定目录中的所有文件，对符合扩展名要求的文件创建懒加载包装对象。
 * 支持自定义文件扩展名列表。
 * 
 * @param directory_path 目录路径
 * @param image_map 输出图像映射
 * @param extensions 支持的文件扩展名列表
 * @param pattern 正则表达式模式
 * @return 加载成功返回true，否则返回false
 */
bool DataLoaderApp::LoadImagesFromDirectory(const std::string& directory_path,
                                          ImageMap& image_map,
                                          const std::vector<std::string>& extensions,
                                          const std::string& pattern) {
    PRINT_INFO("Starting to load images from directory: %s\n", directory_path.c_str());

    // 检查目录是否存在
    if (!fs::exists(directory_path) || !fs::is_directory(directory_path)) {
        PRINT_ERROR(RED, "Directory does not exist: %s" RESET, directory_path.c_str());
        return false;
    }

    // 如果提供了正则表达式，创建正则对象
    std::optional<std::regex> filename_pattern;
    if (!pattern.empty()) {
        filename_pattern = std::regex(pattern);
    }

    // 遍历目录中的所有文件
    for (const auto& entry : fs::directory_iterator(directory_path)) {
        if (!entry.is_regular_file()) continue;

        std::string extension = entry.path().extension().string();
        // 转换为小写进行比较
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

        // 检查文件扩展名是否在允许的列表中
        if (std::find(extensions.begin(), extensions.end(), extension) != extensions.end()) {
            std::string filename = entry.path().filename().string();
            
            // 如果提供了正则表达式，尝试匹配并提取时间戳
            if (filename_pattern) {
                std::smatch matches;
                if (std::regex_search(filename, matches, *filename_pattern) && matches.size() > 1) {
                    filename = matches[1].str();
                }
            }

            // 创建懒加载包装对象
            double timestamp = std::stod(filename);
            image_map[timestamp] = std::make_shared<LazyImage>(entry.path().string());
            // PRINT_DEBUG("Loaded timestamp: %f, filename: %s\n", timestamp, filename.c_str());
        }
    }

    if (image_map.empty()) {
        PRINT_WARNING(YELLOW "No valid images found in directory: %s" RESET, directory_path.c_str());
        return false;
    }

    PRINT_INFO("Successfully loaded %zu images\n", image_map.size());
    return true;
}

/**
 * @brief 从目录加载所有点云文件（懒加载版本）
 * 
 * 遍历指定目录中的所有文件，对符合扩展名要求的文件创建懒加载包装对象。
 * 支持自定义文件扩展名列表和点云类型。
 * 
 * @tparam PointT 点云点类型
 * @param directory_path 目录路径
 * @param point_cloud_map 输出点云映射
 * @param extensions 支持的文件扩展名列表
 * @return 加载成功返回true，否则返回false
 */
template<typename PointT>
bool DataLoaderApp::LoadPointCloudsFromDirectory(const std::string& directory_path,
                                               PointCloudMap<PointT>& point_cloud_map,
                                               const std::vector<std::string>& extensions) {
    try {
        // 检查目录是否存在
        if (!fs::exists(directory_path) || !fs::is_directory(directory_path)) {
            PRINT_ERROR(RED, "Directory does not exist: %s" RESET, directory_path.c_str());
            return false;
        }

        // 遍历目录中的所有文件
        for (const auto& entry : fs::directory_iterator(directory_path)) {
            if (!entry.is_regular_file()) continue;

            std::string extension = entry.path().extension().string();
            // 转换为小写进行比较
            std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

            // 检查文件扩展名是否在允许的列表中
            if (std::find(extensions.begin(), extensions.end(), extension) != extensions.end()) {
                std::string filename = entry.path().filename().string();
                // 创建懒加载包装对象
                try {
                    double timestamp = std::stod(filename);
                    point_cloud_map[timestamp] = std::make_shared<LazyPointCloud<PointT>>(entry.path().string());
                } catch (const std::exception& e) {
                    PRINT_WARNING("Failed to parse timestamp from filename: %s, error: %s\n", 
                                filename.c_str(), e.what());
                    continue;
                }
            }
        }

        if (point_cloud_map.empty()) {
            PRINT_WARNING(YELLOW "No valid point clouds found in directory: %s" RESET, directory_path.c_str());
            return false;
        }

        return true;
    } catch (const std::exception& e) {
        PRINT_ERROR(RED, "Error loading point clouds from directory: %s" RESET, e.what());
        return false;
    }
}

/**
 * @brief 使用KD树提取点云中的局部区域
 * 
 * 使用KD树进行半径搜索，提取指定点周围指定半径内的所有点。
 * 
 * @tparam T 点云点类型
 * @param point_cloud 输入点云
 * @param query_point 查询点
 * @param radius 搜索半径
 * @param subset_cloud 输出局部点云
 * @return 提取成功返回true，否则返回false
 */
template <class T>
bool DataLoaderApp::ExtractRegionPointCloudByKdTree(
    const typename pcl::PointCloud<T>::ConstPtr point_cloud,
    const T &query_point, double radius,
    typename pcl::PointCloud<T>::Ptr subset_cloud) {

  static_assert(pcl::traits::has_xyz<T>::value,
                "T must be a valid PCL point type");

  // 检查输入点云是否为空
  if (point_cloud->empty()) {
    PRINT_ERROR(RED, "Input point cloud is empty." RESET);
    return false;
  }

  // 创建 KdTree 对象并设置输入点云
  pcl::search::KdTree<T> kdtree;
  kdtree.setInputCloud(point_cloud);

  // 存储在半径内找到的点的索引和距离
  std::vector<int> point_indices;
  std::vector<float> point_squared_distances;

  // 执行半径搜索
  int num_points_found = kdtree.radiusSearch(query_point, radius, point_indices,
                                             point_squared_distances);

  if (num_points_found <= 0) {
    PRINT_ERROR(RED, "No points found within the radius of %.2f." RESET,
                radius);
    return false;
  }

  // 提取子点云
  subset_cloud->points.reserve(num_points_found);
  for (int idx : point_indices) {
    subset_cloud->points.push_back(point_cloud->points[idx]);
  }
  subset_cloud->width = static_cast<uint32_t>(subset_cloud->points.size());
  subset_cloud->height = 1;
  subset_cloud->is_dense = false; // 如果可能存在无效点，设置为 false

  return true;
}

/**
 * @brief 加载单个点云文件
 * 
 * 使用PCL库加载点云文件，支持不同类型的点云。
 * 
 * @tparam T 点云点类型
 * @param pcd_path 点云文件路径
 * @param point_cloud 输出点云数据
 * @return 加载成功返回true，否则返回false
 */
template <class T>
bool DataLoaderApp::LoadPcdFile(const std::string &pcd_path,
                                typename pcl::PointCloud<T>::Ptr point_cloud) {
  static_assert(pcl::traits::has_xyz<T>::value,
                "T must be a valid PCL point type");

  if (pcl::io::loadPCDFile<T>(pcd_path, *point_cloud) == -1) {
    PRINT_ERROR(RED, "Failed to load PCD from %s" RESET, pcd_path.c_str());
    return false;
  }
  return true;
}

// 显式实例化常用的点云类型
template bool DataLoaderApp::LoadPointCloudsFromDirectory<pcl::PointXYZ>(
    const std::string&, PointCloudMap<pcl::PointXYZ>&, const std::vector<std::string>&);
template bool DataLoaderApp::LoadPointCloudsFromDirectory<pcl::PointXYZI>(
    const std::string&, PointCloudMap<pcl::PointXYZI>&, const std::vector<std::string>&);
template bool DataLoaderApp::LoadPointCloudsFromDirectory<pcl::PointXYZINormal>(
    const std::string&, PointCloudMap<pcl::PointXYZINormal>&, const std::vector<std::string>&);
template bool DataLoaderApp::LoadPointCloudsFromDirectory<pcl::PointXYZRGB>(
    const std::string&, PointCloudMap<pcl::PointXYZRGB>&, const std::vector<std::string>&);