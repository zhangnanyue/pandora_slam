#ifndef DATA_LOADER_BASE_APP_H
#define DATA_LOADER_BASE_APP_H

// 标准库头文件
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

// 第三方库头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

// 项目头文件
#include "common_utils/print.h"

// 前向声明
namespace pcl {
template <typename PointT>
class PointCloud;
}

// 回调函数类型定义，用于加载文本文件 const std::string& - 文件中的一行文本
using ParseLineCallback = std::function<bool(const std::string&)>;

/**
 * @brief 数据加载器类，用于加载和管理各种类型的数据（图像、点云等）
 *
 * 该类提供了以下主要功能：
 * 1. 加载单个图像、点云和文本文件
 * 2. 从目录批量加载图像和点云数据
 * 3. 支持点云数据的区域提取
 * 4. 提供懒加载机制，优化内存使用
 */
class DataLoaderApp {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 构造函数和析构函数
    DataLoaderApp() = default;
    virtual ~DataLoaderApp() = default;

    // 禁用拷贝构造和赋值操作
    DataLoaderApp(const DataLoaderApp&) = delete;
    DataLoaderApp& operator=(const DataLoaderApp&) = delete;

    // 智能指针类型定义
    typedef std::shared_ptr<const DataLoaderApp> ConstPtr;
    typedef std::shared_ptr<DataLoaderApp> Ptr;

    // 点云类型定义
    using PointcloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
    using PointcloudXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
    using PointcloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
    using PointcloudXYZIPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;
    using PointcloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;
    using PointcloudXYZINPtr = pcl::PointCloud<pcl::PointXYZINormal>::Ptr;
    using PointcloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
    using PointcloudXYZRGBPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;

    // 向量和矩阵类型定义
    using Vec2d = Eigen::Matrix<double, 2, 1>;
    using Vec3d = Eigen::Matrix<double, 3, 1>;
    using Vec3f = Eigen::Matrix<float, 3, 1>;
    using Vec4d = Eigen::Matrix<double, 4, 1>;
    using Vec4f = Eigen::Matrix<float, 4, 1>;
    using Mat3d = Eigen::Matrix<double, 3, 3>;
    using Mat4d = Eigen::Matrix<double, 4, 4>;

    /**
     * @brief 图像数据懒加载包装类
     *
     * 该类实现了图像的懒加载机制，只在首次访问时才加载图像数据。
     * 主要功能：
     * 1. 存储图像文件路径
     * 2. 按需加载图像数据
     * 3. 提供内存管理功能
     */
    class LazyImage {
       public:
        /**
         * @brief 构造函数
         * @param file_path 图像文件路径
         */
        explicit LazyImage(const std::string& file_path) : file_path_(file_path), image_(nullptr) {}

        /**
         * @brief 获取图像数据
         * @return 图像数据的常量引用
         *
         * 如果图像未加载，则首次调用时会加载图像数据
         */
        const cv::Mat& GetImage() const {
            if (!image_) {
                image_ = std::make_shared<cv::Mat>();
                if (!LoadImageFile(file_path_, *image_)) {
                    PRINT_ERROR(RED, "Failed to load image: %s" RESET, file_path_.c_str());
                }
            }
            return *image_;
        }

        /**
         * @brief 获取文件路径
         * @return 文件路径的常量引用
         */
        const std::string& GetFilePath() const {
            return file_path_;
        }

        /**
         * @brief 检查图像是否已加载
         * @return 如果图像已加载返回true，否则返回false
         */
        bool IsLoaded() const {
            return image_ != nullptr;
        }

        /**
         * @brief 清理图像数据
         *
         * 释放已加载的图像数据，但保留文件路径
         */
        void Clear() {
            image_.reset();
        }

       private:
        std::string file_path_;                   ///< 图像文件路径
        mutable std::shared_ptr<cv::Mat> image_;  ///< 图像数据（使用mutable允许在const方法中修改）

        // 静态函数用于加载图像
        static bool LoadImageFile(const std::string& file_path, cv::Mat& image) {
            image = cv::imread(file_path, cv::IMREAD_UNCHANGED);
            if (image.empty()) {
                PRINT_ERROR(RED, "Failed to load image from %s" RESET, file_path.c_str());
                return false;
            }
            return true;
        }
    };

    /**
     * @brief 点云数据懒加载包装类（模板）
     *
     * 该类实现了点云数据的懒加载机制，支持不同类型的点云。
     * 主要功能：
     * 1. 存储点云文件路径
     * 2. 按需加载点云数据
     * 3. 提供内存管理功能
     *
     * @tparam PointT 点云点类型（如PointXYZ, PointXYZRGB等）
     */
    template <typename PointT>
    class LazyPointCloud {
       public:
        /**
         * @brief 构造函数
         * @param file_path 点云文件路径
         */
        explicit LazyPointCloud(const std::string& file_path) : file_path_(file_path), cloud_(nullptr) {}

        /**
         * @brief 获取点云数据
         * @return 点云数据的智能指针
         *
         * 如果点云未加载，则首次调用时会加载点云数据
         */
        const typename pcl::PointCloud<PointT>::Ptr& GetPointCloud() const {
            if (!cloud_) {
                cloud_.reset(new pcl::PointCloud<PointT>);
                if (!DataLoaderApp::LoadPcdFile<PointT>(file_path_, cloud_)) {
                    PRINT_ERROR(RED, "Failed to load point cloud: %s" RESET, file_path_.c_str());
                }
            }
            return cloud_;
        }

        /**
         * @brief 获取文件路径
         * @return 文件路径的常量引用
         */
        const std::string& GetFilePath() const {
            return file_path_;
        }

        /**
         * @brief 检查点云是否已加载
         * @return 如果点云已加载返回true，否则返回false
         */
        bool IsLoaded() const {
            return cloud_ != nullptr;
        }

        /**
         * @brief 清理点云数据
         *
         * 释放已加载的点云数据，但保留文件路径
         */
        void Clear() {
            cloud_.reset();
        }

       private:
        std::string file_path_;                                ///< 点云文件路径
        mutable typename pcl::PointCloud<PointT>::Ptr cloud_;  ///< 点云数据
    };

    // 图像数据映射类型定义
    using ImageMap = std::map<double, std::shared_ptr<LazyImage>>;

    // 点云数据映射类型（使用懒加载包装类）
    template <typename PointT>
    using PointCloudMap = std::map<double, std::shared_ptr<LazyPointCloud<PointT>>>;

   public:
    /**
     * @brief 加载单个点云文件
     * @tparam T 点云点类型
     * @param pcd_path 点云文件路径
     * @param point_cloud 输出点云数据
     * @return 加载成功返回true，否则返回false
     */
    template <class T>
    bool LoadPcdFile(const std::string& pcd_path, typename pcl::PointCloud<T>::Ptr point_cloud);

    /**
     * @brief 加载单个图像文件
     * @param file_path 图像文件路径
     * @param image 输出图像数据
     * @return 加载成功返回true，否则返回false
     */
    virtual bool LoadImageFile(const std::string& file_path, cv::Mat& image);

    /**
     * @brief 加载文本文件并逐行处理
     * @param file_path 文本文件路径
     * @param parse_line_callback 行处理回调函数
     * @return 加载成功返回true，否则返回false
     */
    virtual bool LoadTxtFile(const std::string& file_path, const ParseLineCallback& parse_line_callback);

    /**
     * @brief 使用KD树提取点云中的局部区域
     * @tparam T 点云点类型
     * @param point_cloud 输入点云
     * @param query_point 查询点
     * @param radius 搜索半径
     * @param subset_cloud 输出局部点云
     * @return 提取成功返回true，否则返回false
     */
    template <class T>
    bool ExtractRegionPointCloudByKdTree(const typename pcl::PointCloud<T>::ConstPtr point_cloud,
                                         const T& query_point,
                                         double radius,
                                         typename pcl::PointCloud<T>::Ptr subset_cloud);

    /**
     * @brief 从目录加载所有图像文件（懒加载版本）
     * @param directory_path 目录路径
     * @param image_map 输出图像映射
     * @param extensions 支持的文件扩展名列表
     * @param pattern 可选的正则表达式
     * @return 加载成功返回true，否则返回false
     */
    bool LoadImagesFromDirectory(const std::string& directory_path,
                                 ImageMap& image_map,
                                 const std::vector<std::string>& extensions,
                                 const std::string& pattern = "");

    /**
     * @brief 从目录加载所有点云文件（懒加载版本）
     * @tparam PointT 点云点类型
     * @param directory_path 目录路径
     * @param point_cloud_map 输出点云映射
     * @param extensions 支持的文件扩展名列表
     * @return 加载成功返回true，否则返回false
     */
    template <typename PointT>
    bool LoadPointCloudsFromDirectory(const std::string& directory_path,
                                      PointCloudMap<PointT>& point_cloud_map,
                                      const std::vector<std::string>& extensions = {".pcd"});
};

#endif  // DATA_LOADER_BASE_APP_H
