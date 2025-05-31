#ifndef COMMON_UTILS_OPENCV_YAML_PARSER_H
#define COMMON_UTILS_OPENCV_YAML_PARSER_H

#include <Eigen/Eigen>
#include <filesystem>
#include <memory>
#include <opencv2/opencv.hpp>

#include "colors.h"
#include "print.h"
#include "utils.h"

/**
 * @brief 用于从文件解析 OpenCV YAML 的辅助类。
 *
 * 逻辑如下：
 * - 给定主配置文件的路径，我们将其加载到
 * [cv::FileStorage](https://docs.opencv.org/4.x/da/d56/classcv_1_1FileStorage.html)
 * 对象中。
 */
class YamlParser {
public:
  typedef std::shared_ptr<const YamlParser> ConstPtr;
  typedef std::shared_ptr<YamlParser> Ptr;

  /**
   * @brief 构造函数，加载配置文件
   * @param config_path YAML 文件的路径
   * @param fail_if_not_found 如果无法打开配置文件，是否终止程序
   */
  explicit YamlParser(const std::string &config_path,
                      bool fail_if_not_found = true)
      : config_path_(config_path) {

    // 检查文件是否存在
    if (!fail_if_not_found && !std::filesystem::exists(config_path)) {
      config_ = nullptr;
      return;
    }
    if (!std::filesystem::exists(config_path)) {
      PRINT_ERROR(RED "unable to open the configuration file!\n%s\n" RESET,
                  config_path.c_str());
      std::exit(EXIT_FAILURE);
    }

    // 打开文件，若无法打开则报错
    config_ =
        std::make_shared<cv::FileStorage>(config_path, cv::FileStorage::READ);
    if (!fail_if_not_found && !config_->isOpened()) {
      config_ = nullptr;
      return;
    }
    if (!config_->isOpened()) {
      PRINT_ERROR(RED "unable to open the configuration file!\n%s\n" RESET,
                  config_path.c_str());
      std::exit(EXIT_FAILURE);
    }
  }

  /**
   * @brief 获取配置文件所在的文件夹路径
   * @return 配置文件夹路径
   */
  std::string GetConfigFolder() {
    auto pos = config_path_.find_last_of("/\\");
    if (pos == std::string::npos)
      return "./"; // 若找不到路径分隔符，返回当前目录
    return config_path_.substr(0, pos + 1);
  }

  /**
   * @brief 检查所有参数是否成功读取
   * @return 如果所有参数都找到则返回 True
   */
  bool Successful() const { return all_params_found_successfully_; }

  /**
   * @brief 自定义解析器，用于 ESTIMATOR 参数。
   *
   * 这将从主配置文件加载数据。
   * 如果无法找到，将给出警告提示用户未找到。
   *
   * @tparam T 要查找的参数类型。
   * @param node_name 节点名称
   * @param node_result 结果值（应该已经有默认值）
   * @param required 如果此参数是用户必须设置的，则为 true
   */
  template <class T>
  void ParseConfig(const std::string &node_name, T &node_result,
                   bool required = true) {
    ParseConfigYaml(node_name, node_result, required);
  }

  // /**
  //  * @brief 自定义解析器，用于带有层级的外部参数文件。
  //  *
  //  * 这将首先加载请求的外部文件。
  //  * 然后它将尝试在第一个级别（例如 imu0、cam0、cam1）下查找请求的节点。
  //  *
  //  * @tparam T 要查找的参数类型。
  //  * @param external_node_name 外部节点名称，用于获取相对路径
  //  * @param sensor_name 第一个级别节点名称
  //  * @param node_name 节点名称
  //  * @param node_result 结果值（应该已经有默认值）
  //  * @param required 如果此参数是用户必须设置的，则为 true
  //  */
  // template <class T>
  // void ParseExternal(const std::string &external_node_name,
  //                     const std::string &sensor_name,
  //                     const std::string &node_name, T &node_result,
  //                     bool required = true) {
  //   ParseExternalYaml(external_node_name, sensor_name, node_name,
  //   node_result,
  //                       required);
  // }

  // /**
  //  * @brief 自定义解析器，用于外部参数文件中的 Matrix3d。
  //  *
  //  * 这将首先加载请求的外部文件。
  //  * 然后它将尝试在第一个级别（例如 imu0、cam0、cam1）下查找请求的节点。
  //  *
  //  * @param external_node_name 外部节点名称，用于获取相对路径
  //  * @param sensor_name 第一个级别节点名称
  //  * @param node_name 节点名称
  //  * @param node_result 结果值（应该已经有默认值）
  //  * @param required 如果此参数是用户必须设置的，则为 true
  //  */
  // void ParseExternal(const std::string &external_node_name,
  //                     const std::string &sensor_name,
  //                     const std::string &node_name,
  //                     Eigen::Matrix3d &node_result, bool required = true) {

  //   // 从 YAML 文件中解析
  //   ParseExternalYaml(external_node_name, sensor_name, node_name,
  //   node_result,
  //                       required);
  // }

  //   /**
  //    * @brief 自定义解析器，用于外部参数文件中的 Matrix4d。
  //    *
  //    * 这将首先加载请求的外部文件。
  //    * 然后它将尝试在第一个级别（例如 imu0、cam0、cam1）下查找请求的节点。
  //    *
  //    * @param external_node_name 外部节点名称，用于获取相对路径
  //    * @param sensor_name 第一个级别节点名称
  //    * @param node_name 节点名称
  //    * @param node_result 结果值（应该已经有默认值）
  //    * @param required 如果此参数是用户必须设置的，则为 true
  //    */
  //   void ParseExternal(const std::string &external_node_name, const
  //   std::string &sensor_name, const std::string &node_name,
  //                       Eigen::Matrix4d &node_result, bool required = true) {

  //     // 从 YAML 文件中解析
  //     ParseExternalYaml(external_node_name, sensor_name, node_name,
  //     node_result, required);
  //   }

private:
  /// 配置文件路径
  std::string config_path_;

  /// 配置文件内容
  std::shared_ptr<cv::FileStorage> config_;

  /// 记录所有参数是否成功找到
  bool all_params_found_successfully_ = true;

  /**
   * @brief 检查 YAML 节点对象中是否存在有效键
   * @param file_node OpenCV 文件节点
   * @param node_name 节点名称
   * @return 如果找到数据则返回 True
   */
  static bool NodeFound(const cv::FileNode &file_node,
                        const std::string &node_name) {
    bool found_node = false;
    for (const auto &item : file_node) {
      if (item.name() == node_name) {
        found_node = true;
        break;
      }
    }
    return found_node;
  }

  /**
   * @brief 从配置文件解析参数。
   *
   * 如果无法找到，将给出警告提示用户未找到。
   *
   * @tparam T 要查找的参数类型。
   * @param node_name 节点名称
   * @param node_result 结果值（应该已经有默认值）
   * @param required 如果此参数是用户必须设置的，则为 true
   */
  template <class T>
  void ParseConfigYaml(const std::string &node_name, T &node_result,
                       bool required = true) {

    // 如果配置未打开，直接返回
    if (config_ == nullptr)
      return;

    // 解析节点路径，按 '.' 分割
    std::vector<std::string> nodes = SplitString(node_name, '.');
    if (nodes.empty()) {
      PRINT_WARNING(YELLOW "Invalid node name \"%s\".\n" RESET,
                    node_name.c_str());
      all_params_found_successfully_ = false;
      return;
    }

    // 从根节点开始逐层访问
    cv::FileNode current_node = config_->root();
    for (size_t i = 0; i < nodes.size() - 1; ++i) {
      if (current_node[nodes[i]].empty()) {
        if (required) {
          PRINT_WARNING(YELLOW "Node \"%s\" not found.\n" RESET,
                        node_name.c_str());
          all_params_found_successfully_ = false;
        } else {
          PRINT_DEBUG("Node \"%s\" not found (not required).\n",
                      node_name.c_str());
        }
        return;
      }
      current_node = current_node[nodes[i]];
    }

    // 最后一个节点名称
    std::string final_node_name = nodes.back();

    // 尝试从配置中获取
    try {
      Parse(current_node, final_node_name, node_result, required);
      if constexpr (std::is_same<T, std::string>::value) {
        size_t pos = node_result.find('#');
        if (pos != std::string::npos) {
          node_result = node_result.substr(0, pos);
          node_result = Trim(node_result);
        }
      }
    } catch (...) {
      PRINT_WARNING(YELLOW "unable to parse \"%s\" node of type [%s] in the "
                           "config file!\n" RESET,
                    typeid(node_result).name(), node_name.c_str());
      all_params_found_successfully_ = false;
    }
  }

  /**
   * @brief 尝试从配置中获取请求的参数。
   *
   * 如果无法找到，将给出警告提示用户未找到。
   *
   * @tparam T 要查找的参数类型(int double float string)。
   * @param file_node OpenCV 文件节点
   * @param node_name 节点名称
   * @param node_result 结果值（应该已经有默认值）
   * @param required 如果此参数是用户必须设置的，则为 true
   */
  template <class T>
  void Parse(const cv::FileNode &file_node, const std::string &node_name,
             T &node_result, bool required = true) {

    // 检查是否存在请求的节点
    if (!NodeFound(file_node, node_name)) {
      if (required) {
        PRINT_WARNING(YELLOW
                      "the node \"%s\" of type [%s] was not found...\n" RESET,
                      typeid(node_result).name(), node_name.c_str());
        all_params_found_successfully_ = false;
      } else {
        PRINT_DEBUG(
            "the node \"%s\" of type [%s] was not found (not required)...\n",
            typeid(node_result).name(), node_name.c_str());
      }
      return;
    }

    // 尝试从配置中获取
    try {
      file_node[node_name] >> node_result;
    } catch (...) {
      if (required) {
        PRINT_WARNING(YELLOW "unable to parse \"%s\" node of type [%s] in the "
                             "config file!\n" RESET,
                      typeid(node_result).name(), node_name.c_str());
        all_params_found_successfully_ = false;
      } else {
        PRINT_DEBUG(
            "unable to parse \"%s\" node of type [%s] in the config file "
            "(not required)\n",
            typeid(node_result).name(), node_name.c_str());
      }
    }
  }

  /**
   * @brief 自定义解析器，用于布尔类型（
   *  0, false, False, FALSE => false 和
   *  1, true, True, TRUE => true）
   * @param file_node OpenCV 文件节点
   * @param node_name 节点名称
   * @param node_result 结果值（应该已经有默认值）
   * @param required 如果此参数是用户必须设置的，则为 true
   */
  void Parse(const cv::FileNode &file_node, const std::string &node_name,
             bool &node_result, bool required = true) {

    // 检查是否存在请求的节点
    if (!NodeFound(file_node, node_name)) {
      if (required) {
        PRINT_WARNING(YELLOW
                      "the node \"%s\" of type [%s] was not found...\n" RESET,
                      typeid(node_result).name(), node_name.c_str());
        all_params_found_successfully_ = false;
      } else {
        PRINT_DEBUG(
            "the node \"%s\" of type [%s] was not found (not required)...\n",
            typeid(node_result).name(), node_name.c_str());
      }
      return;
    }

    // 尝试从配置中获取
    try {
      if (file_node[node_name].isInt()) {
        int int_val = static_cast<int>(file_node[node_name]);
        if (int_val == 1) {
          node_result = true;
          return;
        }
        if (int_val == 0) {
          node_result = false;
          return;
        }
      }
      // 读取字符串值并进行比较
      std::string value;
      file_node[node_name] >> value;
      size_t pos = value.find_first_of('#');
      if (pos != std::string::npos) {
        value = value.substr(0, pos);
      }
      pos = value.find_first_of(' ');
      if (pos != std::string::npos) {
        value = value.substr(0, pos);
      }
      if (value == "1" || value == "true" || value == "True" ||
          value == "TRUE") {
        node_result = true;
      } else if (value == "0" || value == "false" || value == "False" ||
                 value == "FALSE") {
        node_result = false;
      } else {
        PRINT_WARNING(
            YELLOW
            "the node \"%s\" has an invalid boolean type of [%s]\n" RESET,
            node_name.c_str(), value.c_str());
        all_params_found_successfully_ = false;
      }
    } catch (...) {
      if (required) {
        PRINT_WARNING(YELLOW "unable to parse \"%s\" node of type [%s] in the "
                             "config file!\n" RESET,
                      typeid(node_result).name(), node_name.c_str());
        all_params_found_successfully_ = false;
      } else {
        PRINT_DEBUG(
            "unable to parse \"%s\" node of type [%s] in the config file "
            "(not required)\n",
            typeid(node_result).name(), node_name.c_str());
      }
    }
  }

  /**
   * @brief 自定义解析器，用于相机外参的 3x3 矩阵
   * @param file_node OpenCV 文件节点
   * @param node_name 节点名称
   * @param node_result 结果值（应该已经有默认值）
   * @param required 如果此参数是用户必须设置的，则为 true
   */
  void Parse(const cv::FileNode &file_node, const std::string &node_name,
             Eigen::Matrix3d &node_result, bool required = true) {

    // 检查是否存在请求的节点
    if (!NodeFound(file_node, node_name)) {
      if (required) {
        PRINT_WARNING(YELLOW
                      "the node \"%s\" of type [%s] was not found...\n" RESET,
                      typeid(node_result).name(), node_name.c_str());
        all_params_found_successfully_ = false;
      } else {
        PRINT_DEBUG(
            "the node \"%s\" of type [%s] was not found (not required)...\n",
            typeid(node_result).name(), node_name.c_str());
      }
      return;
    }

    // 尝试从配置中获取
    node_result = Eigen::Matrix3d::Identity();
    try {
      for (int r = 0; r < std::min<int>(file_node[node_name].size(), 3); r++) {
        for (int c = 0; c < std::min<int>(file_node[node_name][r].size(), 3);
             c++) {
          node_result(r, c) = static_cast<double>(file_node[node_name][r][c]);
        }
      }
    } catch (...) {
      if (required) {
        PRINT_WARNING(YELLOW "unable to parse \"%s\" node of type [%s] in the "
                             "config file!\n" RESET,
                      node_name.c_str(), typeid(node_result).name());
        all_params_found_successfully_ = false;
      } else {
        PRINT_DEBUG(
            "unable to parse \"%s\" node of type [%s] in the config file "
            "(not required)\n",
            node_name.c_str(), typeid(node_result).name());
      }
    }
  }

  /**
   * @brief 自定义解析器，用于相机外参的 4x4 矩阵
   * @param file_node OpenCV 文件节点
   * @param node_name 节点名称
   * @param node_result 结果值（应该已经有默认值）
   * @param required 如果此参数是用户必须设置的，则为 true
   */
  void Parse(const cv::FileNode &file_node, const std::string &node_name,
             Eigen::Matrix4d &node_result, bool required = true) {

    // // 检查是否需要交换节点名称
    std::string node_name_local = node_name;
    // if (node_name == "T_cam_imu" && !NodeFound(file_node, node_name)) {
    //   PRINT_INFO("parameter T_cam_imu not found, trying T_imu_cam instead "
    //              "(will return T_cam_imu still)!\n");
    //   node_name_local = "T_imu_cam";
    // } else if (node_name == "T_imu_cam" && !NodeFound(file_node, node_name))
    // {
    //   PRINT_INFO("parameter T_imu_cam not found, trying T_cam_imu instead "
    //              "(will return T_imu_cam still)!\n");

    //   node_name_local = "T_cam_imu";
    // }

    // 检查是否存在请求的节点
    if (!NodeFound(file_node, node_name)) {
      if (required) {
        PRINT_WARNING(YELLOW
                      "the node \"%s\" of type [%s] was not found...\n" RESET,
                      typeid(node_result).name(), node_name.c_str());
        all_params_found_successfully_ = false;
      } else {
        PRINT_DEBUG(
            "the node \"%s\" of type [%s] was not found (not required)...\n",
            typeid(node_result).name(), node_name.c_str());
      }
      return;
    }
    // 尝试从配置中获取
    node_result = Eigen::Matrix4d::Identity();
    try {
      for (int r = 0; r < std::min<int>(file_node[node_name_local].size(), 4);
           r++) {
        for (int c = 0;
             c < std::min<int>(file_node[node_name_local][r].size(), 4); c++) {
          node_result(r, c) =
              static_cast<double>(file_node[node_name_local][r][c]);
        }
      }
    } catch (...) {
      if (required) {
        PRINT_WARNING(YELLOW "unable to parse \"%s\" node of type [%s] in the "
                             "config file!\n" RESET,
                      node_name.c_str(), typeid(node_result).name());
        all_params_found_successfully_ = false;
      } else {
        PRINT_DEBUG(
            "unable to parse \"%s\" node of type [%s] in the config file "
            "(not required)\n",
            node_name.c_str(), typeid(node_result).name());
      }
    }

    // // 如果交换了变换矩阵，进行反转
    // if (node_name_local != node_name) {
    //   Eigen::Matrix4d tmp(node_result);
    //   node_result = Inv_se3(tmp);
    // }
  }

  // /**
  //  * @brief 从带有层级的外部参数文件解析参数。
  //  *
  //  * 这将首先加载请求的外部文件。
  //  * 然后它将尝试在第一个级别（例如 imu0、cam0、cam1）下查找请求的节点。
  //  *
  //  * @tparam T 要查找的参数类型。
  //  * @param external_node_name 外部节点名称，用于获取相对路径
  //  * @param sensor_name 第一个级别节点名称
  //  * @param node_name 节点名称
  //  * @param node_result 结果值（应该已经有默认值）
  //  * @param required 如果此参数是用户必须设置的，则为 true
  //  */
  // template <class T>
  // void ParseExternalYaml(const std::string &external_node_name,
  //                          const std::string &sensor_name,
  //                          const std::string &node_name, T &node_result,
  //                          bool required = true) {
  //   // 如果配置未打开，直接返回
  //   if (config_ == nullptr)
  //     return;

  //   // 创建外部 YAML 文件的路径
  //   std::string path;
  //   if (!NodeFound(config_->root(), external_node_name)) {
  //     PRINT_ERROR(RED "未找到外部节点 %s！\n" RESET,
  //                 external_node_name.c_str());
  //     std::exit(EXIT_FAILURE);
  //   }
  //   (*config_)[external_node_name] >> path;
  //   std::string relative_folder = GetConfigFolder();

  //   // 尝试从文件中加载外部配置
  //   std::shared_ptr<cv::FileStorage> config_external =
  //       std::make_shared<cv::FileStorage>(relative_folder + path,
  //                                         cv::FileStorage::READ);
  //   if (!config_external->isOpened()) {
  //     PRINT_ERROR(RED "无法打开配置文件！\n%s\n" RESET,
  //                 (relative_folder + path).c_str());
  //     std::exit(EXIT_FAILURE);
  //   }

  //   // 检查是否存在请求的传感器节点
  //   if (!NodeFound(config_external->root(), sensor_name)) {
  //     PRINT_WARNING(YELLOW "未找到类型为 [%s] 的传感器 %s...\n" RESET,
  //                   typeid(node_result).name(), sensor_name.c_str());
  //     all_params_found_successfully_ = false;
  //     return;
  //   }

  //   // 尝试从外部配置中获取
  //   try {
  //     Parse((*config_external)[sensor_name], node_name, node_result,
  //     required);
  //   } catch (...) {
  //     PRINT_WARNING(YELLOW "无法解析外部配置文件 %s 中传感器 %s 的节点 "
  //                          "%s，类型为 [%s]！\n" RESET,
  //                   external_node_name.c_str(), sensor_name.c_str(),
  //                   node_name.c_str(), typeid(node_result).name());
  //     all_params_found_successfully_ = false;
  //   }
  // }
};

#endif /* COMMON_UTILS_OPENCV_YAML_PARSER_H */
