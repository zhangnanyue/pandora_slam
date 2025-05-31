#include "visual_semantic_localization_app.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <sstream>
#include <unordered_map>

#include "common_utils/opencv_yaml_parse.h"
#include "common_utils/print.h"
#include "visual_semantic_localization.h"
#include "visual_semantic_localization_options.h"

bool VisualSemanticLocalizationApp::Initialize(const std::string &config_yaml_file) {
    PRINT_INFO("====== Visual Semantic Localization App Initialization Started ======\n");

    // Initialize data loader
    data_loader_ = std::make_shared<DataLoaderApp>();
    if (!data_loader_) {
        PRINT_ERROR("Failed to initialize data loader\n");
        return false;
    }
    PRINT_INFO("Data loader initialized successfully\n");

    // Load configuration parameters
    options_.LoadSensorParamsAndPrint(config_yaml_file);
    options_.LoadDataAndPrint(config_yaml_file);

    // Load all data
    if (!LoadAllData()) {
        PRINT_ERROR("Failed to load data\n");
        return false;
    }

    PRINT_INFO("====== Visual Semantic Localization App Initialization Completed ======\n");
    return true;
}

void VisualSemanticLocalizationApp::Run() {
    PRINT_INFO("====== Visual Semantic Localization App Running ======\n");

    // Print loaded data statistics
    PRINT_INFO("Data Summary:\n");
    PRINT_INFO("  Raw Images: %zu\n", raw_image_map_.size());
    PRINT_INFO("  Semantic Images: %zu\n", semantic_mask_image_map_.size());
    PRINT_INFO("  IMU Data Points: %zu\n", imu_data_map_.size());
    PRINT_INFO("  Ground Truth Points: %zu\n", ground_truth_map_.size());
    PRINT_INFO("  Semantic Contours: %zu\n", semantic_contours_map_.size());

    // Process data groups
    // 这里的所有数据都以semantic_contours_map_的时间戳为主,以semantic_contours_map_的时间戳为基准,
    // 只处理semantic_contours_map_中存在的时间戳
    for (const auto &[timestamp, contours_data] : semantic_contours_map_) {
        // Pack data group
        DataGroup data_group;
        PackDataGroup(timestamp, data_group);
        // if (PackDataGroup(timestamp, data_group)) {
        //     // Add complete data group to queue
        //     {
        //         std::lock_guard<std::mutex> lock(queue_mutex_);
        //         data_group_queue_.push(data_group);
        //     }
        //     queue_cv_.notify_one();
        // }
    }

    // // Process all data groups
    // while (!data_group_queue_.empty()) {
    //     DataGroup data_group;
    //     {
    //         std::unique_lock<std::mutex> lock(queue_mutex_);
    //         queue_cv_.wait(lock, [this] { return !data_group_queue_.empty(); });
    //         data_group = data_group_queue_.front();
    //         data_group_queue_.pop();
    //     }
    //     // ProcessDataGroup(data_group);
    // }

    PRINT_INFO("====== Visual Semantic Localization App Running Completed ======\n");
    return;
}

bool VisualSemanticLocalizationApp::LoadAllData() {
    PRINT_INFO("====== Loading All Data ======\n");

    // Load raw images
    if (!LoadRawImageData()) {
        PRINT_ERROR("Failed to load raw image data\n");
        return false;
    }

    // Load semantic mask images
    if (!LoadSemanticMaskImageData()) {
        PRINT_ERROR("Failed to load semantic mask image data\n");
        return false;
    }

    // Load IMU data
    if (!LoadIMUData()) {
        PRINT_ERROR("Failed to load IMU data\n");
        return false;
    }

    // Load ground truth data
    if (!LoadGroundTruthData()) {
        PRINT_ERROR("Failed to load ground truth data\n");
        return false;
    }

    // Load semantic contours data
    if (!LoadSemanticContoursData()) {
        PRINT_ERROR("Failed to load semantic contours data\n");
        return false;
    }

    PRINT_INFO("====== All Data Loaded Successfully ======\n");
    return true;
}

bool VisualSemanticLocalizationApp::LoadRawImageData() {
    PRINT_INFO("Loading raw image data...\n");

    // Load raw images from directory with specific file pattern
    if (!data_loader_->LoadImagesFromDirectory(options_.raw_image_data_folder_path, raw_image_map_,
                                               {".jpg", ".png", ".bmp"},
                                               "^(\\d+\\.\\d+)_.*\\.jpg$")) {
        PRINT_ERROR("Failed to load raw images from directory: %s\n",
                    options_.raw_image_data_folder_path.c_str());
        return false;
    }
    // PRINT_INFO("Successfully loaded %zu raw images\n", raw_image_map_.size());

    return true;
}

bool VisualSemanticLocalizationApp::LoadSemanticMaskImageData() {
    PRINT_INFO("Loading semantic mask image data...\n");

    // Load semantic mask images from directory with specific file pattern
    if (!data_loader_->LoadImagesFromDirectory(options_.semantic_data_folder_path,
                                               semantic_mask_image_map_, {".jpg", ".png", ".bmp"},
                                               "^(\\d+\\.\\d+)_.*\\.jpg$")) {
        PRINT_ERROR("Failed to load semantic images from directory: %s\n",
                    options_.semantic_data_folder_path.c_str());
        return false;
    }
    // PRINT_INFO("Successfully loaded %zu semantic images\n", semantic_mask_image_map_.size());

    return true;
}

bool VisualSemanticLocalizationApp::LoadIMUData() {
    PRINT_INFO("Loading IMU data...\n");

    // Load and parse IMU data from text file
    if (!data_loader_->LoadTxtFile(options_.raw_imu_data_file_path, [this](
                                                                        const std::string &line) {
            // Skip comment lines and empty lines
            if (line.empty() || line[0] == '#') return true;

            IMUData data;
            int index;  // Used to store line number but not used
            // Parse IMU data line, format: index timestamp ax ay az gx gy gz
            if (sscanf(line.c_str(), "%d %lf %lf %lf %lf %lf %lf %lf", &index, &data.timestamp,
                       &data.accel.x(), &data.accel.y(), &data.accel.z(), &data.gyro.x(),
                       &data.gyro.y(), &data.gyro.z()) == 8) {
                imu_data_map_[data.timestamp] = data;  // Store data using timestamp as key
                // PRINT_DEBUG("IMU data: timestamp = %f, accel = (%f, %f, %f), gyro = (%f, %f, %f)\n",
                //             data.timestamp, data.accel.x(), data.accel.y(), data.accel.z(),
                //             data.gyro.x(), data.gyro.y(), data.gyro.z());
                return true;
            }
            return false;
        })) {
        PRINT_ERROR("Failed to load IMU data from: %s\n", options_.raw_imu_data_file_path.c_str());
        return false;
    }
    PRINT_INFO("Successfully loaded %zu IMU data points\n", imu_data_map_.size());

    return true;
}

bool VisualSemanticLocalizationApp::LoadGroundTruthData() {
    PRINT_INFO("Loading ground truth data...\n");

    // Load and parse ground truth data from text file
    if (!data_loader_->LoadTxtFile(
            options_.ground_truth_data_file_path, [this](const std::string &line) {
                // Skip comment lines and empty lines
                if (line.empty() || line[0] == '#') return true;

                PoseData pose_data;
                std::string record_name;
                double x, y, z, q_x, q_y, q_z, q_w;
                std::istringstream iss(line);
                // Parse format: record_name timestamp x y z qx qy qz qw
                if (!(iss >> record_name >> pose_data.timestamp >> x >> y >> z >> q_x >> q_y >>
                      q_z >> q_w)) {
                    PRINT_ERROR("Parsing error in ground truth file, skipping line: %s\n",
                                line.c_str());
                    return true;  // Skip error line and continue
                }
                // Construct pose matrix
                pose_data.t = Eigen::Vector3d(x, y, z);
                pose_data.q = Eigen::Quaterniond(q_w, q_x, q_y, q_z);
                pose_data.r = pose_data.q.toRotationMatrix();

                // Construct 4x4 transformation matrix
                pose_data.T.setIdentity();
                pose_data.T.block<3, 3>(0, 0) = pose_data.r;
                pose_data.T.block<3, 1>(0, 3) = pose_data.t;

                ground_truth_map_[pose_data.timestamp] = pose_data;
                // PRINT_DEBUG("Ground truth data: timestamp = %f, position = (%f, %f, %f)\n",
                //             pose_data.timestamp, x, y, z);
                return true;
            })) {
        PRINT_ERROR("Failed to load ground truth data from: %s\n",
                    options_.ground_truth_data_file_path.c_str());
        return false;
    }
    PRINT_INFO("Successfully loaded %zu ground truth data points\n", ground_truth_map_.size());

    return true;
}

bool VisualSemanticLocalizationApp::LoadSemanticContoursData() {
    PRINT_INFO("Loading semantic contours data...\n");

    // Load and parse semantic contours data from text file
    if (!data_loader_->LoadTxtFile(
            options_.semantic_contours_file_path, [this](const std::string &line) {
                // Skip comment lines and empty lines
                if (line.empty() || line[0] == '#') return true;

                std::istringstream iss(line);
                SemanticContoursData semantic_contours;
                std::string category;
                int x, y;

                // Read fixed fields: record_name camera_type lidar_timestamp camera_timestamp
                // category
                if (!(iss >> semantic_contours.record_name >> semantic_contours.camera_type >>
                      semantic_contours.lidar_timestamp >> semantic_contours.camera_timestamp >>
                      category)) {
                    PRINT_ERROR("Failed to read the fixed fields from line: %s\n", line.c_str());
                    return false;
                }

                // Read contour points
                std::vector<cv::Point> contour;
                while (iss >> x >> y) {
                    contour.push_back(cv::Point(x, y));
                }

                if (contour.empty()) {
                    PRINT_ERROR("No contour found for line: %s\n", line.c_str());
                    return false;
                }

                // Store data
                if (semantic_contours_map_.find(semantic_contours.lidar_timestamp) !=
                    semantic_contours_map_.end()) {
                    // If timestamp exists, add new category contour
                    semantic_contours_map_[semantic_contours.lidar_timestamp]
                        .category_contour[category] = contour;
                } else {
                    // Create new semantic contour entry
                    semantic_contours.category_contour[category] = contour;
                    semantic_contours.is_valid = true;
                    semantic_contours_map_[semantic_contours.lidar_timestamp] = semantic_contours;
                }

                // PRINT_DEBUG(
                //     "Loaded semantic contour: timestamp = %f, category = %s, points = %zu\n",
                //     semantic_contours.lidar_timestamp, category.c_str(), contour.size());
                return true;
            })) {
        PRINT_ERROR("Failed to load semantic contours from: %s\n",
                    options_.semantic_contours_file_path.c_str());
        return false;
    }
    PRINT_INFO("Successfully loaded %zu semantic contours data points\n",
               semantic_contours_map_.size());

    return true;
}

bool VisualSemanticLocalizationApp::PackDataGroup(const double &timestamp, DataGroup &data_group) {
    data_group.timestamp = timestamp;
    std::vector<std::string> available_data;

    // 检查并获取原始图像
    auto it_raw_image = raw_image_map_.find(timestamp);
    if (it_raw_image != raw_image_map_.end()) {
        data_group.raw_image = std::make_shared<cv::Mat>(it_raw_image->second->GetImage());
        if (data_group.raw_image) available_data.push_back("raw_image");
    }

    // 检查并获取语义掩码图像
    auto it_semantic_mask_image = semantic_mask_image_map_.find(timestamp);
    if (it_semantic_mask_image != semantic_mask_image_map_.end()) {
        data_group.semantic_mask_image = std::make_shared<cv::Mat>(it_semantic_mask_image->second->GetImage());
        if (data_group.semantic_mask_image) available_data.push_back("semantic_mask_image");
    }

    // 检查并获取地面真值数据
    auto it_ground_truth = ground_truth_map_.find(timestamp);
    if (it_ground_truth != ground_truth_map_.end()) {
        data_group.ground_truth = std::make_shared<PoseData>(it_ground_truth->second);
        if (data_group.ground_truth) available_data.push_back("ground_truth");
    }

    // 检查并获取语义轮廓数据
    auto it_semantic_contours = semantic_contours_map_.find(timestamp);
    if (it_semantic_contours != semantic_contours_map_.end()) {
        data_group.contours = std::make_shared<SemanticContoursData>(it_semantic_contours->second);
        if (data_group.contours) available_data.push_back("semantic_contours");
    }

    // 检查数据完整性
    data_group.is_complete = (available_data.size() == 4);
    
    // 使用标准库构建数据列表字符串
    std::string data_list;
    for (size_t i = 0; i < available_data.size(); ++i) {
        if (i > 0) data_list += ", ";
        data_list += available_data[i];
    }

    // 根据数据完整性选择打印级别
    if (data_group.is_complete) {
        // PRINT_INFO("Data group at timestamp %.3f is complete, available data types: %s\n", timestamp, data_list.c_str());
        PRINT_INFO("Data group at timestamp %.3f is complete\n", timestamp);
    } else {
        PRINT_WARNING("Data group at timestamp %.3f is incomplete, available data types: %s\n", timestamp, data_list.c_str());
    }

    return data_group.is_complete;
}

// void VisualSemanticLocalizationApp::ProcessDataGroup(const DataGroup &data_group) {
//     if (!data_group.IsValid()) {
//         PRINT_ERROR("Invalid data group at timestamp: %f\n", data_group.timestamp);
//         return;
//     }

//     // TODO: Implement the core algorithm logic here
//     // For example:
//     // 1. Process semantic contours
//     // 2. Match with raw image features
//     // 3. Use IMU data for motion prediction
//     // 4. Compare with ground truth
//     // 5. Update localization result

//     PRINT_INFO("Processing data group at timestamp: %f\n", data_group.timestamp);
// }

// void VisualSemanticLocalizationApp::CleanupOldData() {
//     const double max_queue_time = 5.0;  // Maximum queue time in seconds
//     double current_time = GetCurrentTime();

//     std::lock_guard<std::mutex> lock(queue_mutex_);
//     while (!data_group_queue_.empty()) {
//         if (current_time - data_group_queue_.front().timestamp > max_queue_time) {
//             data_group_queue_.pop();
//         } else {
//             break;
//         }
//     }
// }
