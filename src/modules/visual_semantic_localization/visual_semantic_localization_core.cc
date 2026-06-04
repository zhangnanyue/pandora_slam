#include "visual_semantic_localization_core.h"

#include "common_utils/print.h"
#include "common_utils/tic_toc.h"

#include <filesystem>
#include <pcl/io/pcd_io.h>


VisualSemanticLocalizationCore::VisualSemanticLocalizationCore() {
    r_l_to_w_gt_ = Eigen::Matrix3d::Identity();
    t_l_in_w_gt_ = Eigen::Vector3d::Zero();
    r_i_to_w_gt_ = Eigen::Matrix3d::Identity();
    t_i_in_w_gt_ = Eigen::Vector3d::Zero();
}

bool VisualSemanticLocalizationCore::Init(const VisualSemanticLocalizationOptions& options) {
    PRINT_INFO("====== Visual Semantic Localization Core Initialization Started ======\n");

    // Store configuration
    options_ = options;

    // load semantic pointcloud map
    if (!LoadSemanticPointcloudMapData(options_.data.semantic_pointcloud_map_folder_path)){
        PRINT_ERROR("Faild to load semantic pointcloud map");
        return false;
    }

    // Initialize ESKF
    PRINT_INFO("====== Initializing ESKF ======\n");
    ESKFConfig eskf_config;
    eskf_config.freq = options_.imu.freq;
    eskf_config.gravity = options_.imu.gravity;

    // 映射传感器噪声标准差
    eskf_config.sensor_noise_std.acc_noise_std_x = options_.imu.sensor_noise_std.acc_noise_std_x;
    eskf_config.sensor_noise_std.acc_noise_std_y = options_.imu.sensor_noise_std.acc_noise_std_y;
    eskf_config.sensor_noise_std.acc_noise_std_z = options_.imu.sensor_noise_std.acc_noise_std_z;
    eskf_config.sensor_noise_std.gyro_noise_std_x = options_.imu.sensor_noise_std.gyro_noise_std_x;
    eskf_config.sensor_noise_std.gyro_noise_std_y = options_.imu.sensor_noise_std.gyro_noise_std_y;
    eskf_config.sensor_noise_std.gyro_noise_std_z = options_.imu.sensor_noise_std.gyro_noise_std_z;

    // 映射偏置随机游走标准差
    eskf_config.bias_random_walk_std.acc_bias_std_x =
        options_.imu.bias_random_walk_std.acc_bias_std_x;
    eskf_config.bias_random_walk_std.acc_bias_std_y =
        options_.imu.bias_random_walk_std.acc_bias_std_y;
    eskf_config.bias_random_walk_std.acc_bias_std_z =
        options_.imu.bias_random_walk_std.acc_bias_std_z;
    eskf_config.bias_random_walk_std.gyro_bias_std_x =
        options_.imu.bias_random_walk_std.gyro_bias_std_x;
    eskf_config.bias_random_walk_std.gyro_bias_std_y =
        options_.imu.bias_random_walk_std.gyro_bias_std_y;
    eskf_config.bias_random_walk_std.gyro_bias_std_z =
        options_.imu.bias_random_walk_std.gyro_bias_std_z;

    // 映射初始标准差
    eskf_config.initial_std.pos_init_std = options_.imu.initial_std.pos_init_std;
    eskf_config.initial_std.vel_init_std = options_.imu.initial_std.vel_init_std;
    eskf_config.initial_std.rot_init_std = options_.imu.initial_std.rot_init_std;
    eskf_config.initial_std.gyro_bias_init_std = options_.imu.initial_std.gyro_bias_init_std;
    eskf_config.initial_std.acc_bias_init_std = options_.imu.initial_std.acc_bias_init_std;
    eskf_config.initial_std.gravity_init_std = options_.imu.initial_std.gravity_init_std;

    eskf_ = ESKF::Create(eskf_config);
    if (!eskf_) {
        PRINT_ERROR("Failed to create ESKF\n");
        return false;
    }
    PRINT_INFO("====== ESKF created successfully ======\n");
    initialized_ = true;
    PRINT_INFO("====== Visual Semantic Localization Core Initialization Completed ======\n");
    return true;
}

void VisualSemanticLocalizationCore::Process(const DataGroup& data_group) {
    double timestamp = data_group.timestamp;
    PRINT_INFO("====== Visual Semantic Localization Core Processing %.6f frame ======\n",
               timestamp);

    // 真值计算
    if (data_group.ground_truth) {
        r_l_to_w_gt_ = data_group.ground_truth->r;
        t_l_in_w_gt_ = data_group.ground_truth->t;
        r_i_to_w_gt_ = r_l_to_w_gt_ * options_.extrinsics.r_i_to_l_;
        t_i_in_w_gt_ = r_l_to_w_gt_ * options_.extrinsics.t_i_in_l_ + t_l_in_w_gt_;
        PRINT_INFO("ground truth pos in imu frame: [%.6f, %.6f, %.6f]\n",
                   t_i_in_w_gt_(0), t_i_in_w_gt_(1), t_i_in_w_gt_(2));
    }

    // eskf的状态初始化
    // 这里由于提供了真值，没有使用IMU的静态初始化，p和r直接和真值对齐，bg和ba置为0
    if (data_group.is_init_frame && !eskf_initialized_) {
        eskf_->SetInitConditions(timestamp, Vec3d::Zero(), Vec3d::Zero(),
                                 r_i_to_w_gt_, t_i_in_w_gt_);
        eskf_initialized_ = true;

        last_frame_timestamp_ = timestamp;

        PRINT_INFO("ESKF initialized successfully\n");
        return;
    }

    // ESKF Predict
    TicToc predict_timer;
    predict_timer.tic();

    // 预测
    if (data_group.imu_data && !data_group.imu_data->empty()) {
        if (!eskf_->Predict(*data_group.imu_data)){
            PRINT_ERROR("ESKF Predict failed\n");
            return;
        }
    }

    double predict_time = predict_timer.toc();
    PRINT_INFO("%.2f ms for eskf predict \n", predict_time); // ESKF prediction time cost




    // if (!initialized_) {
    //     PRINT_ERROR("Visual Semantic Localization Core not initialized!\n");
    //     return;
    // }

    // // 检查IMU数据
    // if (data_group.imu_data && !data_group.imu_data->empty()) {
    //     PRINT_INFO("Processing IMU data with %zu data points\n", data_group.imu_data->size());
    //     // TODO: 处理IMU数据
    // }

    // // 检查语义轮廓数据
    // if (data_group.semantic_contours && !data_group.semantic_contours->category_contour.empty())
    // {
    //     PRINT_INFO("Processing semantic contours with %zu categories\n",
    //               data_group.semantic_contours->category_contour.size());
    //     // TODO: 处理语义轮廓数据
    // }

    // // 检查原始图像
    // if (data_group.raw_image && !data_group.raw_image->empty()) {
    //     PRINT_INFO("Processing raw image with size %dx%d\n",
    //               data_group.raw_image->cols, data_group.raw_image->rows);
    //     // TODO: 处理原始图像
    // }

    // // Print current state
    // if (eskf_) {
    //     eskf_->PrintStates("After processing data group");
    // }
}

bool VisualSemanticLocalizationCore::LoadSemanticPointcloudMapData(const std::string& dir_path) {
    PRINT_INFO("Loading semantic map data...\n");

    if (dir_path.empty() || !std::filesystem::exists(dir_path)) {
        PRINT_ERROR("Semantic map directory does not exist: %s\n", dir_path.c_str());
        return false;
    }

    for (const auto &entry :
         std::filesystem::directory_iterator(dir_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
            PointcloudXYZIPtr cloud(new pcl::PointCloud<pcl::PointXYZI>());

            if (pcl::io::loadPCDFile<pcl::PointXYZI>(entry.path().string(), *cloud) == -1) {
                PRINT_ERROR("could not read pcd file: %s\n", entry.path().c_str());
                return false;
            }

            semantic_pointcloud_map_[entry.path().filename().string()] = cloud;
        }
    }
    PRINT_INFO("Loaded %zu semantic pointcloud map data\n", semantic_pointcloud_map_.size());
    return true;
}