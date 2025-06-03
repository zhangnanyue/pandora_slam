#include "visual_semantic_localization_core.h"
#include "common_utils/print.h"

bool VisualSemanticLocalizationCore::Initialize(const VisualSemanticLocalizationOptions& options) {
    PRINT_INFO("====== Visual Semantic Localization Core Initialization Started ======\n");
    
    // Store configuration
    options_ = options;
    
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
    eskf_config.bias_random_walk_std.acc_bias_std_x = options_.imu.bias_random_walk_std.acc_bias_std_x;
    eskf_config.bias_random_walk_std.acc_bias_std_y = options_.imu.bias_random_walk_std.acc_bias_std_y;
    eskf_config.bias_random_walk_std.acc_bias_std_z = options_.imu.bias_random_walk_std.acc_bias_std_z;
    eskf_config.bias_random_walk_std.gyro_bias_std_x = options_.imu.bias_random_walk_std.gyro_bias_std_x;
    eskf_config.bias_random_walk_std.gyro_bias_std_y = options_.imu.bias_random_walk_std.gyro_bias_std_y;
    eskf_config.bias_random_walk_std.gyro_bias_std_z = options_.imu.bias_random_walk_std.gyro_bias_std_z;
    
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

void VisualSemanticLocalizationCore::Run(const DataGroup &data_group) {
    if (!initialized_) {
        PRINT_ERROR("VisualSemanticLocalizationCore not initialized!\n");
        return;
    }
    
    // TODO: Implement the core functionality of the visual semantic localization module
    PRINT_INFO("VisualSemanticLocalizationCore::Run\n");
    return;
}