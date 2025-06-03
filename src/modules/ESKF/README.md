# ESKF 模块使用说明

## 初始化 ESKF
ESKF 模块固定使用 18 维状态估计：[r, p, v, bg, ba, g]
- r: 姿态 (3维)
- p: 位置 (3维) 
- v: 速度 (3维)
- bg: 陀螺仪偏置 (3维)
- ba: 加速度计偏置 (3维)
- g: 重力向量 (3维)

### 1. 基本配置
```cpp
ESKFConfig config;
config.imu_dt = 0.01;     // IMU采样时间，默认0.01s (100Hz)
```

### 2. 传感器噪声配置
```cpp
// 加速度计噪声方差
config.sensor_noise.acc_var_x = 9.8e-3;  // x轴
config.sensor_noise.acc_var_y = 9.8e-3;  // y轴
config.sensor_noise.acc_var_z = 9.8e-3;  // z轴

// 陀螺仪噪声方差
config.sensor_noise.gyr_var_x = 1.9e-4;  // x轴
config.sensor_noise.gyr_var_y = 1.9e-4;  // y轴
config.sensor_noise.gyr_var_z = 4.8481e-5;  // z轴
```

### 3. 偏置随机游走配置
```cpp
// 加速度计偏置随机游走方差
config.bias_random_walk.acc_var_x = 6.6667e-7;  // x轴
config.bias_random_walk.acc_var_y = 6.6667e-7;  // y轴
config.bias_random_walk.acc_var_z = 6.6667e-7;  // z轴

// 陀螺仪偏置随机游走方差
config.bias_random_walk.gyr_var_x = 1.0167e-7;  // x轴
config.bias_random_walk.gyr_var_y = 1.0167e-7;  // y轴
config.bias_random_walk.gyr_var_z = 1.0167e-8;  // z轴
```

### 4. 初始协方差配置
```cpp
// 状态初始协方差
config.cov.pos = 1e-2;      // 位置初始协方差
config.cov.vel = 1e-3;      // 速度初始协方差
config.cov.rot = 1e-2;      // 姿态初始协方差
config.cov.bias_gyr = 1e-4; // 陀螺仪偏置初始协方差
config.cov.bias_acc = 1e-4; // 加速度计偏置初始协方差
config.cov.gravity = 1e-3;  // 重力向量初始协方差
```

### 5. 创建 ESKF 实例
```cpp
// 使用自定义配置
auto eskf = ESKF::Create(config);

// 或使用默认配置
auto eskf = ESKF::Create(ESKFConfig{});
```

### 6. 获取状态
```cpp
Mat3d rotation = eskf->r();        // 姿态旋转矩阵
Vec3d position = eskf->p();        // 位置
Vec3d velocity = eskf->v();        // 速度
Vec3d gyro_bias = eskf->bg();      // 陀螺仪偏置
Vec3d accel_bias = eskf->ba();     // 加速度计偏置
Vec3d gravity = eskf->g();         // 重力向量
```

## 注意事项
- 固定使用 18 维状态，无需选择状态维度
- 根据实际 IMU 性能调整噪声参数
- 确保 IMU 数据单位正确：角速度(rad/s)，加速度(m/s^2)
- 初始协方差参数已根据实际应用场景优化 