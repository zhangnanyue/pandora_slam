cmake_minimum_required(VERSION 3.10)

# 设置CMake策略
cmake_policy(SET CMP0074 NEW)  # 使用新的find_package行为
cmake_policy(SET CMP0077 NEW)  # 使用新的option()行为

project(pandora_slam)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译选项
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_PARALLEL_LEVEL 8)  # 启用并行编译
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # 导出编译命令数据库

# 设置编译优化选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# 设置PCL组件
set(PCL_COMPONENTS
    common
    io
    kdtree
    search
    sample_consensus
    filters
    features
    segmentation
    visualization
    surface
    registration
    keypoints
    tracking
    recognition
    stereo
    apps
    outofcore
    people
)

# 查找公共依赖
find_package(PCL 1.8 REQUIRED COMPONENTS ${PCL_COMPONENTS} QUIET)
find_package(Ceres REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Eigen3 REQUIRED QUIET)
find_package(yaml-cpp REQUIRED QUIET)

# 设置公共包含目录
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIR}
)

# 添加子目录
add_subdirectory(modules/camera_model)
add_subdirectory(modules/common_utils)
# add_subdirectory(modules/lidar_visual_alignment)
add_subdirectory(modules/visual_semantic_localization)
add_subdirectory(modules/data_loader)
add_subdirectory(modules/ESKF)
add_subdirectory(examples/common_uitls_example)

# 设置安装路径
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)

# # 安装时包含公共头文件
# install(DIRECTORY modules/common_utils DESTINATION include/common_utils)
