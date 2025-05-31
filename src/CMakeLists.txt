cmake_minimum_required(VERSION 3.10)
project(pandora_slam)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(modules/camera_model)
add_subdirectory(modules/common_utils)
add_subdirectory(modules/lidar_visual_alignment)
add_subdirectory(modules/visual_semantic_localization)
add_subdirectory(modules/data_loader)
add_subdirectory(examples/common_uitls_example)

# # 指定安装路径
# set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)

# # 安装时包含公共头文件
# install(DIRECTORY modules/common_utils DESTINATION include/common_utils)
