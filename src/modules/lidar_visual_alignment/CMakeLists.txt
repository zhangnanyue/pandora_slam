cmake_minimum_required(VERSION 3.10)

project(lidar_visual_alignment VERSION 1.0.0 LANGUAGES CXX)

# 设置源文件
set(SOURCES
    lidar_visual_alignment.cc
    Calibrating.cc
)

# 设置头文件
set(PUBLIC_HEADERS
    lidar_visual_alignment.h
    lidar_visual_alignment_options.h
    Calibrating.h
)

# 创建库目标
add_library(lidar_visual_alignment SHARED ${SOURCES})

# 设置包含目录
target_include_directories(lidar_visual_alignment PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include/lidar_visual_alignment>
)

# 链接依赖库
target_link_libraries(lidar_visual_alignment PUBLIC
    camera_model
    common_utils
    data_loader
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    Eigen3::Eigen
    yaml-cpp
)

# 设置目标属性
set_target_properties(lidar_visual_alignment PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# 安装目标
install(TARGETS lidar_visual_alignment
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/lidar_visual_alignment
)

# 安装配置文件
install(FILES
    config/lidar_visual_alignment.yaml
    DESTINATION config/lidar_visual_alignment
)
