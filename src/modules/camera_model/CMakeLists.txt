cmake_minimum_required(VERSION 3.10)

project(camera_model VERSION 1.0.0 LANGUAGES CXX)

# 设置源文件
set(SOURCES
    pinhole_camera.cc
)

# 设置头文件
set(PUBLIC_HEADERS
    camera_model.h
    pinhole_camera.h
)

# 创建库目标
add_library(camera_model SHARED ${SOURCES})

# 设置包含目录
target_include_directories(camera_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include/camera_model>
)

# 链接依赖库
target_link_libraries(camera_model PUBLIC
    common_utils
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    Eigen3::Eigen
    yaml-cpp
)

# 设置目标属性
set_target_properties(camera_model PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# 安装目标
install(TARGETS camera_model
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/camera_model
)
