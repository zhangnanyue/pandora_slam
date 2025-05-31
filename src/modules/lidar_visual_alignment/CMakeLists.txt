cmake_minimum_required(VERSION 3.10)
project(lidar_visual_alignment VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.8 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


add_library(lidar_visual_alignment SHARED
    Calibrating.cc
    lidar_visual_alignment.cc
)

# target_include_directories(lidar_visual_alignment PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_include_directories(lidar_visual_alignment PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include/lidar_visual_alignment>
)

target_link_libraries(lidar_visual_alignment PUBLIC
    camera_model
    common_utils
    ${CERES_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Eigen3::Eigen
    )


set(PUBLIC_HEADERS
    Calibrating.h
    lidar_visual_alignment.h
    lidar_visual_alignment_options.h
)


set_target_properties(lidar_visual_alignment PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
)


# set_target_properties(lidar_visual_alignment PROPERTIES
#     PUBLIC_HEADER "${PUBLIC_HEADERS}"
# )

# 安装共享库到指定目录
# install(TARGETS lidar_visual_alignment DESTINATION lib)

# 安装目标和头文件
install(TARGETS lidar_visual_alignment
    # EXPORT lidar_visual_alignmentTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/lidar_visual_alignment
)

# 安装config
install(FILES
    config/lidar_visual_alignment.yaml
    DESTINATION config/lidar_visual_alignment
)

# install(EXPORT lidar_visual_alignmentTargets
#     FILE lidar_visual_alignmentTargets.cmake
#     NAMESPACE pandora_slam::
#     DESTINATION lib/cmake/lidar_visual_alignment
# )

# install(FILES
#     Calibrating.h
#     lidar_visual_alignment.h
#     lidar_visual_alignment_options.h
#     DESTINATION include/lidar_visual_alignment
# )
