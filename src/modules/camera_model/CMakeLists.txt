cmake_minimum_required(VERSION 3.10)
project(camera_model VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_library(camera_model SHARED
    pinhole_camera.cc
)

target_include_directories(camera_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include/camera_model>
)

# include_directories(${EIGEN3_INCLUDE_DIR})
target_link_libraries(camera_model
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

set(PUBLIC_HEADERS
    camera_model.h
    pinhole_camera.h
)

set_target_properties(camera_model PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
)

# 安装目标和头文件
install(TARGETS camera_model
    # EXPORT camera_modelTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/camera_model
)


# # 安装共享库到指定目录
# install(TARGETS camera_model DESTINATION lib)
# install(FILES
#     camera_model.h
#     pinhole_camera.h
#     DESTINATION include/camera_model
# )
