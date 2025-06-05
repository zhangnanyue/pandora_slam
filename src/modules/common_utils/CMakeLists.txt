cmake_minimum_required(VERSION 3.10)

project(common_utils VERSION 1.0.0 LANGUAGES CXX)

# 设置源文件
set(SOURCES
    print.cc
)

# 设置头文件
set(PUBLIC_HEADERS
    colors.h
    eigen_types.h
    math_utils.h
    opencv_yaml_parse.h
    print.h
    so3_utils.h
    utils.h
    common_data_types.h
    so3_math.h
    pcl_utils.h
    tic_toc.h
)

# 创建库目标
add_library(common_utils SHARED ${SOURCES})

# 设置包含目录
target_include_directories(common_utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include/common_utils>
)

# 链接依赖库
target_link_libraries(common_utils PUBLIC
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    Eigen3::Eigen
    yaml-cpp
)

# 设置目标属性
set_target_properties(common_utils PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# 安装目标
install(TARGETS common_utils
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/common_utils
)
