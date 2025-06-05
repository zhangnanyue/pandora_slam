cmake_minimum_required(VERSION 3.10)

project(eskf VERSION 1.0.0 LANGUAGES CXX)

# 设置源文件
set(SOURCES
    eskf.cc
)

# 设置头文件
set(PUBLIC_HEADERS
    eskf.h
)

# 创建库目标
add_library(eskf SHARED ${SOURCES})

# 设置包含目录
target_include_directories(eskf PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include/eskf>
)

# 链接依赖库
target_link_libraries(eskf PUBLIC
    camera_model
    common_utils
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    Eigen3::Eigen
    yaml-cpp
    )

# 设置目标属性
set_target_properties(eskf PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# 安装目标
install(TARGETS eskf
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/eskf
)