cmake_minimum_required(VERSION 3.10)
project(data_loader VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找必要的包
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS
    common
    io
    search
    point_types
)
find_package(Ceres REQUIRED)

# 添加库目标
add_library(data_loader SHARED data_loader.cc)

# 设置包含目录
target_include_directories(data_loader PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include/data_loader>
)

# add_executable(data_loader data_loader.cc)

# 设置公共头文件
set(PUBLIC_HEADERS  
    data_loader.h
)

target_link_libraries(data_loader 
    common_utils     
    ${CERES_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Eigen3::Eigen
)



# 设置目标属性
set_target_properties(data_loader PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
)

# 安装目标
install(TARGETS data_loader
    # EXPORT lidar_visual_alignmentTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/data_loader
)

