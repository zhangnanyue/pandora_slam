cmake_minimum_required(VERSION 3.10)

project(visual_semantic_localization VERSION 1.0.0 LANGUAGES CXX)

# 设置源文件
set(SOURCES
    visual_semantic_localization_app.cc
    visual_semantic_localization_core.cc
)

# 设置头文件
set(PUBLIC_HEADERS
    data_types.h
    visual_semantic_localization_app.h
    visual_semantic_localization_core.h
    visual_semantic_localization_options.h
)

# 创建库目标
add_library(visual_semantic_localization SHARED ${SOURCES})

# 设置包含目录
target_include_directories(visual_semantic_localization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include/visual_semantic_localization>
)

# 链接依赖库
target_link_libraries(visual_semantic_localization PUBLIC
    camera_model
    common_utils
    data_loader
    eskf
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    Eigen3::Eigen
    yaml-cpp
    )

# 设置目标属性
set_target_properties(visual_semantic_localization PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# 安装目标
install(TARGETS visual_semantic_localization
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/visual_semantic_localization
)

# 安装配置文件
install(FILES
    config/visual_semantic_localization.yaml
    DESTINATION config/visual_semantic_localization
)