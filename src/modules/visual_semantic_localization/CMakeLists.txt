cmake_minimum_required(VERSION 3.10)

project(visual_semantic_localization VERSION 1.0.0 LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.8 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(visual_semantic_localization SHARED
    visual_semantic_localization_app.cc
    visual_semantic_localization.cc
)

target_include_directories(visual_semantic_localization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include/visual_semantic_localization>
)

target_link_libraries(visual_semantic_localization PUBLIC
    camera_model
    common_utils
    data_loader
    ${CERES_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Eigen3::Eigen
    yaml-cpp
    )

set(PUBLIC_HEADERS
    visual_semantic_localization_app.h
    visual_semantic_localization.h
    visual_semantic_localization_options.h
    data_types.h
)

set_target_properties(visual_semantic_localization PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
)

# 安装目标和头文件
install(TARGETS visual_semantic_localization
    # EXPORT visual_semantic_localizationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/visual_semantic_localization
)

# 安装config
install(FILES
    config/visual_semantic_localization.yaml
    DESTINATION config/visual_semantic_localization
)