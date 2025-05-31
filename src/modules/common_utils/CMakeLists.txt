cmake_minimum_required(VERSION 3.10)
project(common_utils VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_library(common_utils SHARED
    print.cc
)

target_include_directories(common_utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include/common_utils>
)

target_link_libraries(common_utils 
    ${OpenCV_LIBS}
)

set(PUBLIC_HEADERS
    colors.h
    eigen_types.h
    math_utils.h
    opencv_yaml_parse.h
    print.h
    so3_utils.h
    utils.h
)

set_target_properties(common_utils PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
)

# 安装目标和头文件
# install(TARGETS common_utils DESTINATION lib)
install(TARGETS common_utils
    # EXPORT common_utilsTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/common_utils
)



# install(FILES
#     colors.h
#     eigen_types.h
#     math_utils.h
#     opencv_yaml_parse.h
#     print.h
#     so3_utils.h
#     DESTINATION include/common_utils
# )
