cmake_minimum_required(VERSION 3.10)
project(pandora_slam)

set(CMAKE_CXX_STANDARD 17)


# set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install) # build/install
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install) # install
# set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
# set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

add_subdirectory(src)
# add_subdirectory(apps/base_app/data_loader_app)

# add_subdirectory(apps/lidar_visual_alignment_app)
add_subdirectory(run/visual_semantic_localization)

# add_subdirectory(tests/test_opencv_yaml_parse)

enable_testing()

# 安装时包含其他必要的文件
# install(DIRECTORY config DESTINATION etc)


# message("CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
# message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
# message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
# message("PROJECT_BINARY_DIR: ${PROJECT_BINARY_DIR}")
# message("CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
# message("CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")
# message("CMAKE_HOME_DIRECTORY: ${CMAKE_HOME_DIRECTORY}")
# message("CMAKE_COMMAND: ${CMAKE_COMMAND}")
# message("CMAKE_BUILD_TOOL: ${CMAKE_BUILD_TOOL}")
# message("CMAKE_SYSTEM: ${CMAKE_SYSTEM}")
# message("CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
# message("CMAKE_SYSTEM_VERSION: ${CMAKE_SYSTEM_VERSION}")
# message("CMAKE_CXX_COMPILER: ${CMAKE_CXX_COMPILER}")
# message("CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
# message("CMAKE_SIZEOF_VOID_P: ${CMAKE_SIZEOF_VOID_P}")
# message("CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

