cmake_minimum_required(VERSION 3.10)
project(visual_semantic_localization_main VERSION 1.0.0 LANGUAGES CXX)

add_executable(visual_semantic_localization_main main.cc)

target_link_libraries(visual_semantic_localization_main common_utils visual_semantic_localization)

set_target_properties(visual_semantic_localization_main PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
)

install(TARGETS visual_semantic_localization_main DESTINATION bin)

