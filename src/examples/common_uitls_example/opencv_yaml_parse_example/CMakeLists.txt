cmake_minimum_required(VERSION 3.10)
project(opencv_yaml_parse_example VERSION 1.0.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(opencv_yaml_parse_example opencv_yaml_parse_example.cc)

target_link_libraries(opencv_yaml_parse_example PRIVATE 
    Eigen3::Eigen 
    ${OpenCV_LIBS}
    common_utils
)

set_target_properties(opencv_yaml_parse_example PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
)

install(TARGETS opencv_yaml_parse_example DESTINATION bin)
install(FILES 
    config/opencv_yaml_parse_example.yaml 
    DESTINATION config/opencv_yaml_parse_example
)



