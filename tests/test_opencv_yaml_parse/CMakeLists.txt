cmake_minimum_required(VERSION 3.10)
project(test_opencv_yaml_parse VERSION 1.0.0 LANGUAGES CXX)

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(test_opencv_yaml_parse test_opencv_yaml_parse.cc)

target_link_libraries(test_opencv_yaml_parse PRIVATE 
    GTest::GTest GTest::Main common_utils 
    Eigen3::Eigen 
    ${OpenCV_LIBS}
)

set_target_properties(test_opencv_yaml_parse PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
)


add_test(NAME test_opencv_yaml_parse COMMAND test_opencv_yaml_parse)
enable_testing()

install(TARGETS test_opencv_yaml_parse DESTINATION bin)
install(FILES 
    config/test_opencv_yaml_parse.yaml 
    DESTINATION config/test_opencv_yaml_parse
)



