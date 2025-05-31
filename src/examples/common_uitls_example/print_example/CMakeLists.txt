cmake_minimum_required(VERSION 3.10)
project(print_example VERSION 1.0.0 LANGUAGES CXX)

add_executable(print_example print_example.cc)

target_link_libraries(print_example PRIVATE 
    common_utils
)

set_target_properties(print_example PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
)

install(TARGETS print_example DESTINATION bin)
# install(FILES 
#     config/print_example.yaml 
#     DESTINATION config/print_example
# )



