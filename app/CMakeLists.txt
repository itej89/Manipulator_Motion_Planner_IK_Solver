cmake_minimum_required(VERSION 3.14)
project(app)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(app
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)


target_link_libraries(app  ik_solver matplotlib_cpp Eigen3::Eigen)