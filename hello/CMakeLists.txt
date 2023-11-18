cmake_minimum_required(VERSION 3.18)

project(hello)

find_package(Open3D REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} Open3D::Open3D)
