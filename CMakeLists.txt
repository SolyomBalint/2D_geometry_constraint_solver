cmake_minimum_required(VERSION 3.31)

project(2d_geometry_constraint_solver)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)
# Needed bu spdlog to compile
set(CMAKE_POSITION_INDEPENDENT_CODE true)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(imgui REQUIRED)
find_package(imnodes REQUIRED)
find_package(spdlog REQUIRED)
find_package(argparse REQUIRED)
find_package(Boost REQUIRED)

# IMPORTANT NOTE: Currently this project outputs only one binary. There is no
# modularization in respect to the logical modules via archives or shared
# libraries. DO NOT expect source file to work by themselves without the
# correct linking

add_executable(${PROJECT_NAME}  src/main.cpp
                                src/imgui_rendering/graph_render.cpp
                                src/utils/graph.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ./include
            ${OpenGL_INCLUDE_DIRS}
            ${glfw3_INCLUDE_DIRS}
            ${imgui_INCLUDE_DIRS}
            ${imnodes_INCLUDE_DIRS}
            ${spdlog_INCLUDE_DIRS}
            ${argparse_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog glfw imgui argparse::argparse
                                        imnodes OpenGL::GL ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE -Werror $<$<CONFIG:Debug>:-ggdb>)
