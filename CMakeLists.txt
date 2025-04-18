cmake_minimum_required(VERSION 3.31)

project(2d_geometry_constraint_solver)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)
# Needed by spdlog to compile
set(CMAKE_POSITION_INDEPENDENT_CODE true)

message(STATUS "Setting CMAKE_DEVENV_PROFILE to $ENV{DEVENV_PROFILE}")
set(CMAKE_DEVENV_PROFILE $ENV{DEVENV_PROFILE})

configure_file(${CMAKE_CURRENT_LIST_DIR}/pyproject.toml.template ${CMAKE_CURRENT_LIST_DIR}/pyproject.toml)

find_package(spdlog REQUIRED)
find_package(argparse REQUIRED)
find_package(Boost REQUIRED)
find_package(Python3 COMPONENTS Development Interpreter REQUIRED)

# IMPORTANT NOTE: Currently this project outputs only one binary. There is no
# modularization in respect to the logical modules via archives or shared
# libraries. DO NOT expect source file to work by themselves without the
# correct linking

add_executable(${PROJECT_NAME}  src/main.cpp
                                src/utils/graph.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ./include
            ${spdlog_INCLUDE_DIRS}
            ${argparse_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog  argparse::argparse ${Boost_LIBRARIES} Python3::Python)
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:-ggdb>)
