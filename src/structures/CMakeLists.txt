cmake_minimum_required(VERSION 3.25)

project(structures_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Header-only library for the graph structures
add_library(structures INTERFACE)
target_include_directories(structures INTERFACE include)

# Demo / test executable
add_executable(graph_test ../../main/test.cpp)
target_link_libraries(graph_test PRIVATE structures)

target_compile_options(graph_test PRIVATE
    $<$<CONFIG:Debug>:-ggdb>
    -pedantic-errors -Wall -Wextra -Wconversion -Wsign-conversion)
