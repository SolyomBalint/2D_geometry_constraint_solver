cmake_minimum_required(VERSION 4.1)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Allow building as a standalone project or as a subdirectory
if(NOT TARGET structures)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../structures
                     ${CMAKE_CURRENT_BINARY_DIR}/structures_build)
endif()

# Collect source files from src/ and subdirectories
file(GLOB_RECURSE CONSTRAINT_SOLVER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(constraint_solver STATIC ${CONSTRAINT_SOLVER_SOURCES})

target_include_directories(constraint_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(constraint_solver
    PUBLIC  structures Eigen3::Eigen autodiff::autodiff
    PRIVATE spdlog::spdlog)

target_compile_definitions(constraint_solver PRIVATE SPDLOG_USE_STD_FORMAT)

target_compile_options(constraint_solver PRIVATE
    $<$<CONFIG:Debug>:-ggdb>
    -pedantic-errors -Wall -Wextra -Wconversion -Wsign-conversion)
