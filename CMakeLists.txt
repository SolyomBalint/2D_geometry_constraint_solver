cmake_minimum_required(VERSION 4.1)

project(2d_geometry_constraint_solver)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS true)

message(STATUS "Setting CMAKE_DEVENV_PROFILE to $ENV{DEVENV_PROFILE}")
set(CMAKE_DEVENV_PROFILE $ENV{DEVENV_PROFILE})

######################## Conan-provided dependencies ########################

find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

######################## FetchContent dependencies ########################

include(FetchContent)

set(AUTODIFF_VERSION "v1.1.2")
set(AUTODIFF_BUILD_TESTS OFF CACHE BOOL "Turn off autodiff test build" FORCE)
set(AUTODIFF_BUILD_EXAMPLES OFF CACHE BOOL "Turn off autodiff example build" FORCE)
set(AUTODIFF_BUILD_DOCS OFF CACHE BOOL "Turn off autodiff doc build" FORCE)
set(AUTODIFF_BUILD_PYTHON OFF CACHE BOOL "Turn off autodiff python bind generation" FORCE)

FetchContent_Declare(
    autodiff
    GIT_REPOSITORY https://github.com/autodiff/autodiff/
    GIT_TAG ${AUTODIFF_VERSION}
)

FetchContent_MakeAvailable(autodiff)

######################## Sub-projects ########################

add_subdirectory(src/structures)
add_subdirectory(src/constraint_solver)
add_subdirectory(gui)

######################## Main executable ########################

add_executable(${PROJECT_NAME} src/main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE src)

target_link_libraries(${PROJECT_NAME} PRIVATE
    constraint_solver
    spdlog::spdlog
    autodiff::autodiff
    Eigen3::Eigen)

target_compile_definitions(${PROJECT_NAME} PRIVATE SPDLOG_USE_STD_FORMAT)

# Explanation why these flags are used:
# -ggdb: if debugging use it for full support of gdb
# -pedantic-errors: only support standard c++ features
# The rest turns on several useful warnings
# TODO: make sure that the flags are correct for different compilers such as gcc and clang
target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:-ggdb>
    -pedantic-errors -Wall -Wextra -Wconversion -Wsign-conversion)
