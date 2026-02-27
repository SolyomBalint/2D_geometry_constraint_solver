cmake_minimum_required(VERSION 4.1)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find gtkmm-4.0 via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTKMM REQUIRED IMPORTED_TARGET gtkmm-4.0)

# Allow building as a standalone project or as a subdirectory
if(NOT TARGET constraint_solver)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../src/constraint_solver
                     ${CMAKE_CURRENT_BINARY_DIR}/constraint_solver_build)
endif()

# Collect GUI source files
file(GLOB GUI_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_executable(gcs_gui ${GUI_SOURCES})

target_include_directories(gcs_gui PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(gcs_gui PRIVATE
    constraint_solver
    spdlog::spdlog
    PkgConfig::GTKMM
    Eigen3::Eigen
    nlohmann_json::nlohmann_json)

target_compile_definitions(gcs_gui PRIVATE SPDLOG_USE_STD_FORMAT)

target_compile_options(gcs_gui PRIVATE
    $<$<CONFIG:Debug>:-ggdb>
    -pedantic-errors -Wall -Wextra -Wconversion -Wsign-conversion)
