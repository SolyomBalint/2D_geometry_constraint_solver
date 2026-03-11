cmake_minimum_required(VERSION 4.1)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Allow building as a standalone project or as a subdirectory
if(NOT TARGET structures)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../structures
                     ${CMAKE_CURRENT_BINARY_DIR}/structures_build)
endif()

# Collect source files from src/ and subdirectories
file(GLOB_RECURSE CONSTRAINT_SOLVER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(constraint_solver SHARED ${CONSTRAINT_SOLVER_SOURCES})
add_library(Gcs::constraint_solver ALIAS constraint_solver)

target_include_directories(constraint_solver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/includes>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(constraint_solver
    PUBLIC structures Eigen3::Eigen autodiff::autodiff spdlog::spdlog)

set_target_properties(constraint_solver PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES)

target_compile_definitions(constraint_solver PRIVATE
    SPDLOG_USE_STD_FORMAT
    GCS_BUILDING_LIBRARY)

target_compile_options(constraint_solver PRIVATE
    $<$<CONFIG:Debug>:-ggdb>
    -pedantic-errors -Wall -Wextra -Wconversion -Wsign-conversion)

install(TARGETS constraint_solver EXPORT GcsTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/includes/gcs/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gcs)

install(EXPORT GcsTargets
    FILE GcsTargets.cmake
    NAMESPACE Gcs::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Gcs)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GcsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/GcsConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Gcs)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/GcsConfigVersion.cmake
    VERSION 0.1.0
    COMPATIBILITY SameMajorVersion)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/GcsConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/GcsConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Gcs)
