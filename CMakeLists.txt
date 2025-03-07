cmake_minimum_required(VERSION 3.30)

project(2d_geometry_constraint_solver)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(imgui REQUIRED)
find_package(imnodes REQUIRED)

add_executable(${PROJECT_NAME}  src/main.cpp 
                                src/imgui_rendering/graph_render.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ./include
            ${OpenGL_INCLUDE_DIRS}
            ${glfw3_INCLUDE_DIRS}
            ${imgui_INCLUDE_DIRS}
            ${imnodes_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE glfw imgui imnodes OpenGL::GL)
target_compile_options(${PROJECT_NAME} PRIVATE -Werror)
