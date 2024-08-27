
cmake_minimum_required(VERSION 3.16.3)
message("Building with CMake version: ${CMAKE_VERSION}...")

project(imgui_impl CXX)

if (NOT TARGET imgui)
    find_package(imgui CONFIG REQUIRED)
    message("${PROJECT_NAME}: Finding package imgui")
endif ()

add_library(${PROJECT_NAME} STATIC 
    imgui_impl_glfw.cpp 
    imgui_impl_glfw.h 
    imgui_impl_opengl3_loader.h 
    imgui_impl_opengl3.cpp 
    imgui_impl_opengl3.h
    )

target_link_libraries(${PROJECT_NAME} imgui::imgui)