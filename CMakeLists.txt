cmake_minimum_required(VERSION 3.16.3)
message("Building with CMake version: ${CMAKE_VERSION}...")

# TODO: Set the GCC version
project(VisualGroupCoveragePlanner CXX)
message("Project: ${PROJECT_NAME}")

set(CMAKE_CXX_STANDARD 20)
message("${PROJECT_NAME}: Setting CPP version to: ${CMAKE_CXX_STANDARD}")

# Bring the public headers into the project
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/libs)

file(COPY ${CMAKE_SOURCE_DIR}/res/ DESTINATION ${CMAKE_BINARY_DIR}/)

# Add subdirectories
message("${PROJECT_NAME}: Adding subdirectories...")

# TODO wrap in if to check if ai toolbox is found
set(MAKE_LIB 1)
set(AI_LOGGING_ENABLED 1)
include_directories(${PROJECT_SOURCE_DIR}/libs/AI-Toolbox/include)
add_subdirectory(libs/AI-Toolbox)

add_subdirectory(libs/imgui_impl)
add_subdirectory(libs/cnpy)

# Finding all the dependency package
# ######### ADD DEPENDENCY PACKAGE HERE ############
if(NOT TARGET range-v3)
    find_package(range-v3 CONFIG REQUIRED)
    message("${PROJECT_NAME}: Finding package range-v3")
endif()

if(NOT TARGET Eigen3)
    find_package(Eigen3 CONFIG REQUIRED)
    message("${PROJECT_NAME}: Finding package Eigen3")
endif()

if(NOT TARGET blend2d)
    find_package(blend2d CONFIG REQUIRED)
    message("${PROJECT_NAME}: Finding package blend2d")
endif()

if(NOT TARGET glfw3)
    find_package(glfw3 CONFIG REQUIRED)
    message("${PROJECT_NAME}: Finding package glfw3")
endif()

if(NOT TARGET GLEW)
    find_package(GLEW CONFIG REQUIRED)
    message("${PROJECT_NAME}: Finding package glew")
endif()

if(NOT TARGET glm)
    find_package(glm CONFIG REQUIRED)
    message("${PROJECT_NAME}: Finding package glm")
endif()

if(NOT TARGET imgui)
    find_package(imgui CONFIG REQUIRED)
    message("${PROJECT_NAME}: Finding package imgui")
endif()

if(NOT TARGET spdlog)
    find_package(spdlog CONFIG REQUIRED)
    message("${PROJECT_NAME}: Finding package spdlog")
endif()

if(NOT TARGET yaml-cpp)
    find_package(yaml-cpp CONFIG REQUIRED)
    message("${PROJECT_NAME}: Finding package yaml-cpp")
endif()


# Add the source files
message("${PROJECT_NAME}: Adding the source files")
file(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE H_SOURCES ${PROJECT_SOURCE_DIR}/include/*.h)
message("Project Sources: ${SOURCES}")

# Creating a library
add_library(${PROJECT_NAME} ${SOURCES} ${H_SOURCES})

# add_library(${PROJECT_NAME} src/main.cpp src/geometry/Geoemtry.cpp src/utils/Config.cpp include/Space.h include/Geometry.h include/Config.h)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    imgui_impl
    range-v3::range-v3
    Eigen3::Eigen
    blend2d::blend2d
    glfw
    GLEW::GLEW
    glm::glm
    imgui::imgui
    spdlog::spdlog
    AIToolboxMDP
    yaml-cpp
    cnpy
)

# Create the executable
message("${PROJECT_NAME}: Creating the executable...")
add_executable(coverage-planner src/main.cpp)

# Link libraries
message("${PROJECT_NAME}: Linking libraries...")
target_link_libraries(coverage-planner
    PUBLIC
    ${PROJECT_NAME}
)

# Testing
message("Adding Tests for ${PROJECT_NAME}")
enable_testing()
add_subdirectory(test)

message("Build for project(${PROJECT_NAME}) FINISHED!")