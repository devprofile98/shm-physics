cmake_minimum_required(VERSION 3.14)

project(shm-physic)
find_package(Threads)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#set(shm_src
#    F:/project/SHM/PlayGround/world.cpp
#    source/glad.c
#    source/Engine.cpp
#    source/context_manager.cpp
#    source/Handler.cpp
#    source/Camera.cpp
#    source/Light.cpp

#)

add_executable(playground src/main.cpp)
#add_library(shm-engine SHARED ${opengl_renderer_src} ${shm_src} )

target_include_directories(playground PUBLIC include/ include/cyclon/)
