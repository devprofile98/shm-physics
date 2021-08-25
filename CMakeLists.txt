cmake_minimum_required(VERSION 3.14)

project(shm-physic)
find_package(Threads)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(cyclon_src
    src/particle.cpp
    src/pfgen.cpp
    src/pcontacts.cpp
    src/plinks.cpp
    src/pworld.cpp
    src/core.cpp
    src/body.cpp
    src/fgen.cpp
    src/world.cpp
    src/collide_coarse.cpp
    src/contacts.cpp
    src/collide_fine.cpp

)

#add_executable(shmphysics ${cyclon_src})
add_library(shm-physic SHARED ${cyclon_src})

target_include_directories(shm-physic PUBLIC include/ include/cyclon/)
