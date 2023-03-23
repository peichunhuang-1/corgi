cmake_minimum_required(VERSION 3.10)
project(CORE C CXX ASM)
set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set (CMAKE_EXE_LINKER_FLAGS)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "-std=c99")
set(CMAKE_CXX_FLAGS "-std=c99 -Wno-error=deprecated-declarations -Wno-deprecated-declarations ")

set(BOOST_COMPONENTS ALL)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/devel)

find_package(Boost REQUIRED COMPONENTS serialization thread program_options)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(NLopt REQUIRED)

link_directories(/usr/local/lib)
include_directories(${Boost_INCLUDE_DIRS})

add_subdirectory(project)
