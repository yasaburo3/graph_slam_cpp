cmake_minimum_required(VERSION 2.8.3)
project(graph_slam_cpp)
set( CMAKE_CXX_FLAGS "-std=c++11" )

include_directories(include)

include(cmake/eigen.cmake)

find_package(Python3 COMPONENTS Development NumPy)

aux_source_directory(./src SRCS)
add_executable(graph_slam ${SRCS})


target_include_directories(graph_slam PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
target_link_libraries(graph_slam Python3::Python Python3::NumPy)