cmake_minimum_required(VERSION 2.6)
project(map_cut)
set( CMAKE_CXX_FLAGS "-std=c++11 -o3")
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")
link_libraries(${PCL_LIBRART_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(map_cut main.cpp)
target_link_libraries(map_cut ${PCL_LIBRARIES})

install(TARGETS map_cut RUNTIME DESTINATION bin)


#target_link_libraries(map_cut ${PCL_LIBRARIES} ${PCL_IO_LIBRARIES})

