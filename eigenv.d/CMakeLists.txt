
### Created: Feb 19, 2020opied from PCL

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(extract_indices)

find_package(PCL 1.7 REQUIRED)
add_definitions (-std=c++11)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

### HOW about this!
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (Box main.cpp )
target_link_libraries (Box ${PCL_LIBRARIES})
