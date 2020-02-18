
### Copied from PCL
### http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
###

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(extract_indices)

find_package(PCL 1.7 REQUIRED common segmentation kdtree visualization)
### find_package(PCL 1.7 REQUIRED common segmentation visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

### HOW about this!
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (extract_indices extract_indices.cpp)
target_link_libraries (extract_indices ${PCL_LIBRARIES})
