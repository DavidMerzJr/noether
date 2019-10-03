cmake_minimum_required(VERSION 2.8.3)

## Compile as C++14 
add_compile_options(-std=c++14)

project(noether_edge_paths)

find_package(VTK 7.1 REQUIRED NO_MODULE)
include(${VTK_USE_FILE})

find_package(catkin REQUIRED COMPONENTS
  noether_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    noether_msgs
    roscpp
  DEPENDS
    VTK
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(noether_edge_paths
  src/edge_paths.cpp
)
target_link_libraries(noether_edge_paths
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS noether_edge_paths
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
