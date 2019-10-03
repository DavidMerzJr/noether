cmake_minimum_required(VERSION 3.5.0)
project(pcl_bspline_reconstruction VERSION 0.1.0 LANGUAGES CXX)

find_package(console_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED)
catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
#  CATKIN_DEPENDS
  DEPENDS
    console_bridge
    PCL
)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

add_library(${PROJECT_NAME} SHARED src/bspline_reconstruction.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES} console_bridge)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME} PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_demo src/demo.cpp)
target_link_libraries(${PROJECT_NAME}_demo PUBLIC ${PCL_LIBRARIES} ${PROJECT_NAME} console_bridge)
target_compile_options(${PROJECT_NAME}_demo PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_demo PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_demo PUBLIC cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_demo PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_demo SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(TARGETS ${PROJECT_NAME}_demo DESTINATION bin)
#install(FILES package.xml DESTINATION share/${PROJECT_NAME})
