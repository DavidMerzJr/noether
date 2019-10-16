cmake_minimum_required(VERSION 3.5.0)
project(robust_edge_extraction VERSION 0.1.0 LANGUAGES CXX)

find_package(console_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenMP REQUIRED)
if(NOT TARGET OpenMP::OpenMP_CXX)
    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

endif()

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
#  CATKIN_DEPENDS
  DEPENDS
    console_bridge
    PCL
    EIGEN3
    OpenMP
)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

add_library(${PROJECT_NAME} SHARED src/robust_edge_extraction.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES} console_bridge OpenMP::OpenMP_CXX)
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
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_demo src/demo.cpp)
target_link_libraries(${PROJECT_NAME}_demo PUBLIC ${PROJECT_NAME} ${PCL_LIBRARIES} console_bridge)
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

#install(TARGETS ${PROJECT_NAME} DESTINATION lib)
#install(TARGETS ${PROJECT_NAME}_demo DESTINATION bin)
#install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
