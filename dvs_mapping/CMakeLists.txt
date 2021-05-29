project(dvs_mapping)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

SET(CMAKE_BUILD_TYPE RelWithDebInfo) # Release, RelWithDebInfo
ADD_DEFINITIONS(
    -std=c++11
    -fopenmp
    -w
    -Wfatal-errors
)

################################################################################

set(HEADERS
  include/dvs_depth_from_defocus/depth_defocus_node.hpp
  include/dvs_depth_from_defocus/depth_vector.hpp
  include/dvs_depth_from_defocus/dvs_mapping_nodelet.h
)

set(SOURCES
  src/depth_defocus_node.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Executables
################################################################################

cs_add_executable(dvs_mapping_ros src/dvs_mapping_ros.cpp)
target_link_libraries(dvs_mapping_ros ${PROJECT_NAME})


cs_add_library(dvs_mapping_nodelet src/dvs_mapping_nodelet.cpp)
target_link_libraries(dvs_mapping_nodelet ${PROJECT_NAME})

################################################################################
# Tests

catkin_add_gtest(${PROJECT_NAME}_tests
  test/test_median_filter
  test/test_depth_vector
)
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

################################################################################
cs_install()
cs_export()

