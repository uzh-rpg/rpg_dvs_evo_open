project(dvs_reconstruction)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


SET(CMAKE_BUILD_TYPE RelWithDebInfo) # Release, RelWithDebInfo
ADD_DEFINITIONS(
    -std=c++11
    # -O3
)


################################################################################
set(HEADERS
  include/dvs_reconstruction/poisson_solver/laplace.h
  include/dvs_reconstruction/mosaic.hpp
)

set(SOURCES
  src/mosaic.cpp
  src/poisson_solver/laplace.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Nodes
################################################################################

cs_add_executable(dvs_reconstruction_ros src/dvs_reconstruction_ros.cpp)
target_link_libraries(dvs_reconstruction_ros ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  fftw3 fftw3_threads
)


################################################################################
cs_install()
cs_export()
