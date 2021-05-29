project(svo_common)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(SvoSetup)

set(HEADERS
  include/svo/common/camera.h
  include/svo/common/camera_fwd.h
  include/svo/common/feature_wrapper.h
  include/svo/common/frame.h
  include/svo/common/imu_calibration.h
  include/svo/common/logging.h
  include/svo/common/occupancy_grid_2d.h
  include/svo/common/point.h
  include/svo/common/seed.h
  include/svo/common/transformation.h
  include/svo/common/types.h
  )

set(SOURCES
  src/frame.cpp
  src/point.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

################################################################################
cs_install()
cs_export()
