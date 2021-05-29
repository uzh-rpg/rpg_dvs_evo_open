project(svo)
cmake_minimum_required(VERSION 2.8.3)

################################################################################
# user build settings
option(SVO_WITH_CUDA "option to use CUDA within SVO" OFF)
option(SVO_WITH_PANGOLIN "option to use PANGOLIN within SVO" OFF)


################################################################################
find_package(catkin_simple)
catkin_simple()

################################################################################
# generic setup (compiler flags, etc.)
include(SvoSetup)

set(CUDA_FOUND false)
if(${cuda_catkin_FOUND})
 find_cuda()
 if ((NOT ${CUDA_FOUND}) OR (${CUDA_VERSION_MAJOR} LESS 7))
   set(CUDA_FOUND false)
 endif()
endif()

if(NOT ${CUDA_FOUND})
  set(SVO_WITH_CUDA OFF)
endif()

message("SVO_WITH_CUDA: ${SVO_WITH_CUDA}")
configure_file(include/svo/config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/svo/config.h)

list(APPEND SVO_SOURCEFILES
  src/frame_handler_mono.cpp
  src/frame_handler_stereo.cpp
  src/frame_handler_array.cpp
  src/frame_handler_base.cpp
  src/map.cpp
  src/pose_optimizer.cpp
  src/initialization.cpp
  src/reprojector.cpp
  src/imu_handler.cpp
  #src/patch_normal.cpp
  src/stereo_triangulation.cpp
  src/io.cpp
)

list(APPEND SVO_HEADERFILES
  include/svo/abstract_bundle_adjustment.h
  include/svo/frame_handler_base.h
  include/svo/frame_handler_mono.h
  include/svo/frame_handler_stereo.h
  include/svo/frame_handler_array.h
  include/svo/global.h
  include/svo/imu_handler.h
  include/svo/initialization.h
  include/svo/map.h
  include/svo/pose_optimizer.h
  include/svo/reprojector.h
  include/svo/stereo_triangulation.h
  include/svo/io.h
  include/svo/config.h
  include/svo/svo.h
  )

################################################################################
# Create svo library
include_directories(include ${INCLUDE_DIRS})
cs_add_library(svo SHARED ${SVO_SOURCEFILES} ${SVO_HEADERFILES})
target_link_libraries(svo ${LINK_LIBS})

################################################################################
cs_install()
cs_export()
