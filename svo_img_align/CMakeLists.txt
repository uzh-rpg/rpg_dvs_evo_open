project(svo_img_align)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple)
catkin_simple()

include(SvoSetup)

set(HEADERS_CPU
  include/svo/img_align/sparse_img_align_base.h
  include/svo/img_align/sparse_img_align.h
  )

set(SOURCES_CPU
  src/sparse_img_align_base.cpp
  src/sparse_img_align.cpp
  )

set(HEADERS_GPU
  include/svo/img_align/gpu_types_cuda.h
  include/svo/img_align/frame_gpu.h
  include/svo/img_align/sparse_img_align_gpu.h
  include/svo/img_align/sparse_img_align_device_utils.cuh
  )

set(SOURCES_GPU
  src/frame_gpu.cpp
  src/sparse_img_align_gpu.cpp
  src/sparse_img_align_device_utils.cu
  )

set(CUDA_FOUND false)
if(${cuda_catkin_FOUND})
 find_cuda()
 if ((NOT ${CUDA_FOUND}) OR (${CUDA_VERSION_MAJOR} LESS 7))
   set(CUDA_FOUND false)
 endif()
endif()

if (${CUDA_FOUND})
 cs_cuda_add_library(${PROJECT_NAME} ${SOURCES_CPU} ${SOURCES_GPU} ${HEADERS_CPU} ${HEADERS_GPU})
else()
 cs_add_library(${PROJECT_NAME} ${SOURCES_CPU} ${HEADERS_CPU})
endif()
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

cs_install()
cs_export()
