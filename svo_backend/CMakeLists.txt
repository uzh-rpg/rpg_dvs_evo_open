project(svo_backend)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(SvoSetup)

set(SOURCES
  src/backend_optimizer.cpp
  include/svo/backend/backend_optimizer.h
  src/graph_manager.cpp
  include/svo/backend/graph_manager.h
  src/backend_interface.cpp
  include/svo/backend/backend_interface.h
  #src/smart_factors.cpp
  include/svo/backend/smart_factors_fwd.h
#  src/backend_factory.cpp
#  include/svo/backend/backend_factory.h
#  src/backend_visualizer.cpp
#  include/svo/backend/backend_visualizer.h
  include/svo/backend/backend_types.h
)

cs_add_library(${PROJECT_NAME} ${SOURCES})

cs_install()
cs_export()
