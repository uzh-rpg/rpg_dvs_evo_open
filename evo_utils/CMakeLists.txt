cmake_minimum_required(VERSION 2.8.3)
project(evo_utils)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})

set(HEADERS
  include/evo_utils/camera.hpp
  include/evo_utils/geometry.hpp
  include/evo_utils/interpolation.hpp
  include/evo_utils/main.hpp
  include/rpg_common_ros/params_helper.hpp
)

set(SOURCES
  src/utils_geometry.cpp
  src/camera.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

cs_install()
cs_export()