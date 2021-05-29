SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
#SET(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# user build settings
SET(TRACE FALSE)
SET(USE_LOOP_CLOSING FALSE)

# Set definitions
IF(TRACE)
  ADD_DEFINITIONS(-DSVO_TRACE)
ENDIF()

IF(USE_LOOP_CLOSING)
  ADD_DEFINITIONS(-DSVO_LOOP_CLOSING)
ENDIF()

ADD_DEFINITIONS(-DSVO_USE_ROS)
ADD_DEFINITIONS(-DSVO_USE_OPENGV)
ADD_DEFINITIONS(-DSVO_DEPTHFILTER_IN_REPROJECTOR)
#ADD_DEFINITIONS(-DSVO_USE_VIKIT_CAMERA)

# TODO add this to the config.h.in option file when we merge the cuda branch
if (PANGOLIN_FOUND)
  add_definitions(-DSVO_USE_PANGOLIN)
endif()

#############################################################################
# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas -Wno-unused-but-set-parameter -Wno-int-in-bool-context")

IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
  ADD_DEFINITIONS(-DHAVE_FAST_NEON)
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3 -mno-avx")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -ffast-math -fno-finite-math-only")
