cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE RelWithDebInfo)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -march=native")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable compile optimizations
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")

# Enable debug flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3  -Wall")

# SSE3 is needed by libviso2 (its a cpu arch attribute)
# If we are on ARM we need to find an alternative
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3")

message(STATUS "Compiler set to: ${CMAKE_CXX_COMPILER}")
message(STATUS "Comper flags: ${CMAKE_CXX_FLAGS}")

set(CMAKE_BUILD_TYPE Debug)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

# Include libraries
#find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV 3 REQUIRED core imgcodecs videoio ximgproc features2d xfeatures2d)
find_package(GeographicLib 1.34 REQUIRED)
find_package(Pangolin REQUIRED)

# Include our header files
include_directories(
        ${PROJECT_SOURCE_DIR}/../../../include
        ${Pangolin_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/../../../thirdparty
        ${EIGEN3_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS} # For Boost::Filesystem
        ${OpenCV_INCLUDE_DIRS}
        ${GeographicLib_INCLUDE_DIRS}
)

set(sources_SFO
        ${PROJECT_SOURCE_DIR}/../../../src/gtsamTracker.cpp
        ${PROJECT_SOURCE_DIR}/../../../src/mapDrawer.cpp
        ${PROJECT_SOURCE_DIR}/../../../src/frameDrawer.cpp
        ${PROJECT_SOURCE_DIR}/../../../src/system.cpp
        ${PROJECT_SOURCE_DIR}/../../../src/loadFunctions.cpp
)

set(sources_ORB
        #        src/tracking.cpp
        #        src/ORBextractor.cc
        #        src/ORBmatcher.cc
        #        src/Frame.cc
        #        src/Converter.cc
)

# Create the main executable, and link libraries
add_executable(ros-stereo
        src/ros_stereo.cpp
        ${sources_SFO}
        ${sources_thirdparty}

        #${sources_ORB}
)

target_link_libraries(ros-stereo
        ${Pangolin_LIBRARY}
        gtsam
        ${GTSAM_LIBS}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${GeographicLib_LIBRARIES}

        #${PROJECT_SOURCE_DIR}/thirdparty/DBoW2/lib/libDBoW2.so
)


# Node for stereo camera
rosbuild_add_executable(ros-stereo
        src/ros_stereo.cpp
)

target_link_libraries(Stereo
${LIBS}
)


