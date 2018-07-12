cmake_minimum_required(VERSION 2.8.8)

# Project name
project(SFO)

# Include our cmake files
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules/)

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

# Include libraries
#find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV 3 REQUIRED core imgcodecs videoio ximgproc features2d xfeatures2d)
find_package(GeographicLib 1.34 REQUIRED)
find_package(Pangolin REQUIRED)

# Include our header files
include_directories(
        include
        ${Pangolin_INCLUDE_DIR}
        thirdparty
        ${EIGEN3_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS} # For Boost::Filesystem
	    ${OpenCV_INCLUDE_DIRS}
        ${GeographicLib_INCLUDE_DIRS}
)

# Include all thirdparty source files in build
file(GLOB_RECURSE sources_thirdparty thirdparty/libviso2/*.cpp)

#####################################
# Main class files
#####################################
set(sources_SFO
        src/gtsamTracker.cpp
        src/mapDrawer.cpp
        src/frameDrawer.cpp
        src/system.cpp
        src/loadFunctions.cpp
)

set(sources_ORB
#        src/tracking.cpp
#        src/ORBextractor.cc
#        src/ORBmatcher.cc
#        src/Frame.cc
#        src/Converter.cc
)


# Create the main executable, and link libraries
add_executable(kitti-stereo
        examples/kitti-stereo.cpp
        ${sources_SFO}
        ${sources_thirdparty}
        #${sources_ORB}
)

target_link_libraries(kitti-stereo
        ${Pangolin_LIBRARY}
        gtsam
        ${GTSAM_LIBS}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${GeographicLib_LIBRARIES}

        ${PROJECT_SOURCE_DIR}/thirdparty/DBoW2/lib/libDBoW2.so
)

