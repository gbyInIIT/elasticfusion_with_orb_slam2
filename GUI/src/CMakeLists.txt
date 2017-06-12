cmake_minimum_required(VERSION 2.6.0)

project(ElasticFusion)

#set(ORB_SOURCE_DIR "/home/gao/Downloads/ORB_SLAM2")
if (DEFINED ORB_SOURCE_DIR)
    if(NOT EXISTS ${ORB_SOURCE_DIR})
        message( FATAL_ERROR "${ORB_SOURCE_DIR} does NOT exist." )
    endif()
else()
    message( FATAL_ERROR "ORB_SOURCE_DIR is NOT defined.")
endif()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        genmsg
        image_transport
        sensor_msgs
        tf
        )
message(STATUS "catkin_lib: ${catkin_LIBRARIES}")
include_directories(
        ${catkin_INCLUDE_DIRS}
)

find_package(ZLIB REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenNI2 REQUIRED)
set(EFUSION_LIBRARY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libefusion.so")
set(EFUSION_INCLUDE_DIR "../../Core/src")
message(STATUS "efustion lib: ${EFUSION_LIBRARY}")

include_directories(${ZLIB_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EFUSION_INCLUDE_DIR})
include_directories(${OPENNI2_INCLUDE_DIR})
include_directories(${ORB_SOURCE_DIR})
include_directories(${ORB_SOURCE_DIR}/include)

file(GLOB srcs *.cpp)
file(GLOB tools_srcs Tools/*.cpp)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse2 -msse3 -Wall -std=c++11")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -Wall -std=c++11")

add_definitions(-Dlinux=1)

add_executable(ElasticFusionSR300_ORB
        MainSR300_ORB.cpp MainControllerSR300_ORB.cpp
        Tools/GroundTruthOdometry.cpp Tools/RawLogReader.cpp Tools/LiveLogReaderSR300_ORB.cpp Tools/SR300_ORB_Interface.cpp
        )
target_link_libraries(ElasticFusionSR300_ORB
        ${ZLIB_LIBRARY}
        ${Pangolin_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${EFUSION_LIBRARY}
        realsense
        ${catkin_LIBRARIES}
        ${ORB_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${ORB_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${ORB_SOURCE_DIR}/lib/libORB_SLAM2.so
        )
add_dependencies(ElasticFusionSR300_ORB efusion)
set(CMAKE_VERBOSE_MAKEFILE on)
