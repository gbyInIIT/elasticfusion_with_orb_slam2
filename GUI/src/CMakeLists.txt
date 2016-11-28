cmake_minimum_required(VERSION 2.6.0)

project(ElasticFusion)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        genmsg
        image_transport
        sensor_msgs
        tf
        )
message(STATUS "${catkin_LIBRARIES}")
include_directories(
        ${catkin_INCLUDE_DIRS}
)

find_package(ZLIB REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenNI2 REQUIRED)
find_package(efusion REQUIRED)

include_directories(${ZLIB_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EFUSION_INCLUDE_DIR})
include_directories(${OPENNI2_INCLUDE_DIR})

file(GLOB srcs *.cpp)
file(GLOB tools_srcs Tools/*.cpp)

set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3 -Wall -std=c++11")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -std=c++11")

add_definitions(-Dlinux=1)

add_executable(ElasticFusion
               Main.cpp MainController.cpp Tools/GroundTruthOdometry.cpp Tools/RawLogReader.cpp Tools/LiveLogReader.cpp Tools/OpenNI2Interface.cpp
)
add_executable(ElasticFusionSR300
        MainSR300.cpp MainControllerSR300.cpp
        Tools/GroundTruthOdometry.cpp Tools/RawLogReader.cpp Tools/LiveLogReaderSR300.cpp Tools/SR300Interface.cpp
        )

add_executable(ElasticFusionRos
        MainRos.cpp MainControllerRos.cpp
        Tools/GroundTruthOdometry.cpp Tools/RawLogReader.cpp Tools/LiveLogReaderRos.cpp Tools/RosInterface.cpp
        )

target_link_libraries(ElasticFusion
                      ${ZLIB_LIBRARY}
                      ${Pangolin_LIBRARIES}
                      ${CUDA_LIBRARIES}
                      ${EFUSION_LIBRARY}
                      ${OPENNI2_LIBRARY}
)

target_link_libraries(ElasticFusionSR300
        ${ZLIB_LIBRARY}
        ${Pangolin_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${EFUSION_LIBRARY}
        realsense
        )

target_link_libraries(ElasticFusionRos
        ${ZLIB_LIBRARY}
        ${Pangolin_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${EFUSION_LIBRARY}
        ${catkin_LIBRARIES}
        )

set(CMAKE_VERBOSE_MAKEFILE on)
