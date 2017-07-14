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
message(STATUS "catkin_lib: ${catkin_LIBRARIES}")
include_directories(
        ${catkin_INCLUDE_DIRS}
)

find_package(ZeroMQ REQUIRED)
find_package(cppzmq REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenNI2 REQUIRED)
#find_package(efusion REQUIRED)
set(EFUSION_LIBRARY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libefusion.so")
set(EFUSION_INCLUDE_DIR "../../Core/src")
message(STATUS "efustion lib: ${EFUSION_LIBRARY}")
message(STATUS "ZeroMQ lib: ${ZeroMQ_LIBRARY}")
message(STATUS "ZeroMQ include: ${ZeroMQ_INCLUDE_DIR}")
message(STATUS "cppzmq lib: ${cppzmq_LIBRARY}")
message(STATUS "cppzmq include: ${cppzmq_INCLUDE_DIR}")

#include_directories(${ZeroMQ_INCLUDE_DIR})
include_directories(${cppzmq_INCLUDE_DIR})
include_directories(${ZLIB_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EFUSION_INCLUDE_DIR})
include_directories(${OPENNI2_INCLUDE_DIR})
set(ORB_SOURCE_DIR "/home/gao/Downloads/ORB_SLAM2")
include_directories(${ORB_SOURCE_DIR})
include_directories(${ORB_SOURCE_DIR}/include)

file(GLOB srcs *.cpp)
file(GLOB tools_srcs Tools/*.cpp)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse2 -msse3 -Wall -std=c++11")
#set(CMAKE_CXX_FLAGS" ${CMAKE_CXX_FLAGS} -msse2 -msse3 -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -Wall -std=c++11")

add_definitions(-Dlinux=1)

add_executable(ElasticFusion
               Main.cpp MainController.cpp Tools/GroundTruthOdometry.cpp Tools/RawLogReader.cpp Tools/LiveLogReader.cpp Tools/OpenNI2Interface.cpp
)
add_executable(ElasticFusionSR300
        MainSR300.cpp MainControllerSR300.cpp
        Tools/GroundTruthOdometry.cpp Tools/RawLogReader.cpp Tools/LiveLogReaderSR300.cpp Tools/SR300Interface.cpp
        )

add_executable(ElasticFusionSR300_ORB
        MainSR300_ORB.cpp MainControllerSR300_ORB.cpp
        Tools/GroundTruthOdometry.cpp Tools/RawLogReader.cpp Tools/LiveLogReaderSR300_ORB.cpp Tools/SR300_ORB_Interface.cpp
        )

add_executable(ElasticFusionRos
        MainRos.cpp MainControllerRos.cpp
        Tools/GroundTruthOdometry.cpp Tools/RawLogReader.cpp Tools/LiveLogReaderRos.cpp Tools/RosInterface.cpp
        )

add_executable(gen_image_and_camera_pose
        gen_image_and_camera_pose.cpp
        Tools/RosInterface.cpp
        )

add_executable(HandEyeCalibrate
        MainCalibration.cpp Tools/RosInterface.cpp
        )

target_link_libraries(ElasticFusion
                      ${ZLIB_LIBRARY}
                      ${Pangolin_LIBRARIES}
                      ${CUDA_LIBRARIES}
                      ${EFUSION_LIBRARY}
                      ${OPENNI2_LIBRARY}
        realsense
)

target_link_libraries(ElasticFusionSR300
        ${ZLIB_LIBRARY}
        ${Pangolin_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${EFUSION_LIBRARY}
        realsense
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

target_link_libraries(ElasticFusionRos
        ${cppzmq_LIBRARY}
        ${ZLIB_LIBRARY}
        ${Pangolin_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${EFUSION_LIBRARY}
#        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${ORB_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${ORB_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${ORB_SOURCE_DIR}/lib/libORB_SLAM2.so
        )

target_link_libraries(HandEyeCalibrate
        ${ZLIB_LIBRARY}
        ${Pangolin_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${EFUSION_LIBRARY}
        #        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${ORB_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${ORB_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${ORB_SOURCE_DIR}/lib/libORB_SLAM2.so
        )

target_link_libraries(gen_image_and_camera_pose
        ${ZLIB_LIBRARY}
        ${Pangolin_LIBRARIES}
#        ${CUDA_LIBRARIES}
#        ${EFUSION_LIBRARY}
        ${catkin_LIBRARIES}
        ${ORB_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${ORB_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${ORB_SOURCE_DIR}/lib/libORB_SLAM2.so
        )
set(CMAKE_VERBOSE_MAKEFILE on)

add_dependencies(HandEyeCalibrate efusion)
add_dependencies(ElasticFusion efusion)
add_dependencies(ElasticFusionRos efusion)
add_dependencies(ElasticFusionSR300 efusion)
add_dependencies(ElasticFusionSR300_ORB efusion)
