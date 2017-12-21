/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef INTERFACEROS_H_
#define INTERFACEROS_H_

#include <string>
#include <iostream>
#include <algorithm>
#include <map>

#include "ThreadMutexObject.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include "System.h"

class RosInterface
{
public:
    RosInterface(std::string argInDepthCameraYamlPath = std::string("/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Realsense_SR300.yaml"));
    virtual ~RosInterface();


    bool ok() {
        return initSuccessful;
    }

    static const int numBuffers = 500;
    ThreadMutexObject<int> latestAllFrameIndex;
    ros::Time rgbImageRosTimeBuffers[numBuffers];
    std::pair<uint8_t *, int64_t> depthBuffers[numBuffers];
    std::pair<uint8_t *, int64_t> rgbBuffers[numBuffers];
    std::pair<Eigen::Matrix4f, int64_t> cameraToObjectTransMatBuffers[numBuffers];
    ThreadMutexObject<bool> isCameraInitialized;
    ThreadMutexObject<bool> isSystemRunning;
    sensor_msgs::CameraInfo cameraInfo;
    void depthRgbMsgCallback(const sensor_msgs::ImageConstPtr& rgbImage, const sensor_msgs::ImageConstPtr& depthImage);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cameraInfoConstPtr);
    int width, height, fps;

    static const int nPointCloudMsgBuffer = 10;
    sensor_msgs::PointCloud2 pointCloudMsgBuffer[nPointCloudMsgBuffer];
    ThreadMutexObject<int> latestPointCloudIndex;
    ORB_SLAM2::System *pSLAM;
    std::string depthCameraConfigYamlPath;
    ThreadMutexObject<bool> isRosOk;
    ThreadMutexObject<bool> isRuningDataDaemon;//isDisplayImage;

private:
    std::thread rosThread;
    int64_t lastAllFrameTime;
    bool initSuccessful;
};
#endif
