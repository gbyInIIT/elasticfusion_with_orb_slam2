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

#include "RosInterface.h"
#include <unistd.h>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

RosInterface::RosInterface(int inWidth, int inHeight, int fps)
 : width(inWidth),
   height(inHeight),
   fps(fps),
   initSuccessful(true)
{
    latestAllFrameIndex.assign(-1);
    for(int i = 0; i < numBuffers; i++) {
        uint8_t * newDepthImage = (uint8_t *)calloc(width * height * 2, sizeof(uint8_t));
        uint8_t * newRgbImage = (uint8_t *)calloc(width * height * 3, sizeof(uint8_t));
        depthBuffers[i] = std::pair<uint8_t *, int64_t>(newDepthImage, 0);
        rgbBuffers[i] = std::pair<uint8_t *, int64_t>(newRgbImage, 0);
    }
    ThreadMutexObject<bool> isFinished(false);
    auto rosAction = [this]() {
        int argc = 0;
        ros::init(argc, NULL, "depth_rgb_elasticfusion_sub");
        ros::start();
        ros::NodeHandle nh;
        message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
        message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
        message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
        sync.registerCallback(boost::bind(&RosInterface::depthRgbMsgCallback, this, _1,_2));
        ros::spin();
        ros::shutdown();
    };
    rosThread = std::move(std::thread(rosAction));
}

void RosInterface::depthRgbMsgCallback(const sensor_msgs::ImageConstPtr& rgbImage, const sensor_msgs::ImageConstPtr& depthImage)
{
    cv_bridge::CvImagePtr cvDepthImagePtr;
    try {
        cvDepthImagePtr = cv_bridge::toCvCopy(depthImage, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_bridge::CvImagePtr cvRgbImagePtr;
    try {
        cvRgbImagePtr = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    lastAllFrameTime = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
    const cv::Size depthImageSize = cvDepthImagePtr->image.size();
    const cv::Size rgbImageSize = cvRgbImagePtr->image.size();
    if (rgbImageSize.height != depthImageSize.height || rgbImageSize.width != depthImageSize.width) {
        ROS_ERROR("RGB image size != depth image size!!!");
        return;
    }
    const int nPixel = depthImageSize.height * depthImageSize.width;
    if (!cvRgbImagePtr->image.isContinuous()) {
        ROS_ERROR("RGB image is not continuous!!!");
        return;
    }
    if (!cvDepthImagePtr->image.isContinuous()) {
        ROS_ERROR("Depth image is not continuous!!!");
        return;
    }
    int bufferIndex = (latestAllFrameIndex.getValue() + 1) % numBuffers;
    memcpy(depthBuffers[bufferIndex].first, cvDepthImagePtr->image.data, nPixel * 2);
    memcpy(rgbBuffers[bufferIndex].first, cvRgbImagePtr->image.data, nPixel * 3);
    depthBuffers[bufferIndex].second = lastAllFrameTime;
    rgbBuffers[bufferIndex].second = lastAllFrameTime;
    latestAllFrameIndex++;
}

RosInterface::~RosInterface()
{
    if(initSuccessful) {
        printf("RosInterface dead.");
        for (int i = 0; i < numBuffers; i++) {
            free(depthBuffers[i].first);
            free(rgbBuffers[i].first);
        }
        rosThread.join();
    }
}


