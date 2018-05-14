#ifndef INTERFACESR300_ORB_H_
#define INTERFACESR300_ORB_H_

#include <string>
#include <iostream>
#include <algorithm>
#include <map>

#include "ThreadMutexObject.h"
#include <librealsense/rs.hpp>
#include "System.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
class SR300_ORB_Interface
{
public:
    std::string depthCameraConfigYamlPath;
    std::string orbVocBinPath;
    ORB_SLAM2::System *pSLAM;
    SR300_ORB_Interface(int inWidth = 640, int inHeight = 480, int fps = 30,
                        std::string argInDepthCameraYamlPath = std::string("/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Realsense_SR300.yaml"),
                        std::string argInOrbVocBinPath = std::string("/home/gao/Downloads/ORB_SLAM2/Vocabulary/ORBvoc.bin"));
    virtual ~SR300_ORB_Interface();

    const int width, height, fps;

    bool ok()
    {
        return initSuccessful;
    }

    std::string error()
    {
        errorText.erase(std::remove_if(errorText.begin(), errorText.end(), &SR300_ORB_Interface::isTab), errorText.end());
        return errorText;
    }

    static const int numBuffers = 1000;
    ThreadMutexObject<int> latestAllFrameIndex;
    ThreadMutexObject<bool> shouldStop;
    ros::Time rgbImageRosTimeBuffers[numBuffers];
    std::pair<uint8_t *, int64_t> rgbBuffers[numBuffers];
    std::pair<uint8_t *, int64_t> depthAlignedToRgbBuffers[numBuffers];
    std::pair<Eigen::Matrix4f, int64_t> cameraToObjectTransMatBuffers[numBuffers];
    float fx, fy, cx, cy;

private:
    std::thread rosThread;
    rs::context ctx;
    rs::device * dev;
    std::thread allFrameDaemonThread;

    int64_t lastAllFrameTime;

    bool initSuccessful;
    std::string errorText;

    //For removing tabs from OpenNI's error messages
    static bool isTab(char c)
    {
        switch(c)
        {
            case '\t':
                return true;
            default:
                return false;
        }
    }
};
#endif
