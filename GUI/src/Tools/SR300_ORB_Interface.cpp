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

#include "SR300_ORB_Interface.h"
#include <unistd.h>

SR300_ORB_Interface::SR300_ORB_Interface(int inWidth, int inHeight, int fps, std::string argInDepthCameraYamlPath = std::string("/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Realsense_SR300.yaml")
 : width(inWidth),
   height(inHeight),
   fps(fps),
   initSuccessful(true), depthCameraConfigYamlPath(argInDepthCameraYamlPath)
{
    pSLAM = new ORB_SLAM2::System("/home/gao/Downloads/ORB_SLAM2/Vocabulary/ORBvoc.txt",
                                  depthCameraConfigYamlPath, ORB_SLAM2::System::RGBD,false);
//                           "/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Realsense_SR300.yaml", ORB_SLAM2::System::RGBD,false);
//                                "/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Realsense_R200.yaml", ORB_SLAM2::System::RGBD,false);
//                           "/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Asus.yaml", ORB_SLAM2::System::RGBD,false);
//                                  "/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Arrfou.yaml", ORB_SLAM2::System::RGBD,false);
    // Create a context object. This object owns the handles to all connected realsense devices.
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) {
        fprintf(stderr, "No device found!");
    }
    // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    // Configure all streams to run at VGA resolution at 60 frames per second
    dev->enable_stream(rs::stream::depth, inWidth, inHeight, rs::format::z16, fps);
    dev->enable_stream(rs::stream::color, inWidth, inHeight, rs::format::rgb8, fps);
    dev->enable_stream(rs::stream::infrared, inWidth, inHeight, rs::format::y8, fps);
    try { dev->enable_stream(rs::stream::infrared2, inWidth, inHeight, rs::format::y8, fps); }
    catch(...) { printf("Device does not provide infrared2 stream.\n"); }
    initSuccessful = true;
    printf("depth scale: %f\n", dev->get_depth_scale());
    const rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
    const rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
    latestDepthIndex.assign(-1);
    latestRgbIndex.assign(-1);
    latestAllFrameIndex.assign(-1);
    for(int i = 0; i < numBuffers; i++) {
        uint8_t * newDepthImage = (uint8_t *)calloc(width * height * 2, sizeof(uint8_t));
        uint8_t * newRgbImage = (uint8_t *)calloc(width * height * 3, sizeof(uint8_t));
        uint8_t * newDepthAlignedToRgbImage = (uint8_t *)calloc(width * height * 2, sizeof(uint8_t));
        uint8_t * newRgbAlignedToDepthImage = (uint8_t *)calloc(width * height * 3, sizeof(uint8_t));
        depthBuffers[i] = std::pair<uint8_t *, int64_t>(newDepthImage, 0);
        rgbBuffers[i] = std::pair<uint8_t *, int64_t>(newRgbImage, 0);
        depthAlignedToRgbBuffers[i] = std::pair<uint8_t *, int64_t>(newDepthAlignedToRgbImage, 0);
        rgbAlginedToDepthBuffers[i] = std::pair<uint8_t *, int64_t>(newRgbAlignedToDepthImage, 0);
    }
    for(int i = 0; i < numBuffers; i++) {
        uint8_t * newDepth = (uint8_t *)calloc(width * height * 2, sizeof(uint8_t));
        uint8_t * newImage = (uint8_t *)calloc(width * height * 3, sizeof(uint8_t));
        frameBuffers[i] = std::pair<std::pair<uint8_t *, uint8_t *>, int64_t>(std::pair<uint8_t *, uint8_t *>(newDepth, newImage), 0);
    }
    auto all_stream_feeder = [this]() {
        while(!dev->is_streaming()) {
            usleep(1000);
        }
        int nPixel = dev->get_stream_width(rs::stream::color_aligned_to_depth) * dev->get_stream_height(rs::stream::color_aligned_to_depth);
        assert(dev->get_stream_width(rs::stream::depth_aligned_to_color)*dev->get_stream_height(rs::stream::depth_aligned_to_color) == nPixel);
        assert(dev->get_stream_width(rs::stream::depth)*dev->get_stream_height(rs::stream::depth) == nPixel);
        assert(dev->get_stream_width(rs::stream::color)*dev->get_stream_height(rs::stream::color) == nPixel);
        while(true) {
            dev->wait_for_frames();
            std::chrono::system_clock::duration epoch_dur = std::chrono::system_clock::now().time_since_epoch();
            lastAllFrameTime =  epoch_dur / std::chrono::milliseconds(1);
            int bufferIndex = (latestAllFrameIndex.getValue() + 1) % numBuffers;
            memcpy(depthBuffers[bufferIndex].first, dev->get_frame_data(rs::stream::depth), nPixel * 2);
            memcpy(rgbBuffers[bufferIndex].first, dev->get_frame_data(rs::stream::color), nPixel * 3);
            memcpy(depthAlignedToRgbBuffers[bufferIndex].first, dev->get_frame_data(rs::stream::depth_aligned_to_color), nPixel * 2);
            memcpy(rgbAlginedToDepthBuffers[bufferIndex].first, dev->get_frame_data(rs::stream::color_aligned_to_depth), nPixel * 3);
            depthBuffers[bufferIndex].second = lastAllFrameTime;
            rgbBuffers[bufferIndex].second = lastAllFrameTime;
            depthAlignedToRgbBuffers[bufferIndex].second = lastAllFrameTime;
            rgbAlginedToDepthBuffers[bufferIndex].second = lastAllFrameTime;
            cv::Mat cvRgbImageMat = cv::Mat(height, width, CV_8UC3, rgbBuffers[bufferIndex].first, cv::Mat::AUTO_STEP);
            cv::Mat cvDepthImageMat = cv::Mat(height, width, CV_8UC3, rgbBuffers[bufferIndex].first, cv::Mat::AUTO_STEP);
            cv::Mat m = pSLAM->TrackRGBD(cvRgbImageMat, cvDepthImageMat, lastAllFrameTime/1000.);
            Eigen::Matrix4f transMat;
            transMat << m.at<float>(0, 0), m.at<float>(0, 1), m.at<float>(0, 2), m.at<float>(0, 3),
                    m.at<float>(1, 0), m.at<float>(1, 1), m.at<float>(1, 2), m.at<float>(1, 3),
                    m.at<float>(2, 0), m.at<float>(2, 1), m.at<float>(2, 2), m.at<float>(2, 3),
                    m.at<float>(3, 0), m.at<float>(3, 1), m.at<float>(3, 2), m.at<float>(3, 3);
//    Eigen::Matrix4f invTransMat = transMat.inverse();
//    for (int i = 0; i < 4; i++) {
//        for (int j = 0; j < 4; j++) {
//            printf("%f ", invTransMat(i, j));
//        }
//        printf("\n");
//    }
//    printf("\n");
//    fflush(stdout);
            cameraToObjectTransMatBuffers[bufferIndex].first = transMat.inverse();
//    cameraToObjectTransMatBuffers[bufferIndex].first = transMat;
            cameraToObjectTransMatBuffers[bufferIndex].second = lastAllFrameTime;
            latestAllFrameIndex++;
            usleep(10000);
        }
    };
    allFrameDaemonThread = std::move(std::thread(all_stream_feeder));
    dev->start();
}

SR300_ORB_Interface::~SR300_ORB_Interface()
{
    if(initSuccessful)
    {
        printf("SR300_ORB_Interface dead.");
        dev->stop();
        dev->disable_stream(rs::stream::depth);
        dev->disable_stream(rs::stream::color);
        dev->disable_stream(rs::stream::infrared);
        try { dev->disable_stream(rs::stream::infrared2);}
        catch(...) { printf("Device does not provide infrared2 stream.\n"); }
        for(int i = 0; i < numBuffers; i++) {
            free(rgbBuffers[i].first);
        }
        for(int i = 0; i < numBuffers; i++) {
            free(frameBuffers[i].first.first);
            free(frameBuffers[i].first.second);
        }
        delete pSLAM;
    }
}

bool SR300_ORB_Interface::findMode(int x, int y, int fps)
{
    return true;
}

void SR300_ORB_Interface::printModes()
{
//    const openni::Array<openni::VideoMode> & depthModes = depthStream.getSensorInfo().getSupportedVideoModes();
//
//    openni::VideoMode currentDepth = depthStream.getVideoMode();
//
//    std::cout << "Depth Modes: (" << currentDepth.getResolutionX() <<
//                                     "x" <<
//                                     currentDepth.getResolutionY() <<
//                                     " @ " <<
//                                     currentDepth.getFps() <<
//                                     "fps " <<
//                                     formatMap[currentDepth.getPixelFormat()] << ")" << std::endl;
//
//    for(int i = 0; i < depthModes.getSize(); i++)
//    {
//        std::cout << depthModes[i].getResolutionX() <<
//                     "x" <<
//                     depthModes[i].getResolutionY() <<
//                     " @ " <<
//                     depthModes[i].getFps() <<
//                     "fps " <<
//                     formatMap[depthModes[i].getPixelFormat()] << std::endl;
//    }
//
//    const openni::Array<openni::VideoMode> & rgbModes = rgbStream.getSensorInfo().getSupportedVideoModes();
//
//    openni::VideoMode currentRGB = depthStream.getVideoMode();
//
//    std::cout << "RGB Modes: (" << currentRGB.getResolutionX() <<
//                                   "x" <<
//                                   currentRGB.getResolutionY() <<
//                                   " @ " <<
//                                   currentRGB.getFps() <<
//                                   "fps " <<
//                                   formatMap[currentRGB.getPixelFormat()] << ")" << std::endl;
//
//    for(int i = 0; i < rgbModes.getSize(); i++)
//    {
//        std::cout << rgbModes[i].getResolutionX() <<
//                     "x" <<
//                     rgbModes[i].getResolutionY() <<
//                     " @ " <<
//                     rgbModes[i].getFps() <<
//                     "fps " <<
//                     formatMap[rgbModes[i].getPixelFormat()] << std::endl;
//    }
}

void SR300_ORB_Interface::setAutoExposure(bool value)
{
//    rgbStream.getCameraSettings()->setAutoExposureEnabled(value);
}

void SR300_ORB_Interface::setAutoWhiteBalance(bool value)
{
//    rgbStream.getCameraSettings()->setAutoWhiteBalanceEnabled(value);
}

bool SR300_ORB_Interface::getAutoExposure()
{
//    return rgbStream.getCameraSettings()->getAutoExposureEnabled();
    return true;
}

bool SR300_ORB_Interface::getAutoWhiteBalance()
{
//    return rgbStream.getCameraSettings()->getAutoWhiteBalanceEnabled();
    return true;
}
