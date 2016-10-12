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

#include "SR300Interface.h"

SR300Interface::SR300Interface(int inWidth, int inHeight, int fps)
 : width(inWidth),
   height(inHeight),
   fps(fps),
   initSuccessful(true)
{
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
    latestDepthIndex.assign(-1);
    latestRgbIndex.assign(-1);
    for(int i = 0; i < numBuffers; i++) {
        uint8_t * newImage = (uint8_t *)calloc(width * height * 3, sizeof(uint8_t));
        rgbBuffers[i] = std::pair<uint8_t *, int64_t>(newImage, 0);
    }
    for(int i = 0; i < numBuffers; i++) {
        uint8_t * newDepth = (uint8_t *)calloc(width * height * 2, sizeof(uint8_t));
        uint8_t * newImage = (uint8_t *)calloc(width * height * 3, sizeof(uint8_t));
        frameBuffers[i] = std::pair<std::pair<uint8_t *, uint8_t *>, int64_t>(std::pair<uint8_t *, uint8_t *>(newDepth, newImage), 0);
    }
    auto depth_callback = [this](rs::frame frame) {
        lastDepthTime = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        int bufferIndex = (latestDepthIndex.getValue() + 1) % numBuffers;
        memcpy(frameBuffers[bufferIndex].first.first, frame.get_data(), frame.get_width() * frame.get_height() * 2);
        frameBuffers[bufferIndex].second = lastDepthTime;
        int lastImageVal = latestRgbIndex.getValue();
        if(lastImageVal == -1) {
            return;
        }
        lastImageVal %= numBuffers;
        memcpy(frameBuffers[bufferIndex].first.second, rgbBuffers[lastImageVal].first, frame.get_width() * frame.get_height() * 3);
        latestDepthIndex++;
    };
    dev->set_frame_callback(rs::stream::depth, depth_callback);
    auto color_callback = [this](rs::frame frame) {
        lastRgbTime = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        int bufferIndex = (latestRgbIndex.getValue() + 1) % numBuffers;
        memcpy(rgbBuffers[bufferIndex].first, frame.get_data(), frame.get_width() * frame.get_height() * 3);
        rgbBuffers[bufferIndex].second = lastRgbTime;
        latestRgbIndex++;
    };
    dev->set_frame_callback(rs::stream::color, color_callback);
    dev->start();
}

SR300Interface::~SR300Interface()
{
    if(initSuccessful)
    {
        printf("SR300Interface dead.");
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
    }
}

bool SR300Interface::findMode(int x, int y, int fps)
{
    return true;
}

void SR300Interface::printModes()
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

void SR300Interface::setAutoExposure(bool value)
{
//    rgbStream.getCameraSettings()->setAutoExposureEnabled(value);
}

void SR300Interface::setAutoWhiteBalance(bool value)
{
//    rgbStream.getCameraSettings()->setAutoWhiteBalanceEnabled(value);
}

bool SR300Interface::getAutoExposure()
{
//    return rgbStream.getCameraSettings()->getAutoExposureEnabled();
    return true;
}

bool SR300Interface::getAutoWhiteBalance()
{
//    return rgbStream.getCameraSettings()->getAutoWhiteBalanceEnabled();
    return true;
}
