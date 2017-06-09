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

#include "LiveLogReaderSR300_ORB.h"

LiveLogReaderSR300_ORB::LiveLogReaderSR300_ORB(std::string file, bool flipColors)
 : LogReader(file, flipColors),
   lastFrameTime(-1),
   lastGot(-1)
{
    std::cout << "Creating live capture... "; std::cout.flush();

	asus = new SR300_ORB_Interface(Resolution::getInstance().width(), Resolution::getInstance().height());

	decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];

	decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];

    if(!asus->ok()) {
        std::cout << "failed!" << std::endl;
        std::cout << asus->error();
    }
    else {
        std::cout << "success!" << std::endl;

        std::cout << "Waiting for first frame"; std::cout.flush();

        int lastDepth = asus->latestDepthIndex.getValue();

        do {
            usleep(33333);
            std::cout << "."; std::cout.flush();
            lastDepth = asus->latestAllFrameIndex.getValue();
//            lastDepth = asus->latestDepthIndex.getValue();
        } while(lastDepth == -1);

        std::cout << " got it!" << std::endl;
    }
}

LiveLogReaderSR300_ORB::~LiveLogReaderSR300_ORB()
{
    delete [] decompressionBufferDepth;

    delete [] decompressionBufferImage;

	delete asus;
}

void LiveLogReaderSR300_ORB::getNext()
{
//    int lastDepth = asus->latestDepthIndex.getValue();
    int lastDepth = asus->latestAllFrameIndex.getValue();

    assert(lastDepth != -1);

    int bufferIndex = lastDepth % SR300_ORB_Interface::numBuffers;

    if(bufferIndex == lastGot)
    {
        return;
    }

    if(lastFrameTime == asus->depthBuffers[bufferIndex].second)
//        if(lastFrameTime == asus->frameBuffers[bufferIndex].second)
    {
        return;
    }

    memcpy(&decompressionBufferDepth[0], asus->depthAlignedToRgbBuffers[bufferIndex].first, Resolution::getInstance().numPixels() * 2);
//    memcpy(&decompressionBufferDepth[0], asus->depthBuffers[bufferIndex].first, Resolution::getInstance().numPixels() * 2);
//    memcpy(&decompressionBufferDepth[0], asus->frameBuffers[bufferIndex].first.first, Resolution::getInstance().numPixels() * 2);
    memcpy(&decompressionBufferImage[0], asus->rgbBuffers[bufferIndex].first, Resolution::getInstance().numPixels() * 3);
//    memcpy(&decompressionBufferImage[0], asus->rgbAlginedToDepthBuffers[bufferIndex].first, Resolution::getInstance().numPixels() * 3);
//    memcpy(&decompressionBufferImage[0], asus->frameBuffers[bufferIndex].first.second, Resolution::getInstance().numPixels() * 3);

    lastFrameTime = asus->depthBuffers[bufferIndex].second;
//    lastFrameTime = asus->frameBuffers[bufferIndex].second;

    timestamp = lastFrameTime;

    rgb = (unsigned char *)&decompressionBufferImage[0];
    depth = (unsigned short *)&decompressionBufferDepth[0];

    currentPose = asus->cameraToObjectTransMatBuffers[bufferIndex].first;

    imageReadBuffer = 0;
    depthReadBuffer = 0;

    imageSize = Resolution::getInstance().numPixels() * 3;
    depthSize = Resolution::getInstance().numPixels() * 2;

    if(flipColors)
    {
        for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }
}

const std::string LiveLogReaderSR300_ORB::getFile()
{
    return Parse::get().baseDir().append("live");
}

int LiveLogReaderSR300_ORB::getNumFrames()
{
    return std::numeric_limits<int>::max();
}

bool LiveLogReaderSR300_ORB::hasMore()
{
    return true;
}

void LiveLogReaderSR300_ORB::setAuto(bool value)
{
//    asus->setAutoExposure(value);
//    asus->setAutoWhiteBalance(value);
}
