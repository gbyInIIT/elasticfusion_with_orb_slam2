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

#include "LiveLogReaderRos.h"

LiveLogReaderRos::LiveLogReaderRos(std::string file, bool flipColors, RosInterface * asusIn)
 : LogReader(file, flipColors),
   lastFrameTime(-1),
   lastGot(-1)
{
    std::cout << "Creating live capture... "; std::cout.flush();

	asus = asusIn;
//    asus = new RosInterface();

	decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];

	decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];

    if(!asus->ok()) {
        std::cout << "failed!" << std::endl;
    }
    else {
        std::cout << "success!" << std::endl;

        std::cout << "Waiting for first frame"; std::cout.flush();

        int lastDepth = asus->latestAllFrameIndex.getValue();

        do {
            usleep(33333);
            std::cout << "."; std::cout.flush();
            lastDepth = asus->latestAllFrameIndex.getValue();
//            lastDepth = asus->latestDepthIndex.getValue();
        } while(lastDepth == -1);

        std::cout << " got it!" << std::endl;
    }
}

LiveLogReaderRos::~LiveLogReaderRos()
{
    delete [] decompressionBufferDepth;

    delete [] decompressionBufferImage;

	delete asus;
}

void LiveLogReaderRos::getNext()
{
    int lastDepth = asus->latestAllFrameIndex.getValue();

    assert(lastDepth != -1);

    int bufferIndex = lastDepth % RosInterface::numBuffers;

    if(bufferIndex == lastGot)
    {
        return;
    }

    if(lastFrameTime == asus->depthBuffers[bufferIndex].second)
    {
        return;
    }

    memcpy(&decompressionBufferDepth[0], asus->depthBuffers[bufferIndex].first, Resolution::getInstance().numPixels() * 2);
    memcpy(&decompressionBufferImage[0], asus->rgbBuffers[bufferIndex].first, Resolution::getInstance().numPixels() * 3);

    lastFrameTime = asus->depthBuffers[bufferIndex].second;

    timestamp = lastFrameTime;

    rgb = (unsigned char *)&decompressionBufferImage[0];
    depth = (unsigned short *)&decompressionBufferDepth[0];

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

const std::string LiveLogReaderRos::getFile()
{
    return Parse::get().baseDir().append("live");
}

int LiveLogReaderRos::getNumFrames()
{
    return std::numeric_limits<int>::max();
}

bool LiveLogReaderRos::hasMore()
{
    return true;
}

void LiveLogReaderRos::setAuto(bool value)
{
//    asus->setAutoExposure(value);
//    asus->setAutoWhiteBalance(value);
}
