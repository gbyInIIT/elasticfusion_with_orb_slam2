#include "LiveLogReaderSR300_ORB.h"

LiveLogReaderSR300_ORB::LiveLogReaderSR300_ORB(std::string file, bool flipColors,
                                               std::string argInDepthCameraYamlPath,
                                               std::string argInOrbVocBinPath)
 : LogReader(file, flipColors),
   lastFrameTime(-1),
   lastGot(-1)
{
    std::cout << "Creating live capture... "; std::cout.flush();
	realsense = new SR300_ORB_Interface(Resolution::getInstance().width(), Resolution::getInstance().height(), 30, argInDepthCameraYamlPath,  argInOrbVocBinPath);
	decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];

	decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];

    if(!realsense->ok()) {
        std::cout << "failed!" << std::endl;
        std::cout << realsense->error();
    }
    else {
        std::cout << "success!" << std::endl;

        std::cout << "Waiting for first frame"; std::cout.flush();

        int lastDepth = realsense->latestAllFrameIndex.getValue();

        do {
            usleep(33333);
            std::cout << "."; std::cout.flush();
            lastDepth = realsense->latestAllFrameIndex.getValue();
        } while(lastDepth == -1);

        std::cout << " got it!" << std::endl;
    }
}

LiveLogReaderSR300_ORB::~LiveLogReaderSR300_ORB()
{
    delete [] decompressionBufferDepth;

    delete [] decompressionBufferImage;

	delete realsense;
}

void LiveLogReaderSR300_ORB::getNext()
{
    int lastDepth = realsense->latestAllFrameIndex.getValue();

    assert(lastDepth != -1);

    int bufferIndex = lastDepth % SR300_ORB_Interface::numBuffers;

    if(bufferIndex == lastGot)
    {
        return;
    }

    if(lastFrameTime == realsense->depthAlignedToRgbBuffers[bufferIndex].second)
    {
        return;
    }

    memcpy(&decompressionBufferDepth[0], realsense->depthAlignedToRgbBuffers[bufferIndex].first, Resolution::getInstance().numPixels() * 2);
    memcpy(&decompressionBufferImage[0], realsense->rgbBuffers[bufferIndex].first, Resolution::getInstance().numPixels() * 3);

    lastFrameTime = realsense->depthAlignedToRgbBuffers[bufferIndex].second;

    timestamp = lastFrameTime;

    rgb = (unsigned char *)&decompressionBufferImage[0];
    depth = (unsigned short *)&decompressionBufferDepth[0];

    currentPose = realsense->cameraToObjectTransMatBuffers[bufferIndex].first;

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
}
