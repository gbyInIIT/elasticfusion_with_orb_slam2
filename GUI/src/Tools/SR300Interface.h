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

#ifndef INTERFACESR300_H_
#define INTERFACESR300_H_

#include <string>
#include <iostream>
#include <algorithm>
#include <map>

#include "ThreadMutexObject.h"
#include <librealsense/rs.hpp>

class SR300Interface
{
    public:
        SR300Interface(int inWidth = 640, int inHeight = 480, int fps = 30);
        virtual ~SR300Interface();

        const int width, height, fps;

        void printModes();
        bool findMode(int x, int y, int fps);
        void setAutoExposure(bool value);
        void setAutoWhiteBalance(bool value);
        bool getAutoExposure();
        bool getAutoWhiteBalance();

        bool ok()
        {
            return initSuccessful;
        }

        std::string error()
        {
            errorText.erase(std::remove_if(errorText.begin(), errorText.end(), &SR300Interface::isTab), errorText.end());
            return errorText;
        }

        static const int numBuffers = 10;
        ThreadMutexObject<int> latestDepthIndex;
        std::pair<std::pair<uint8_t *, uint8_t *>, int64_t> frameBuffers[numBuffers];

    private:
        rs::context ctx;
        rs::device * dev;

        int64_t lastRgbTime;
        int64_t lastDepthTime;

        ThreadMutexObject<int> latestRgbIndex;
        std::pair<uint8_t *, int64_t> rgbBuffers[numBuffers];

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
