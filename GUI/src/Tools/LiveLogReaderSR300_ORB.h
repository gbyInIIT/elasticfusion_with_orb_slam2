#ifndef LIVELOGREADERSR300_ORB_H_
#define LIVELOGREADERSR300_ORB_H_

#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <signal.h>

#include <Utils/Parse.h>

#include "LogReader.h"
#include "SR300_ORB_Interface.h"

class LiveLogReaderSR300_ORB : public LogReader
{
	public:
    LiveLogReaderSR300_ORB(std::string file, bool flipColors,
                                                   std::string argInDepthCameraYamlPath,
                                                   std::string argInOrbVocBinPath);

		virtual ~LiveLogReaderSR300_ORB();

        void getNext();

        int getNumFrames();

        bool hasMore();

        bool rewound()
        {
            return false;
        }

        void rewind()
        {

        }

        void getBack()
        {

        }

        void fastForward(int frame)
        {

        }

        const std::string getFile();

        void setAuto(bool value);

		SR300_ORB_Interface * realsense;
        Eigen::Matrix4f currentPose;

	private:
		int64_t lastFrameTime;
		int lastGot;
};

#endif /* LIVELOGREADERSR300_H_ */
