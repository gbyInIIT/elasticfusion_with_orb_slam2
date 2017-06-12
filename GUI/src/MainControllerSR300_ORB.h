#include <ElasticFusion.h>
#include <Utils/Parse.h>

#include "Tools/GUI.h"
#include "Tools/GroundTruthOdometry.h"
#include "Tools/RawLogReader.h"
#include "Tools/LiveLogReader.h"
#include "Tools/LiveLogReaderSR300_ORB.h"

#ifndef MAINCONTROLLER_SR300_ORB_H_
#define MAINCONTROLLER_SR300_ORB_H_

class MainControllerSR300_ORB
{
    public:
        MainControllerSR300_ORB(int argc, char * argv[]);
        virtual ~MainControllerSR300_ORB();

        void launch();

    private:
        void run();

        void loadCalibration(const std::string & filename);

        bool good;
        ElasticFusion * eFusion;
        GUI * gui;
        GroundTruthOdometry * groundTruthOdometry;
        LogReader * logReader;

        bool iclnuim;
        std::string logFile;
        std::string poseFile;

        float confidence,
              depth,
              icp,
              icpErrThresh,
              covThresh,
              photoThresh,
              fernThresh;

        int timeDelta,
            icpCountThresh,
            start,
            end;

        bool fillIn,
             openLoop,
             reloc,
             frameskip,
             quiet,
             fastOdom,
             so3,
             rewind,
             frameToFrameRGB;

        int framesToSkip;
        bool streaming;
        bool resetButton;

        Resize * resizeStream;
};

#endif /* MAINCONTROLLER_H_ */
