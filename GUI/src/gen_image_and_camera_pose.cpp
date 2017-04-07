#include "Tools/RosInterface.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "third_party/stb_image_write.h"
#include <signal.h>
#include "Tools/ThreadMutexObject.h"
//#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

ThreadMutexObject<bool> isRosOk;
ThreadMutexObject<bool> isRuningDataDaemon;//isDisplayImage;

void mySigintHandler(int sig)
{
    fprintf(stderr, "\nCtrl-C pressed.\n");
    isRosOk.assign(false);
    isRuningDataDaemon.assign(false);
}
int main(int argc, char * argv[])
{
    signal(SIGINT, mySigintHandler);

    RosInterface * asus = new RosInterface();

    ThreadMutexObject<int> saveImageNumber(0);

    isRosOk.assign(true);
    isRuningDataDaemon.assign(true);

    auto saveImageAction = [asus, &saveImageNumber]() {
        FILE * pfile = fopen("filename_and_xyzabc.csv", "w");
        int lastSaveAllFrameIndex = -1;
        while (isRuningDataDaemon.getValue()) {
            const int copyAllFrameIndex = asus->latestAllFrameIndex.getValue();
            if (copyAllFrameIndex != lastSaveAllFrameIndex) {
                const int bufferIndex = copyAllFrameIndex % asus->numBuffers;
                std::stringstream ss;
                ss << "slam_" << saveImageNumber.getValue() << ".png";
                stbi_write_png(ss.str().data(),
                               asus->width,
                               asus->height,
                               3,
                               asus->rgbBuffers[bufferIndex].first,
                               asus->width * 3);
                saveImageNumber.assign(saveImageNumber.getValue() + 1);

                Eigen::Matrix4f cameraToObjectTransMat = asus->cameraToObjectTransMatBuffers[bufferIndex].first;
                tf::Matrix3x3 cameraToObjectRotMat(cameraToObjectTransMat(0, 0), cameraToObjectTransMat(0, 1), cameraToObjectTransMat(0, 2),
                                                   cameraToObjectTransMat(1, 0), cameraToObjectTransMat(1, 1), cameraToObjectTransMat(1, 2),
                                                   cameraToObjectTransMat(2, 0), cameraToObjectTransMat(2, 1), cameraToObjectTransMat(2, 2));
                tfScalar x = cameraToObjectTransMat(0, 3);
                tfScalar y = cameraToObjectTransMat(1, 3);
                tfScalar z = cameraToObjectTransMat(2, 3);
                tfScalar a, b, c;
                cameraToObjectRotMat.getEulerYPR(a, b, c);
                fprintf(pfile, "%s,%e,%e,%e,%e,%e,%e\n", ss.str().data(), x, y, z, a, b, c);
                lastSaveAllFrameIndex = copyAllFrameIndex;
            }
            usleep(1000);
        }
        fclose(pfile);
    };
    std::thread saveImageDaemonThread(saveImageAction);
    while (isRosOk.getValue()) {
        usleep(10000);
    }
    saveImageDaemonThread.join();
    return 0;
}
