#include "Tools/RosInterface.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "third_party/stb_image_write.h"
#include <signal.h>
#include "Tools/ThreadMutexObject.h"
//#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <iomanip>

ThreadMutexObject<bool> isRosOk;
ThreadMutexObject<bool> isRuningDataDaemon;//isDisplayImage;

void mySigintHandler(int sig)
{
    fprintf(stderr, "\nCtrl-Z pressed.\n");
    fflush(stderr);
    isRosOk.assign(false);
    isRuningDataDaemon.assign(false);
}
int main(int argc, char * argv[])
{
//    signal(SIGINT, mySigintHandler);

    RosInterface * asus = new RosInterface();

    ThreadMutexObject<int> saveImageNumber(0);

    isRosOk.assign(true);
    isRuningDataDaemon.assign(true);

    printf("Save image daemon running...\n");
    fflush(stdout);
    signal(SIGTSTP, mySigintHandler);
    FILE * pfile = fopen("filename_and_xyzabc.csv", "w");
    int lastSaveAllFrameIndex = -1;
    auto saveImageAction = [](std::string fileName, const int width, const int height, const void * data) {
        stbi_write_png(fileName.c_str(), width, height, 3, data, width * 3);
    };
    while (isRuningDataDaemon.getValue()) {
        const int copyAllFrameIndex = asus->latestAllFrameIndex.getValue();
        if (copyAllFrameIndex != lastSaveAllFrameIndex) {
            const int bufferIndex = copyAllFrameIndex % asus->numBuffers;
            std::stringstream ss;
            ss << "slam_" << setw(6) << setfill('0') << saveImageNumber.getValue() << ".png";
            std::string fileName = ss.str();
            std::thread saveImageThread (saveImageAction, fileName, asus->width, asus->height, asus->rgbBuffers[bufferIndex].first);
            saveImageThread.detach();
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
            fprintf(pfile, "%s,%e,%e,%e,%e,%e,%e\n", fileName.c_str(), x, y, z, a, b, c);
            lastSaveAllFrameIndex = copyAllFrameIndex;
        }
        usleep(1000);
    }
    fclose(pfile);
    printf("Save image daemon running done.\n");
    fflush(stdout);
    printf("Waiting for detached image saving thread to terminate.\n");
    fflush(stdout);
    usleep(3000000);
    printf("Waiting for detached image saving thread to terminate done.\n");
    fflush(stdout);
    delete asus;
    return 0;
}
