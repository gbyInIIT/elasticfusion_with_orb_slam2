#include "Tools/RosInterface.h"
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char * argv[])
{
    RosInterface * asus = new RosInterface();
    cv::namedWindow( "RGB", cv::WINDOW_AUTOSIZE );
    cv::namedWindow( "Depth", cv::WINDOW_AUTOSIZE );
    while(asus->isSystemRunning.getValue()) {
        const int imageIdx = asus->latestAllFrameIndex.getValue();
        if (imageIdx >= 0) {
            const int buffIdx = imageIdx % asus->numBuffers;
            cv::Size size(asus->width, asus->height);
            cv::Mat rgbImage(size, CV_8UC3, asus->rgbBuffers[buffIdx].first);
            cv::Mat bgrImage;
            cv::cvtColor(rgbImage, bgrImage, CV_RGB2BGR);
            cv::imshow( "RGB", bgrImage);
            cv::waitKey(3);
            cv::Mat depthImage(size, CV_16UC1, asus->depthBuffers[buffIdx].first);
            cv::Mat grayDepthImage;
            cv::cvtColor(depthImage, grayDepthImage, CV_GRAY2BGR);
            cv::imshow( "Depth", grayDepthImage);
            cv::waitKey(30);
        }
    }
    return 0;
}
