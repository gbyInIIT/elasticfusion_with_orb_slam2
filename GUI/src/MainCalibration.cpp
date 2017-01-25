#include "Tools/RosInterface.h"
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
int main(int argc, char * argv[])
{
    RosInterface * asus = new RosInterface();
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle node;
    tf::TransformListener listener(ros::Duration(100.0));
    cv::namedWindow( "RGB", cv::WINDOW_AUTOSIZE );
    cv::namedWindow( "Depth", cv::WINDOW_AUTOSIZE );
    ros::Rate rate(30);
    while(node.ok()) {
        tf::StampedTransform transform;
        const int imageIdx = asus->latestAllFrameIndex.getValue();
        if (imageIdx >= 0) {
            const int buffIdx = imageIdx % asus->numBuffers;
            ros::Time rgbImageRosTime = asus->rgbImageRosTimeBuffers[buffIdx];
            try{
                ros::Duration timeout(300);
//                listener.waitForTransform("iiwa_base", "flange", rgbImageRosTime, timeout);
                listener.lookupTransform("iiwa_base", "flange", rgbImageRosTime, transform);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
//                ros::Duration(1.0).sleep();
                rate.sleep();
                continue;
            }
            tf::Quaternion q = transform.getRotation();
            printf("x=%f, y=%f, z=%f, w=%f\n", q.getX(), q.getY(), q.getZ(), q.getW());
//            cv::Size size(asus->width, asus->height);
//            cv::Mat rgbImage(size, CV_8UC3, asus->rgbBuffers[buffIdx].first);
//            cv::Mat bgrImage;
//            cv::cvtColor(rgbImage, bgrImage, CV_RGB2BGR);
//            cv::imshow( "RGB", bgrImage);
//            cv::waitKey(3);
//            cv::Mat depthImage(size, CV_16UC1, asus->depthBuffers[buffIdx].first);
//            cv::Mat grayDepthImage;
//            cv::cvtColor(depthImage, grayDepthImage, CV_GRAY2BGR);
//            cv::imshow( "Depth", grayDepthImage);
//            cv::waitKey(3);
        }
        rate.sleep();
    }
    return 0;
}
