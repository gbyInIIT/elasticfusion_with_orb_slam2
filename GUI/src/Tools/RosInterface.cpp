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

#include "RosInterface.h"
#include <unistd.h>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
//#include <turtlesim/Pose.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../third_party/stb_image_write.h"

//RosInterface::RosInterface(std::string argInDepthCameraYamlPath = std::string("/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Realsense_SR300.yaml"))
RosInterface::RosInterface(std::string argInDepthCameraYamlPath)// = std::string("/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Realsense_SR300.yaml"))
 : width(-1),
   height(-1),
   fps(30),
   initSuccessful(true)
{
    depthCameraConfigYamlPath = argInDepthCameraYamlPath;
    latestAllFrameIndex.assign(-1);
    isCameraInitialized.assign(false);
    isSystemRunning.assign(true);
    latestPointCloudIndex.assign(-1);
    for (int i = 0; i < nPointCloudMsgBuffer; i++) {
        sensor_msgs::PointCloud2 & pointCloudMsg = pointCloudMsgBuffer[i];
        sensor_msgs::PointField x;
        x.name = "x";
        x.offset = 0;
        x.datatype = sensor_msgs::PointField::FLOAT32;
        x.count = 1;
        sensor_msgs::PointField y;
        y.name = "y";
        y.offset = 4;
        y.datatype = sensor_msgs::PointField::FLOAT32;
        y.count = 1;
        sensor_msgs::PointField z;
        z.name = "z";
        z.offset = 8;
        z.datatype = sensor_msgs::PointField::FLOAT32;
        z.count = 1;
        sensor_msgs::PointField rgb;
        rgb.name = "rgb";
        rgb.offset = 12;
        rgb.datatype = sensor_msgs::PointField::UINT32;
        rgb.count = 1;
//        sensor_msgs::PointField normal_x;
//        normal_x.name = "normal_x";
////        normal_x.name = "nx";
//        normal_x.offset = 16;
//        normal_x.datatype = sensor_msgs::PointField::FLOAT32;
//        normal_x.count = 1;
//        sensor_msgs::PointField normal_y;
//        normal_y.name = "normal_y";
////        normal_y.name = "ny";
//        normal_y.offset = 20;
//        normal_y.datatype = sensor_msgs::PointField::FLOAT32;
//        normal_y.count = 1;
//        sensor_msgs::PointField normal_z;
//        normal_z.name = "normal_z";
////        normal_z.name = "nz";
//        normal_z.offset = 24;
//        normal_z.datatype = sensor_msgs::PointField::FLOAT32;
//        normal_z.count = 1;
        pointCloudMsg.fields = {x, y, z, rgb};
//        pointCloudMsg.fields = {x, y, z, rgb, normal_x, normal_y, normal_z};
        pointCloudMsg.is_bigendian = false;// default is false
        pointCloudMsg.point_step = 16;// 4 + 4 + 4 + 4
//        pointCloudMsg.point_step = 28;// 4 + 4 + 4 + 4 + 4 + 4 + 4
        pointCloudMsg.is_dense = false;// default false maybe it is safer even if there is no invalid point in the data
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    pSLAM = new ORB_SLAM2::System("/home/gao/Downloads/ORB_SLAM2/Vocabulary/ORBvoc.bin",
                                  depthCameraConfigYamlPath, ORB_SLAM2::System::RGBD,false);
//                           "/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Realsense_SR300.yaml", ORB_SLAM2::System::RGBD,false);
//                                "/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Realsense_R200.yaml", ORB_SLAM2::System::RGBD,false);
//                           "/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Asus.yaml", ORB_SLAM2::System::RGBD,false);
//                                  "/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Arrfou.yaml", ORB_SLAM2::System::RGBD,false);
    auto rosAction = [this]() {
        isRosOk.assign(true);
        int argc = 0;
        ros::init(argc, NULL, "depth_rgb_elasticfusion_sub");
        ros::start();
        ros::NodeHandle nh;
        message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
        message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
        message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
        sync.registerCallback(boost::bind(&RosInterface::depthRgbMsgCallback, this, _1,_2));
        ros::Subscriber sub = nh.subscribe("camera/camera_info", 10, &RosInterface::cameraInfoCallback, this);
        ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("elasticfusion/point_cloud", 10);
        ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>("orb_slam2/camera_pose", 10);
        int previousPointCloudIndex = -1;
        tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0, 0, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        ros::Rate rate(30);
        while(ros::ok() && isSystemRunning.getValue()) {
            ros::spinOnce();
            ros::Time current_time = ros::Time::now();
            const int currentPointCloudIndex = latestPointCloudIndex.getValue();
            if (currentPointCloudIndex != previousPointCloudIndex) {
                previousPointCloudIndex = currentPointCloudIndex;
                int bufferIndex = currentPointCloudIndex % nPointCloudMsgBuffer;
//                br.sendTransform(tf::StampedTransform(transform, current_time, "iiwa_base", "elasticfusion_point_cloud"));
                pub.publish(pointCloudMsgBuffer[bufferIndex]);
            }
            if (latestAllFrameIndex.getValue() >= 0) {
                const int bufferIndex = latestAllFrameIndex.getValue() % numBuffers;
                Eigen::Matrix4f cameraToObjectTransMat = cameraToObjectTransMatBuffers[bufferIndex].first;
                ros::Time imageRosTime = rgbImageRosTimeBuffers[bufferIndex];
                tf::Matrix3x3 cameraToObjectRotMat(cameraToObjectTransMat(0, 0), cameraToObjectTransMat(0, 1), cameraToObjectTransMat(0, 2),
                                                  cameraToObjectTransMat(1, 0), cameraToObjectTransMat(1, 1), cameraToObjectTransMat(1, 2),
                                                  cameraToObjectTransMat(2, 0), cameraToObjectTransMat(2, 1), cameraToObjectTransMat(2, 2));
                tf::Vector3 cameraToWorldTranslationVec(cameraToObjectTransMat(0, 3), cameraToObjectTransMat(1, 3), cameraToObjectTransMat(2, 3));
                tf::Transform cameraToCloudTrans(cameraToObjectRotMat, cameraToWorldTranslationVec);
//                br.sendTransform(tf::StampedTransform(cameraToCloudTrans, current_time, "iiwa_base", "realsense_camera"));
                geometry_msgs::Pose cameraPose;
                geometry_msgs::Point cameraPosePoint;
                geometry_msgs::Quaternion cameraPoseQuaternion;
                cameraPosePoint.x = cameraToWorldTranslationVec.x();
                cameraPosePoint.y = cameraToWorldTranslationVec.y();
                cameraPosePoint.z = cameraToWorldTranslationVec.z();
                tf::Quaternion tmpQ;
                cameraToObjectRotMat.getRotation(tmpQ);
//                tf::Matrix3x3 cameraToCloudRotMatInv = cameraToObjectRotMat.inverse();
//                cameraToCloudRotMatInv.getRotation(tmpQ);
                cameraPoseQuaternion.x = tmpQ.x();
                cameraPoseQuaternion.y = tmpQ.y();
                cameraPoseQuaternion.z = tmpQ.z();
                cameraPoseQuaternion.w = tmpQ.w();
                cameraPose.position = cameraPosePoint;
                cameraPose.orientation = cameraPoseQuaternion;
                geometry_msgs::PoseStamped cameraPoseStamped;
                cameraPoseStamped.header.frame_id = "world";
                cameraPoseStamped.header.stamp = imageRosTime;
                cameraPoseStamped.header.seq = 0;
                cameraPoseStamped.pose = cameraPose;
                posePub.publish(cameraPoseStamped);
            }
//            usleep(100);
            rate.sleep();
        }
        printf("ROS exited.\n");
        fflush(stdout);
        isRuningDataDaemon.assign(false);
        while(isRosOk.getValue()) {
            usleep(1000);
        }
        pSLAM->Shutdown();
        ros::shutdown();
    };
    rosThread = std::move(std::thread(rosAction));
    while (!isCameraInitialized.getValue()) {
        usleep(10000);
    }
    auto saveImageDaemonAction = [this]() {
        ThreadMutexObject<int> saveImageNumber(0);

        isRuningDataDaemon.assign(true);

        printf("Save image daemon running...\n");
        fflush(stdout);
//        signal(SIGTSTP, mySigintHandler);
        FILE * pfile = fopen("filename_and_xyzabc.csv", "w");
        int lastSaveAllFrameIndex = -1;
        auto saveImageAction = [](std::string fileName, const int width, const int height, const void * data) {
            stbi_write_png(fileName.c_str(), width, height, 3, data, width * 3);
        };
        while (isRuningDataDaemon.getValue()) {
            const int copyAllFrameIndex = this->latestAllFrameIndex.getValue();
            if (copyAllFrameIndex != lastSaveAllFrameIndex) {
                const int bufferIndex = copyAllFrameIndex % this->numBuffers;
                std::stringstream ss;
                ss << "slam_" << setw(6) << setfill('0') << saveImageNumber.getValue() << ".png";
                std::string fileName = ss.str();
                std::thread saveImageThread (saveImageAction, fileName, this->width, this->height, this->rgbBuffers[bufferIndex].first);
                saveImageThread.detach();
                saveImageNumber.assign(saveImageNumber.getValue() + 1);

                Eigen::Matrix4f cameraToObjectTransMat = this->cameraToObjectTransMatBuffers[bufferIndex].first;
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
            usleep(100000);
        }
        fclose(pfile);
        printf("Save image daemon running done.\n");
        fflush(stdout);
        printf("Waiting for detached image saving thread to terminate.\n");
        fflush(stdout);
        usleep(3000000);
        printf("Waiting for detached image saving thread to terminate done.\n");
        fflush(stdout);
        isRosOk.assign(false);
    };
//    std::thread saveImageDaemonThread (saveImageDaemonAction);
//    saveImageDaemonThread.detach();
    isRosOk.assign(false);
}

void RosInterface::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cameraInfoConstPtr) {
    if (!isCameraInitialized.getValue()) {
        cameraInfo = *cameraInfoConstPtr;
        width = cameraInfo.width;
        height = cameraInfo.height;
        printf("CameraInfo:\n");
        printf("width = %d, height = %d\n", width, height);
        printf("%s\n", cameraInfo.distortion_model.c_str());
        for(int i = 0; i < numBuffers; i++) {
            uint8_t * newDepthImage = (uint8_t *)calloc(width * height * 2, sizeof(uint8_t));
            uint8_t * newRgbImage = (uint8_t *)calloc(width * height * 3, sizeof(uint8_t));
            depthBuffers[i] = std::pair<uint8_t *, int64_t>(newDepthImage, 0);
            rgbBuffers[i] = std::pair<uint8_t *, int64_t>(newRgbImage, 0);
            cameraToObjectTransMatBuffers[i].first.setIdentity();
            cameraToObjectTransMatBuffers[i].second = 0;
        }
        isCameraInitialized.assign(true);
    }
}

void RosInterface::depthRgbMsgCallback(const sensor_msgs::ImageConstPtr& rgbImage, const sensor_msgs::ImageConstPtr& depthImage)
{
    if (!isCameraInitialized.getValue()) {
        return;
    }
    cv_bridge::CvImagePtr cvDepthImagePtr;
    try {
        cvDepthImagePtr = cv_bridge::toCvCopy(depthImage, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_bridge::CvImagePtr cvRgbImagePtr;
    try {
        cvRgbImagePtr = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    lastAllFrameTime = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
    const cv::Size depthImageSize = cvDepthImagePtr->image.size();
    const cv::Size rgbImageSize = cvRgbImagePtr->image.size();
    if (rgbImageSize.height != depthImageSize.height || rgbImageSize.width != depthImageSize.width) {
        ROS_ERROR("RGB image size != depth image size!!!");
        return;
    }
    const int nPixel = depthImageSize.height * depthImageSize.width;
    if (!cvRgbImagePtr->image.isContinuous()) {
        ROS_ERROR("RGB image is not continuous!!!");
        return;
    }
    if (!cvDepthImagePtr->image.isContinuous()) {
        ROS_ERROR("Depth image is not continuous!!!");
        return;
    }
    int bufferIndex = (latestAllFrameIndex.getValue() + 1) % numBuffers;
    memcpy(depthBuffers[bufferIndex].first, cvDepthImagePtr->image.data, nPixel * 2);
    memcpy(rgbBuffers[bufferIndex].first, cvRgbImagePtr->image.data, nPixel * 3);
    depthBuffers[bufferIndex].second = lastAllFrameTime;
    rgbBuffers[bufferIndex].second = lastAllFrameTime;
    rgbImageRosTimeBuffers[bufferIndex] = rgbImage->header.stamp;
    cv::Mat m = pSLAM->TrackRGBD(cvRgbImagePtr->image,cvDepthImagePtr->image,cvRgbImagePtr->header.stamp.toSec());
    Eigen::Matrix4f transMat;
    transMat << m.at<float>(0, 0), m.at<float>(0, 1), m.at<float>(0, 2), m.at<float>(0, 3),
            m.at<float>(1, 0), m.at<float>(1, 1), m.at<float>(1, 2), m.at<float>(1, 3),
            m.at<float>(2, 0), m.at<float>(2, 1), m.at<float>(2, 2), m.at<float>(2, 3),
            m.at<float>(3, 0), m.at<float>(3, 1), m.at<float>(3, 2), m.at<float>(3, 3);
//    Eigen::Matrix4f invTransMat = transMat.inverse();
//    for (int i = 0; i < 4; i++) {
//        for (int j = 0; j < 4; j++) {
//            printf("%f ", invTransMat(i, j));
//        }
//        printf("\n");
//    }
//    printf("\n");
//    fflush(stdout);
    cameraToObjectTransMatBuffers[bufferIndex].first = transMat.inverse();
//    cameraToObjectTransMatBuffers[bufferIndex].first = transMat;
    cameraToObjectTransMatBuffers[bufferIndex].second = lastAllFrameTime;
    latestAllFrameIndex++;
}

RosInterface::~RosInterface()
{
    if(initSuccessful) {
        printf("Destroying RosInterface...");
        printf("Waiting for ROS node shutting down...\n");
        fflush(stdout);
        isSystemRunning.assign(false);
        rosThread.join();
        printf("Waiting for ROS node shutting down done.\n");
        fflush(stdout);
        printf("Deleting depth/rgb Buffers...\n");
        fflush(stdout);
        for (int i = 0; i < numBuffers; i++) {
            free(depthBuffers[i].first);
            free(rgbBuffers[i].first);
        }
        printf("Deleting depth/rgb buffers done.\n");
        fflush(stdout);
        printf("Deleting ORB SLAM2...\n");
        fflush(stdout);
        delete pSLAM;
        printf("Destroying RosInterface done.\n");
        fflush(stdout);
    }
}

