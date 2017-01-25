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

RosInterface::RosInterface()
 : width(-1),
   height(-1),
   fps(30),
   initSuccessful(true)
{
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
    pSLAM = new ORB_SLAM2::System("/home/gao/Downloads/ORB_SLAM2/Vocabulary/ORBvoc.txt",
                           "/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Realsense_SR300.yaml", ORB_SLAM2::System::RGBD,false);
//                           "/home/gao/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Asus.yaml", ORB_SLAM2::System::RGBD,false);
    auto rosAction = [this]() {
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
                br.sendTransform(tf::StampedTransform(transform, current_time, "iiwa_base", "elasticfusion_point_cloud"));
                pub.publish(pointCloudMsgBuffer[bufferIndex]);
            }
            if (latestAllFrameIndex.getValue() >= 0) {
                const int bufferIndex = latestAllFrameIndex.getValue() % numBuffers;
                Eigen::Matrix4f cameraToCloudTransMat = poseMat[bufferIndex].first;
                tf::Matrix3x3 cameraToCloudRotMat(cameraToCloudTransMat(0, 0), cameraToCloudTransMat(0, 1), cameraToCloudTransMat(0, 2),
                                                  cameraToCloudTransMat(1, 0), cameraToCloudTransMat(1, 1), cameraToCloudTransMat(1, 2),
                                                  cameraToCloudTransMat(2, 0), cameraToCloudTransMat(2, 1), cameraToCloudTransMat(2, 2));
                tf::Vector3 cameraToCloudTranslationVec(cameraToCloudTransMat(0, 3), cameraToCloudTransMat(1, 3), cameraToCloudTransMat(2, 3));
                tf::Transform cameraToCloudTrans(cameraToCloudRotMat, cameraToCloudTranslationVec);
                br.sendTransform(tf::StampedTransform(cameraToCloudTrans, current_time, "iiwa_base", "realsense_camera"));
                geometry_msgs::Pose cameraPose;
                geometry_msgs::Point cameraPosePoint;
                geometry_msgs::Quaternion cameraPoseQuaternion;
                cameraPosePoint.x = cameraToCloudTranslationVec.x();
                cameraPosePoint.y = cameraToCloudTranslationVec.y();
                cameraPosePoint.z = cameraToCloudTranslationVec.z();
                tf::Quaternion tmpQ;
                cameraToCloudRotMat.getRotation(tmpQ);
//                tf::Matrix3x3 cameraToCloudRotMatInv = cameraToCloudRotMat.inverse();
//                cameraToCloudRotMatInv.getRotation(tmpQ);
                cameraPoseQuaternion.x = tmpQ.x();
                cameraPoseQuaternion.y = tmpQ.y();
                cameraPoseQuaternion.z = tmpQ.z();
                cameraPoseQuaternion.w = tmpQ.w();
                cameraPose.position = cameraPosePoint;
                cameraPose.orientation = cameraPoseQuaternion;
                geometry_msgs::PoseStamped cameraPoseStamped;
                cameraPoseStamped.header.frame_id = "elasticfusion_point_cloud";
                cameraPoseStamped.header.stamp = current_time;
                cameraPoseStamped.header.seq = 0;
                cameraPoseStamped.pose = cameraPose;
                posePub.publish(cameraPoseStamped);
            }
//            usleep(100);
            rate.sleep();
        }
        ros::shutdown();
    };
    rosThread = std::move(std::thread(rosAction));
    while (!isCameraInitialized.getValue()) {
        usleep(10000);
    }
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
            poseMat[i].first.setIdentity();
            poseMat[i].second = 0;
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
    poseMat[bufferIndex].first = transMat.inverse();
//    poseMat[bufferIndex].first = transMat;
    poseMat[bufferIndex].second = lastAllFrameTime;
    latestAllFrameIndex++;
}

RosInterface::~RosInterface()
{
    if(initSuccessful) {
        printf("RosInterface dead.");
        isSystemRunning.assign(false);
        rosThread.join();
        for (int i = 0; i < numBuffers; i++) {
            free(depthBuffers[i].first);
            free(rgbBuffers[i].first);
        }
        delete pSLAM;
    }
}

