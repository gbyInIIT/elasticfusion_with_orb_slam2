#include "Tools/RosInterface.h"
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visp_hand2eye_calibration/TransformArray.h>
#include <visp_hand2eye_calibration/compute_effector_camera_quick.h>
#include "Tools/ThreadMutexObject.h"

int main(int argc, char * argv[])
{
    RosInterface * asus = new RosInterface();
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle node;
    tf::TransformListener listener(ros::Duration(100.0));
    tf::TransformBroadcaster br;
    ros::Publisher worldToHandPub = node.advertise<visp_hand2eye_calibration::TransformArray>("world_effector", 10);
    ros::Publisher cameraToObjectPub = node.advertise<visp_hand2eye_calibration::TransformArray>("camera_object", 10);
    ros::ServiceClient serviceClient = node.serviceClient<visp_hand2eye_calibration::compute_effector_camera_quick>("compute_effector_camera_quick");
    cv::namedWindow( "RGB", cv::WINDOW_AUTOSIZE );
    cv::namedWindow( "Depth", cv::WINDOW_AUTOSIZE );
    ros::Rate rate(30);
    visp_hand2eye_calibration::TransformArray cameraToObjectRosTransformArray;
    visp_hand2eye_calibration::TransformArray worldToHandRosTransformArray;
    const size_t sizeOfCalibrationArray = 200;
    cameraToObjectRosTransformArray.transforms.resize(sizeOfCalibrationArray);
    worldToHandRosTransformArray.transforms.resize(sizeOfCalibrationArray);
    int iCalibrationArray = 0;
    unsigned int iCalibrationArraySent = 0;
//    tf::Transform cameraToFlangeRosTransform;
    ThreadMutexObject<tf::Transform> cameraToFlangeRosTransform;
    while(node.ok()) {
        tf::StampedTransform transform;
        const int imageIdx = asus->latestAllFrameIndex.getValue();
        if (imageIdx >= 0) {
            const int buffIdx = imageIdx % asus->numBuffers;
            Eigen::Matrix4f cameraToObjectTransMat = asus->cameraToObjectTransMatBuffers[buffIdx].first;
            ros::Time rgbImageRosTime = asus->rgbImageRosTimeBuffers[buffIdx];
            ros::Time nowRosTime = ros::Time::now();
            try{
                ros::Duration timeout(300);
//                listener.waitForTransform("iiwa_base", "flange", rgbImageRosTime, timeout);
//                listener.lookupTransform("iiwa_base", "flange", rgbImageRosTime, transform);
//                listener.lookupTransform("flange", "iiwa_base", rgbImageRosTime, transform);
                listener.lookupTransform("flange", "iiwa_base", ros::Time(0), transform);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
//                ros::Duration(1.0).sleep();
                rate.sleep();
                continue;
            }
            br.sendTransform(tf::StampedTransform(transform.inverse(), nowRosTime, "iiwa_base", "flange_check"));
            tf::Quaternion q = transform.getRotation();
//            printf("flange to iiwa_base:\n");
//            printf("x=%f, y=%f, z=%f, w=%f\n", q.getX(), q.getY(), q.getZ(), q.getW());

            tf::Matrix3x3 cameraToObjectRotMat(cameraToObjectTransMat(0, 0), cameraToObjectTransMat(0, 1), cameraToObjectTransMat(0, 2),
                                               cameraToObjectTransMat(1, 0), cameraToObjectTransMat(1, 1), cameraToObjectTransMat(1, 2),
                                               cameraToObjectTransMat(2, 0), cameraToObjectTransMat(2, 1), cameraToObjectTransMat(2, 2));
            tf::Quaternion cameraToObjectQuaternion;
            cameraToObjectRotMat.getRotation(cameraToObjectQuaternion);
            tf::Vector3 cameraToObjectTranslationVec(cameraToObjectTransMat(0, 3), cameraToObjectTransMat(1, 3), cameraToObjectTransMat(2, 3));
            tf::Transform cameraToObjectTfTransform(cameraToObjectRotMat, cameraToObjectTranslationVec);
            cameraToObjectTfTransform = cameraToObjectTfTransform.inverse();
            cameraToObjectQuaternion = cameraToObjectTfTransform.getRotation();
            cameraToObjectTranslationVec = cameraToObjectTfTransform.getOrigin();

            tf::Quaternion worldToHandQuaternion = transform.getRotation();
            tf::Vector3 worldToHandTranslationVec = transform.getOrigin();
            tf::Transform worldToHandTfTransform(transform);
            worldToHandTfTransform = worldToHandTfTransform.inverse();
            worldToHandQuaternion = worldToHandTfTransform.getRotation();
            worldToHandTranslationVec = worldToHandTfTransform.getOrigin();


            if (iCalibrationArray < sizeOfCalibrationArray) {
//                camera to object transformation
                geometry_msgs::Transform cameraToObjectRosTransform;
                cameraToObjectRosTransform.translation.x = cameraToObjectTranslationVec.x();
                cameraToObjectRosTransform.translation.y = cameraToObjectTranslationVec.y();
                cameraToObjectRosTransform.translation.z = cameraToObjectTranslationVec.z();
                cameraToObjectRosTransform.rotation.x = cameraToObjectQuaternion.x();
                cameraToObjectRosTransform.rotation.y = cameraToObjectQuaternion.y();
                cameraToObjectRosTransform.rotation.z = cameraToObjectQuaternion.z();
                cameraToObjectRosTransform.rotation.w = cameraToObjectQuaternion.w();
                cameraToObjectRosTransformArray.transforms[iCalibrationArray] = cameraToObjectRosTransform;
//                world to hand transformation
                geometry_msgs::Transform worldToHandRosTransform;
                worldToHandRosTransform.translation.x = worldToHandTranslationVec.x();
                worldToHandRosTransform.translation.y = worldToHandTranslationVec.y();
                worldToHandRosTransform.translation.z = worldToHandTranslationVec.z();
                worldToHandRosTransform.rotation.x = worldToHandQuaternion.x();
                worldToHandRosTransform.rotation.y = worldToHandQuaternion.y();
                worldToHandRosTransform.rotation.z = worldToHandQuaternion.z();
                worldToHandRosTransform.rotation.w = worldToHandQuaternion.w();
                worldToHandRosTransformArray.transforms[iCalibrationArray] = worldToHandRosTransform;
                iCalibrationArray++;
            } else {
                cameraToObjectRosTransformArray.header.frame_id = "calibrate_realsense";
                cameraToObjectRosTransformArray.header.seq = iCalibrationArraySent;
                cameraToObjectRosTransformArray.header.stamp = nowRosTime;
                cameraToObjectPub.publish(cameraToObjectRosTransformArray);
                worldToHandRosTransformArray.header.frame_id = "calibrate_realsense_world";
                worldToHandRosTransformArray.header.seq = iCalibrationArraySent;
                worldToHandRosTransformArray.header.stamp = nowRosTime;
                worldToHandPub.publish(worldToHandRosTransformArray);
                iCalibrationArraySent++;
                iCalibrationArray = 0;
                auto calibrationAction = [&cameraToObjectRosTransformArray, &worldToHandRosTransformArray, &serviceClient, &cameraToFlangeRosTransform]() {
                    visp_hand2eye_calibration::compute_effector_camera_quick srv;
                    srv.request.camera_object = cameraToObjectRosTransformArray;
                    srv.request.world_effector = worldToHandRosTransformArray;
                    if (serviceClient.call(srv)) {
                        ROS_INFO("fine:) \n");
                        tf::Vector3 cameraToFlangeTranslationVec(srv.response.effector_camera.translation.x,
                                                                 srv.response.effector_camera.translation.y,
                                                                 srv.response.effector_camera.translation.z);
                        tf::Quaternion cameraToFlangeQuaternion(srv.response.effector_camera.rotation.x,
                                                                srv.response.effector_camera.rotation.y,
                                                                srv.response.effector_camera.rotation.z,
                                                                srv.response.effector_camera.rotation.w);
                        tf::Transform tmpTrans(cameraToFlangeQuaternion, cameraToFlangeTranslationVec);
                        cameraToFlangeRosTransform.assign(tmpTrans);
//                    cameraToFlangeRosTransform = cameraToFlangeRosTransform.inverse();
                    } else {
                        ROS_ERROR("Failed to call service add_two_ints");
                    }
                };
                std::thread calibrationActionThread(calibrationAction);
                calibrationActionThread.detach();
            }
            if (iCalibrationArraySent > 0) {
                br.sendTransform(tf::StampedTransform(cameraToFlangeRosTransform.getValue(), nowRosTime, "flange_check", "realsense_camera_calibrated"));
            }
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
            cv::waitKey(3);
        }
        rate.sleep();
    }
    return 0;
}
