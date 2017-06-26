#include "SR300_ORB_Interface.h"
#include <unistd.h>
#define my_assert(x) if (x) {} else {cerr<<endl<<std::string("assertion failed @line:") + std::to_string(__LINE__)+" in file:"<<__FILE__<<endl; throw -1;}

SR300_ORB_Interface::SR300_ORB_Interface(int inWidth, int inHeight, int fps,
                                         std::string argInDepthCameraYamlPath,
                                         std::string argInOrbVocBinPath)
 : width(inWidth),
   height(inHeight),
   fps(fps),
   initSuccessful(true),
   shouldStop(false),
   depthCameraConfigYamlPath(argInDepthCameraYamlPath),
   orbVocBinPath(argInOrbVocBinPath)
{
    pSLAM = new ORB_SLAM2::System(orbVocBinPath, depthCameraConfigYamlPath, ORB_SLAM2::System::RGBD, false);
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) {
        fprintf(stderr, "No device found!");
    }
    // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    // Configure all streams to run at VGA resolution at 60 frames per second
    dev->enable_stream(rs::stream::depth, inWidth, inHeight, rs::format::z16, fps);
    dev->enable_stream(rs::stream::color, inWidth, inHeight, rs::format::rgb8, fps);
    dev->enable_stream(rs::stream::infrared, inWidth, inHeight, rs::format::y8, fps);
    initSuccessful = true;
    printf("depth scale: %f\n", dev->get_depth_scale());
    const float depth_scale = dev->get_depth_scale();
    std::cout << "Depth factor of stream:" << depth_scale <<std::endl;
    const rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
    std::cout << "Intrinsic parameter of stream:" << rs::stream::color<<std::endl;
    std::cout << "Capturing " << rs::stream ::color << " at " << color_intrin.width << " x " << color_intrin.height;
    std::cout << std::setprecision(1) << std::fixed << ", fov = " << color_intrin.hfov() << " x " << color_intrin.vfov() << ", distortion = " << color_intrin.model() << std::endl;
    std::cout << std::setprecision(6);
    fx = color_intrin.fx;
    fy = color_intrin.fy;
    cx = color_intrin.ppx;
    cy = color_intrin.ppy;
    cv::FileStorage fSettings(depthCameraConfigYamlPath, cv::FileStorage::READ);
    float yaml_fx = fSettings["Camera.fx"];
    float yaml_fy = fSettings["Camera.fy"];
    float yaml_cx = fSettings["Camera.cx"];
    float yaml_cy = fSettings["Camera.cy"];
    const float th = 1e-6;
    my_assert(fabs(fx - yaml_fx) < th);
    my_assert(fabs(fy - yaml_fy) < th);
    my_assert(fabs(cx - yaml_cx) < th);
    my_assert(fabs(cy - yaml_cy) < th);

    std::cout << "fx:" << color_intrin.fx << std::endl;
    std::cout << "fy:" << color_intrin.fy << std::endl;
    std::cout << "ppx:" << color_intrin.ppx <<std::endl;
    std::cout << "ppy:" << color_intrin.ppy << std::endl;

    if (color_intrin.model() == rs::distortion::modified_brown_conrady) {
        std::cout << "distortion coefficients: ";
        std::cout << color_intrin.coeffs[0]<< ' ' << color_intrin.coeffs[1]<< ' '  << color_intrin.coeffs[2]<< ' '  << color_intrin.coeffs[3]<< ' '  << color_intrin.coeffs[4] << std::endl;
    }
    std::cout << std::endl;
    latestAllFrameIndex.assign(-1);
    for(int i = 0; i < numBuffers; i++) {
        uint8_t * newRgbImage = (uint8_t *)calloc(width * height * 3, sizeof(uint8_t));
        uint8_t * newDepthAlignedToRgbImage = (uint8_t *)calloc(width * height * 2, sizeof(uint8_t));
        rgbBuffers[i] = std::pair<uint8_t *, int64_t>(newRgbImage, 0);
        depthAlignedToRgbBuffers[i] = std::pair<uint8_t *, int64_t>(newDepthAlignedToRgbImage, 0);
    }
//    double whiteBalance = 3800;//devPtr->get_option(rs::option::color_white_balance);
//    dev->set_option(rs::option::color_white_balance, whiteBalance);
    dev->set_option(rs::option::color_enable_auto_exposure, 1);
    dev->start();
    auto all_stream_feeder = [this]() {
        while(!dev->is_streaming()) {
            usleep(1000);
        }
        const int nPixel = dev->get_stream_width(rs::stream::color) * dev->get_stream_height(rs::stream::color);
        my_assert(dev->get_stream_width(rs::stream::depth_aligned_to_color)*dev->get_stream_height(rs::stream::depth_aligned_to_color) == nPixel);
        while(!shouldStop.getValue()) {
            dev->wait_for_frames();
            std::chrono::system_clock::duration epoch_dur = std::chrono::system_clock::now().time_since_epoch();
            lastAllFrameTime =  epoch_dur / std::chrono::milliseconds(1);
            const int bufferIndex = (latestAllFrameIndex.getValue() + 1) % numBuffers;
            memcpy(rgbBuffers[bufferIndex].first, dev->get_frame_data(rs::stream::color), nPixel * 3);
            memcpy(depthAlignedToRgbBuffers[bufferIndex].first, dev->get_frame_data(rs::stream::depth_aligned_to_color), nPixel * 2);
            rgbBuffers[bufferIndex].second = lastAllFrameTime;
            depthAlignedToRgbBuffers[bufferIndex].second = lastAllFrameTime;
            cv::Mat cvRgbImageMat = cv::Mat(cv::Size(width, height), CV_8UC3, rgbBuffers[bufferIndex].first, cv::Mat::AUTO_STEP);
            cv::Mat cvDepthImageMat = cv::Mat(cv::Size(width, height), CV_16UC1, depthAlignedToRgbBuffers[bufferIndex].first, cv::Mat::AUTO_STEP);
            cv::Mat m = pSLAM->TrackRGBD(cvRgbImageMat, cvDepthImageMat, double(lastAllFrameTime)/1000.0);
            Eigen::Matrix4f transMat;
            if (0 == m.cols) {
                transMat = Eigen::Matrix4f::Zero();
                cameraToObjectTransMatBuffers[bufferIndex].first = transMat;
                cameraToObjectTransMatBuffers[bufferIndex].second = lastAllFrameTime;
            } else {
                transMat << m.at<float>(0, 0), m.at<float>(0, 1), m.at<float>(0, 2), m.at<float>(0, 3),
                        m.at<float>(1, 0), m.at<float>(1, 1), m.at<float>(1, 2), m.at<float>(1, 3),
                        m.at<float>(2, 0), m.at<float>(2, 1), m.at<float>(2, 2), m.at<float>(2, 3),
                        m.at<float>(3, 0), m.at<float>(3, 1), m.at<float>(3, 2), m.at<float>(3, 3);
                cameraToObjectTransMatBuffers[bufferIndex].first = transMat.inverse();
                cameraToObjectTransMatBuffers[bufferIndex].second = lastAllFrameTime;
            }
            latestAllFrameIndex++;
            usleep(1000);
        }
    };
    allFrameDaemonThread = std::move(std::thread(all_stream_feeder));
}

SR300_ORB_Interface::~SR300_ORB_Interface()
{
    if(initSuccessful)
    {
        shouldStop.assign(true);
        allFrameDaemonThread.join();
        dev->stop();
        dev->disable_stream(rs::stream::depth);
        dev->disable_stream(rs::stream::color);
        dev->disable_stream(rs::stream::infrared);
        for(int i = 0; i < numBuffers; i++) {
            free(rgbBuffers[i].first);
            free(depthAlignedToRgbBuffers[i].first);
        }
        delete pSLAM;
    }
}

