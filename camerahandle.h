//
// Created by luke on 19.12.17.
//
#ifndef STEREOVISIONPROJECT_MAIN_H
#define STEREOVISIONPROJECT_MAIN_H

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <iomanip>
#include <fstream>


#include <vrmusbcamcpp.h>

#define LEFT_INDEX 1
#define RIGHT_INDEX 2

void initCamera(VRmUsbCamCPP::DevicePtr device, VRmDWORD& port, VRmUsbCamCPP::ColorFormat screen_colorformat, VRmUsbCamCPP::ImageFormat& target_format);

void initCamera(int L_CAM,int R_CAM);

cv::Mat readCamera(VRmUsbCamCPP::DevicePtr device, VRmDWORD& f_port);

cv::Mat readSingleFrame(VRmUsbCamCPP::DevicePtr device, VRmDWORD& f_port);

cv::Mat readLeftFrame();

cv::Mat readRightFrame();

void openCamera (VRmUsbCamCPP::DevicePtr &deviceL,VRmUsbCamCPP::DevicePtr &deviceR);

void invertCameras(VRmUsbCamCPP::DevicePtr& deviceL, VRmDWORD& f_portL,VRmUsbCamCPP::DevicePtr& deviceR, VRmDWORD& f_portR);

void CloseDevice(VRmUsbCamCPP::DevicePtr &device);

void loadCameraConfig(VRmUsbCamCPP::DevicePtr deviceL,VRmUsbCamCPP::DevicePtr deviceR);

static VRmUsbCamCPP::ImageFormat final_target_format;

#endif //STEREOVISIONPROJECT_MAIN_H
