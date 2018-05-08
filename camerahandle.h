#ifndef STEREOVISIONPROJECT_CAMERAHANDLE_H
#define STEREOVISIONPROJECT_CAMERAHANDLE_H


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
#include "stereovision.h"

#define LEFT_INDEX 1
#define RIGHT_INDEX 2

/// VRmUSB camera methods

void initCamera(VRmUsbCamCPP::DevicePtr device, VRmDWORD& port, VRmUsbCamCPP::ColorFormat screen_colorformat, VRmUsbCamCPP::ImageFormat& target_format);

void openCamera (VRmUsbCamCPP::DevicePtr &deviceL,VRmUsbCamCPP::DevicePtr &deviceR);

cv::Mat readSingleFrame(VRmUsbCamCPP::DevicePtr device, VRmDWORD& f_port);

void adjustExposure(VRmUsbCamCPP::DevicePtr& deviceL, VRmDWORD& f_portL,VRmUsbCamCPP::DevicePtr& deviceR, VRmDWORD& f_portR);

void invertCameras(VRmUsbCamCPP::DevicePtr& deviceL, VRmDWORD& f_portL,VRmUsbCamCPP::DevicePtr& deviceR, VRmDWORD& f_portR);

void CloseDevice(VRmUsbCamCPP::DevicePtr &device);

void resetDevice(VRmUsbCamCPP::DevicePtr &deviceL, VRmUsbCamCPP::DevicePtr &deviceR);

static VRmUsbCamCPP::ImageFormat final_target_format;

/// Logitech camera methods

cv::Mat readLeftFrame();

cv::Mat readRightFrame();


#endif //STEREOVISIONPROJECT_CAMERAHANDLE_H
