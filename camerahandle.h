/*
 * Plik nagłówkowy zawierający metody obsługujące kamery
 */

/// Zabezpieczenie przed kilkukrotnym wczytaniem pliku nagłówkowego
#ifndef STEREOVISIONPROJECT_CAMERAHANDLE_H
#define STEREOVISIONPROJECT_CAMERAHANDLE_H

/// Dodanie bibliotek wykorzystywanych w projekcie
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <iomanip>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <vrmusbcamcpp.h>
#include "stereovision.h"

/// Definicja indeksu kamer Logitech
#define LEFT_INDEX 1
#define RIGHT_INDEX 2

/// Zmienne i metody obsługujące kamery VRmUSB
void openCamera (VRmUsbCamCPP::DevicePtr &deviceL,VRmUsbCamCPP::DevicePtr &deviceR);

void initCamera(VRmUsbCamCPP::DevicePtr device, VRmDWORD& port, VRmUsbCamCPP::ColorFormat screen_colorformat, VRmUsbCamCPP::ImageFormat& target_format);

cv::Mat readSingleFrame(VRmUsbCamCPP::DevicePtr device, VRmDWORD& f_port);

void adjustExposure(VRmUsbCamCPP::DevicePtr& deviceL, VRmDWORD& f_portL,VRmUsbCamCPP::DevicePtr& deviceR, VRmDWORD& f_portR);

void invertCameras(VRmUsbCamCPP::DevicePtr& deviceL, VRmDWORD& f_portL,VRmUsbCamCPP::DevicePtr& deviceR, VRmDWORD& f_portR);

void CloseDevice(VRmUsbCamCPP::DevicePtr &device);

void resetDevice(VRmUsbCamCPP::DevicePtr &deviceL, VRmUsbCamCPP::DevicePtr &deviceR);

static VRmUsbCamCPP::ImageFormat final_target_format;

/// Metody obsługujące kamery Logitech

cv::Mat readLeftFrame();

cv::Mat readRightFrame();


#endif //STEREOVISIONPROJECT_CAMERAHANDLE_H
