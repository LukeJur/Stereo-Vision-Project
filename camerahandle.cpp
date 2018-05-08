#include "camerahandle.h"


using namespace VRmUsbCamCPP;

void openCamera (DevicePtr &deviceL, DevicePtr &deviceR)
{
    VRmUsbCam::UpdateDeviceKeyList();
    //Check for connected devices
    std::vector<DeviceKeyPtr> devlist = VRmUsbCam::get_DeviceKeyList();

    for (size_t i=0; i<devlist.size(); ++i)
    {
        if (!devlist[i]->get_Busy())
        {
            deviceL= VRmUsbCam::OpenDevice(devlist[i]);
            if (!devlist[i+1]->get_Busy()){deviceR= VRmUsbCam::OpenDevice(devlist[i+1]);};
            break;
        }
    }
    if(!deviceL)
    {
        std::cerr << "Left camera not detected" << std::endl;
        exit(-1);
    }
    if(!deviceR)
    {
        std::cout << "Second camera not detected \n" << std::endl;
        exit(-1);
    }
}
void initCamera(DevicePtr device,unsigned int &port, ColorFormat f_screen_color_format, ImageFormat &target_format)
{
    // get connected sensor ports
    std::vector<int> sensor_port_list= device->get_SensorPortList();
    // for this demo we switch off all connected sensor but the first one in the port list
    for(size_t ii=0; ii<sensor_port_list.size();ii++)
    {
        port= sensor_port_list[ii];

        // on single sensor devices this property does not exist
        VRmPropId sensor_enable = (VRmPropId)(VRM_PROPID_GRAB_SENSOR_ENABLE_1_B-1+port);
        if(device->get_PropertySupported(sensor_enable))
        {
            //enable first sensor in port list
            bool enable = 1;
            if(ii)
                enable = 0;
            device->set_PropertyValue(sensor_enable, enable);
        }
    }
    device->LoadConfig(1);
    //now get the first sensor port
    port= sensor_port_list[0];

    // get the currently selected source (raw) format
    ImageFormat sourceFormat = device->get_SourceFormat(port);

    std::cout << "Selected source format: " << sourceFormat.ToString() << std::endl;

    // select a target format from the list of formats we can convert the source images to.
    // we search for the screen's pixelformat for rendering
    std::vector<ImageFormat> targetFormatList= device->get_TargetFormatList(port);
    for ( size_t i = 0; i < targetFormatList.size(); ++i )
    {
        if (  targetFormatList[i].get_ColorFormat() == f_screen_color_format )
        {
            target_format = targetFormatList[i];
            final_target_format = targetFormatList[i];
            break;
        }
    }

    // check if the correct format was chosen
    if(target_format.get_ColorFormat() != f_screen_color_format)
    {
        const char *screen_color_format_str;
        VRmUsbCamGetStringFromColorFormat(f_screen_color_format, &screen_color_format_str);
        std::stringstream ss;
        ss << "Error: " << screen_color_format_str << " not found in target format list.";
        throw VRmUsbCamCPP::Exception(ss.str());
    }

    std::cout << "Selected target format: " + target_format.ToString() << std::endl;
}

void invertCameras(VRmUsbCamCPP::DevicePtr& deviceL, VRmDWORD& f_portL,VRmUsbCamCPP::DevicePtr& deviceR, VRmDWORD& f_portR)
{
    VRmUsbCamCPP::DevicePtr tempPtr = deviceR;
    VRmDWORD  tempPort = f_portR;

    deviceR = deviceL;
    f_portR = f_portL;

    deviceL = tempPtr;
    f_portL = tempPort;

    std::cout << "Inverted cameras" << std::endl;
}

cv::Mat readSingleFrame(VRmUsbCamCPP::DevicePtr device, VRmDWORD& f_port)
{
        ImagePtr p_source_image;
        int FramesDropped = 0;

        if(p_source_image= device->LockNextImage(f_port, &FramesDropped, 10000))
        {
            ImagePtr p_target_img = VRmUsbCam::NewImage(final_target_format);

            VRmUsbCam::ConvertImage(p_source_image,p_target_img);

            cv::Mat *pImage = new cv::Mat(p_target_img.get()->get_ImageFormat().get_Size().m_height,
                                          p_target_img.get()->get_ImageFormat().get_Size().m_width,CV_8UC4,
                                          p_target_img->get_Buffer());

            device->UnlockNextImage(p_source_image);

            cv::Mat outImage = pImage->clone();
            delete pImage;
            outImage.convertTo(outImage,CV_8UC3);
            p_source_image.reset();
            p_target_img.reset();
            cvtColor(outImage,outImage, CV_BGRA2BGR);
            return outImage;
        }
        else
        {
            // VRmUsbCamLockNextImageEx2() didi not return an image. why?
            switch(VRmUsbCamGetLastErrorCode())
            {
                case VRM_ERROR_CODE_FUNCTION_CALL_TIMEOUT:
                case VRM_ERROR_CODE_TRIGGER_TIMEOUT:
                case VRM_ERROR_CODE_TRIGGER_STALL:
                    std::cout << "VRmUsbCamLockNextImageEx2() failed with " << VRmUsbCamGetLastError() << std::endl;
                    break;

                case VRM_ERROR_CODE_GENERIC_ERROR:
                default:
                    std::cout << "Unhandled error occured" << std::endl;
            }
        }
}

void adjustExposure(VRmUsbCamCPP::DevicePtr& deviceL, VRmDWORD& f_portL,VRmUsbCamCPP::DevicePtr& deviceR , VRmDWORD& f_portR)
{
    /// Camera parameters control window
    int exposure_slider = 100;

    cv::namedWindow("Camera exposure control", cv::WINDOW_AUTOSIZE);

    char exposure[50];
    sprintf(exposure, "Exposure %d", 500);
    createTrackbar(exposure, "Camera exposure control", &exposure_slider, 500, trackbarLink);

    /// Adjust both camera exposure
    char key = waitKey(1) & 0xff;
    Mat leftCam, rightCam;


    std::cout << "Adjusting exposure time for both cameras \n press 'n' to move on" << std::endl;

    if ((boost::get<bool>(deviceL->get_PropertyValue(VRM_PROPID_PLUGIN_IMAGE_PROCESSING_B))) &&
        (boost::get<bool>(deviceR->get_PropertyValue(VRM_PROPID_PLUGIN_IMAGE_PROCESSING_B))))
        std::cout << "Image processing Enabled" << std::endl;

    while (!(key == 'n')) {

        leftCam = readSingleFrame(deviceL, f_portL);
        rightCam = readSingleFrame(deviceR, f_portR);

        static float previousSample;

        if (previousSample != exposure_slider) {
            deviceL->set_PropertyValue(VRM_PROPID_CAM_EXPOSURE_TIME_F, (float) exposure_slider);
            deviceR->set_PropertyValue(VRM_PROPID_CAM_EXPOSURE_TIME_F, (float) exposure_slider);

        }
        previousSample = exposure_slider;

        imshow("LeftCam", leftCam);
        imshow("RightCam", rightCam);

        key = waitKey(1) & 0xff;
        if (key == 'i') invertCameras(deviceL, f_portL, deviceR, f_portR);
        else if (key == 'q') {
            CloseDevice(deviceL);
            CloseDevice(deviceR);
        }
    }
    destroyAllWindows();
}

cv::Mat readLeftFrame()
{
    static cv::VideoCapture leftCaputre(LEFT_INDEX);
    cv::Mat image;
    if(leftCaputre.isOpened())
        leftCaputre >> image;
    else {
        std::cout << "Video caputre not opened error" << std::endl;
        exit(-1);
    }
    leftCaputre.set(CV_CAP_PROP_AUTOFOCUS,0);
    return image;
}

cv::Mat readRightFrame()
{
    static cv::VideoCapture rightCapture(RIGHT_INDEX);
    cv::Mat image;

    if(rightCapture.isOpened())
        rightCapture >> image;
    else {
        std::cout << "Video caputre not opened error" << std::endl;
        exit(-1);
    }
    rightCapture.set(CV_CAP_PROP_AUTOFOCUS,0);
    return image;
}

void CloseDevice(DevicePtr &device)
{
    device->Stop();
    std::cout << "\nStopping device" << std::endl;

    device->Close();
    std::cout << "\nClosing device" << std::endl;

    device.reset();

    VRmUsbCam::ResetDeviceKeyList();
    std::cout << "\nReseting device keylist" << std::endl;
}

void resetDevice(DevicePtr &deviceL, DevicePtr &deviceR)
{
    std::cout << "Reseting cameras" << std::endl;

    VRmUsbCamClearLastError();

    deviceL->Close();
    std::cout << "\nClosing device" << std::endl;

    deviceL.reset();

    deviceR->Close();
    std::cout << "\nClosing device" << std::endl;

    deviceR.reset();

    VRmUsbCamCleanup();
    std::cout << "\nCelanup done" << std::endl;

    VRmUsbCam::UpdateDeviceKeyList();

    VRmUsbCam::ResetDeviceKeyList();
    std::cout << "\nReseting device keylist" << std::endl;

    VRmUsbCam::UpdateDeviceKeyList();
}

