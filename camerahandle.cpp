//
// Created by luke on 21.12.17.
//
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
    // display error when no camera has been found


}
void loadCameraConfig(DevicePtr deviceL,DevicePtr deviceR)
{
    VRmUserData* left_cam_config;
    VRmUserData* right_cam_config;

    std::string lFilename = "LeftCameraConfig.vcc";

    std::string rFilename = "RightCameraConfig.vcc";

    std::fstream l_backupfile(lFilename.c_str(), std::ios_base::in|std::ios_base::binary);
    l_backupfile.seekg (0, l_backupfile.end);
        int llength = l_backupfile.tellg();
    l_backupfile.seekg (0, l_backupfile.beg);

    char * lbuffer = new char [llength];
    std::fstream r_backupfile(rFilename.c_str(), std::ios_base::in|std::ios_base::binary);
    r_backupfile.seekg (0, r_backupfile.end);
        int rlength = r_backupfile.tellg() ;
    r_backupfile.seekg (0, r_backupfile.beg);

    char * rbuffer = new char [rlength];

    l_backupfile.read(lbuffer,llength);
    r_backupfile.read(rbuffer,rlength);

    if(l_backupfile) {
        std::cout << "File containg left camera conifg loaded properly" << std::endl;
        if(r_backupfile) {
            std::cout << "File containing right camera configuration loaded properly" << std::endl;
            std::cout << rlength << std::endl;
            std::cout << lbuffer << std::endl;

            std::vector<unsigned char> lufer(lbuffer,lbuffer + strlen(lbuffer));
            std::vector<unsigned char> rufer(rbuffer,rbuffer + strlen(rbuffer));
            deviceL->SaveUserData(lufer);
            deviceR->SaveUserData(rufer);
        }
    }

    delete[] lbuffer;
    delete[] rbuffer;
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

//
//    if (device->get_PropertySupported(VRM_PROPID_CAM_EXPOSURE_TIME_F))
//    {
//        device->set_PropertyValue(VRM_PROPID_CAM_EXPOSURE_TIME_F, 200.f);
//    }
//    if (device->get_PropertySupported(VRM_PROPID_PLUGIN_IMAGE_PROCESSING_B))
//    {
//        device->set_PropertyValue(VRM_PROPID_PLUGIN_IMAGE_PROCESSING_B, true);
//    }
//    if (device->get_PropertySupported(VRM_PROPID_IMAGEPROC_DPM_B))
//    {
//        device->set_PropertyValue(VRM_PROPID_IMAGEPROC_DPM_B, true);
//    }



//    device->SaveConfig(2);
//    if (device->get_PropertySupported(VRM_PROPID_FILTER_MASTER_CONTRAST_F))
//    {
//        device->set_PropertyValue(VRM_PROPID_FILTER_MASTER_CONTRAST_F, 10.f);
//    }


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
        throw Exception(ss.str());
    }


    std::cout << "Selected target format: " + target_format.ToString() << std::endl;

    // uncomment these lines to flip output image vertical / horizontal:
    //	target_format.FlipHorizontal();
    //	target_format.FlipVertical();
}

void initCamera(int L_CAM,int R_CAM)
{
    cv::VideoCapture* leftCapture,rightCapture;

    leftCapture->open(L_CAM);


}

void invertCameras(VRmUsbCamCPP::DevicePtr& deviceL, VRmDWORD& f_portL,VRmUsbCamCPP::DevicePtr& deviceR, VRmDWORD& f_portR) {

    VRmUsbCamCPP::DevicePtr tempPtr = deviceR;
    VRmDWORD  tempPort = f_portR;

    deviceR = deviceL;
    f_portR = f_portL;

    deviceL = tempPtr;
    f_portL = tempPort;

    std::cout << "Inverted cameras" << std::endl;

}

cv::Mat readCamera(DevicePtr device, VRmDWORD &f_port) {
    float fps;
    ImagePtr p_source_image,p_target_image;
    int FramesDropped = 0;
    // p_target_image = VRmUsbCam::NewImage(f_target_format);


    if(p_source_image= device->LockNextImage(f_port, &FramesDropped, 10000))
    {

        if(p_source_image->get_FrameCounter() %10 == 0)
        {
            fps = boost::get<float>(device->get_PropertyValue(VRM_PROPID_GRAB_FRAMERATE_AVERAGE_F));
            std::cout << "\rFrames/sec: " << fps;
            fflush(stdout);
        }


        cv::Mat *pImage = new cv::Mat(p_source_image.get()->get_ImageFormat().get_Size().m_height,
                                      p_source_image.get()->get_ImageFormat().get_Size().m_width, CV_8UC1,
                                      p_source_image->get_Buffer()); //tutaj powinno być p_target_image ale coś jest nie tak z tym formatem

        cvtColor(*pImage, *pImage, cv::COLOR_BayerGB2RGB);



        cv::Mat outShow = *pImage;

        device->UnlockNextImage(p_source_image);

        p_target_image.reset();


        if(FramesDropped)
            std::cout << "-" << FramesDropped << "frame(s) dropped - \n" << std::endl;


        return outShow;
    }
    else
    {
        // VRmUsbCamLockNextImageEx2() didi not return an image. why?
        // ----------------------------------------------------------
        std::cout << "Somsing wrong \n" << std::endl;
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

cv::Mat readSingleFrame(VRmUsbCamCPP::DevicePtr device, VRmDWORD& f_port)
{
    {
        ImagePtr p_source_image;
        int FramesDropped = 0;

        if(p_source_image= device->LockNextImage(f_port, &FramesDropped, 10000))
        {
            ImagePtr p_target_img = VRmUsbCam::NewImage(final_target_format);

            VRmUsbCam::ConvertImage(p_source_image,p_target_img);

            cv::Mat *pImage = new cv::Mat(p_target_img.get()->get_ImageFormat().get_Size().m_height,
                                          p_target_img.get()->get_ImageFormat().get_Size().m_width,CV_8UC4,
                                          p_target_img->get_Buffer()); //tutaj powinno być p_target_image ale coś jest nie tak z tym formatem

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
            // ----------------------------------------------------------
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



    //VRmUsbCamCleanup();
    // std::cout << "\n Celanup done" << std::endl;

    VRmUsbCam::ResetDeviceKeyList();
    std::cout << "\nReseting device keylist" << std::endl;


}


