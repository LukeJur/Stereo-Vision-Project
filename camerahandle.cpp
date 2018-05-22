/// Dodanie pliku nagłówkowych
#include "camerahandle.h"

/// Wczytanie przestrzeni nazw bibliotek
using namespace VRmUsbCamCPP;

/// Metody kamer VRmUSB

/// Przypisanie wskaźników do kamer
void openCamera (DevicePtr &deviceL, DevicePtr &deviceR)
{
    /// Uaktualnienie listy urządzeń
    VRmUsbCam::UpdateDeviceKeyList();
    /// Pobranie listy urządzen wykrytych przez komputer
    std::vector<DeviceKeyPtr> devlist = VRmUsbCam::get_DeviceKeyList();

    /// Przypisanie zmiennych wskaźników do kamer
    for (size_t i=0; i<devlist.size(); ++i)
    {
        if (!devlist[i]->get_Busy())
        {
            deviceL= VRmUsbCam::OpenDevice(devlist[i]);
            if (!devlist[i+1]->get_Busy()){deviceR= VRmUsbCam::OpenDevice(devlist[i+1]);};
            break;
        }
    }

    /// Sprawdzenie zmiennych
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

/// Inicjalizacja połączenia z kamerami wraz z zadaniem docelowego formatu obrazu
void initCamera(DevicePtr device,unsigned int &port, ColorFormat f_screen_color_format, ImageFormat &target_format)
{
    /// Pobranie portów połączonych kamer
    std::vector<int> sensor_port_list= device->get_SensorPortList();
    /// Aktywacja otrzymanych portów
    for(size_t ii=0; ii<sensor_port_list.size();ii++)
    {
        port= sensor_port_list[ii];
        VRmPropId sensor_enable = (VRmPropId)(VRM_PROPID_GRAB_SENSOR_ENABLE_1_B-1+port);
        if(device->get_PropertySupported(sensor_enable))
        {
            bool enable = 1;
            if(ii)
                enable = 0;
            device->set_PropertyValue(sensor_enable, enable);
        }
    }
    /// Wczytanie konfiguracji z kamery o indeksie 1
    device->LoadConfig(1);

    /// Wczytanie portu kamaery
    port= sensor_port_list[0];

    /// Wczytanie formatu żródłowego
    ImageFormat sourceFormat = device->get_SourceFormat(port);
    std::cout << "Selected source format: " << sourceFormat.ToString() << std::endl;

    /// Wybór docelowego formatu i sprawdzenie jego kompatybilności z urządzeniem
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
/// Odczytanie pojedyńczej klatki obrazu z wybranej kamery
cv::Mat readSingleFrame(VRmUsbCamCPP::DevicePtr device, VRmDWORD& f_port)
{
    /// Deklaracja wskaźnika obrazu
    ImagePtr p_source_image;
    /// Zmienna licząca utracone ramki obrazu
    int FramesDropped = 0;

    /// Wczytanie obrazu do pamięci programu
    if(p_source_image= device->LockNextImage(f_port, &FramesDropped, 10000))
    {
        ImagePtr p_target_img = VRmUsbCam::NewImage(final_target_format);

        VRmUsbCam::ConvertImage(p_source_image,p_target_img);

        /// Zamiana formatu kontenera na Mat
        cv::Mat *pImage = new cv::Mat(p_target_img.get()->get_ImageFormat().get_Size().m_height,
                                      p_target_img.get()->get_ImageFormat().get_Size().m_width,CV_8UC4,
                                      p_target_img->get_Buffer());

        /// Odblokowanie buffora kamery
        device->UnlockNextImage(p_source_image);

        cv::Mat outImage = pImage->clone();
        delete pImage;
        outImage.convertTo(outImage,CV_8UC3);
        p_source_image.reset();
        p_target_img.reset();
        /// Zamiana formatu koloru na format BGR
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
/// Dostosowanie wartości ekspozycji
void adjustExposure(VRmUsbCamCPP::DevicePtr& deviceL, VRmDWORD& f_portL,VRmUsbCamCPP::DevicePtr& deviceR , VRmDWORD& f_portR)
{
    /// Zmienna suwaka ekspozycji
    int exposure_slider = 100;

    /// Inicjalizacja okna z suwakiem
    cv::namedWindow("Camera exposure control", cv::WINDOW_AUTOSIZE);

    /// Deklaracja suwaka do edycji wartości
    char exposure[50];
    sprintf(exposure, "Exposure %d", 500);
    createTrackbar(exposure, "Camera exposure control", &exposure_slider, 500, trackbarLink);

    /// Adjust both camera exposure
    char key = waitKey(1) & 0xff;

    /// Deklaracja kontenerów obrazów
    Mat leftCam, rightCam;

    std::cout << "Adjusting exposure time for both cameras \n press 'n' to move on" << std::endl;

    /// Odczytanie stanu przetwarzania obrazu urządzenia
    if ((boost::get<bool>(deviceL->get_PropertyValue(VRM_PROPID_PLUGIN_IMAGE_PROCESSING_B))) &&
        (boost::get<bool>(deviceR->get_PropertyValue(VRM_PROPID_PLUGIN_IMAGE_PROCESSING_B))))
        std::cout << "Image processing Enabled" << std::endl;

    while (!(key == 'n')) {
        /// Odczytanie ramek obrazu
        leftCam = readSingleFrame(deviceL, f_portL);
        rightCam = readSingleFrame(deviceR, f_portR);

        /// Poprzednia wartość ekspozycji
        static float previousSample;

        /// Ustawienie nowej wartości ekspozycji
        if (previousSample != exposure_slider) {
            deviceL->set_PropertyValue(VRM_PROPID_CAM_EXPOSURE_TIME_F, (float) exposure_slider);
            deviceR->set_PropertyValue(VRM_PROPID_CAM_EXPOSURE_TIME_F, (float) exposure_slider);

        }
        previousSample = exposure_slider;

        /// Wizualizacja obrazu z kamer
        imshow("LeftCam", leftCam);
        imshow("RightCam", rightCam);

        /// Obsługa klawiszy do zamiany indeksow oraz wyjscia z pętli
        key = waitKey(1) & 0xff;
        if (key == 'i') invertCameras(deviceL, f_portL, deviceR, f_portR);
        else if (key == 'q') {
            CloseDevice(deviceL);
            CloseDevice(deviceR);
        }
    }
    destroyAllWindows();
}
/// Zamiana indeksów kamer
void invertCameras(VRmUsbCamCPP::DevicePtr& deviceL, VRmDWORD& f_portL,VRmUsbCamCPP::DevicePtr& deviceR, VRmDWORD& f_portR)
{
    /// Deklaracja tymczasowego wskaźnika i portu
    VRmUsbCamCPP::DevicePtr tempPtr = deviceR;
    VRmDWORD  tempPort = f_portR;

    /// Zamiana wskaźników i portów
    deviceR = deviceL;
    f_portR = f_portL;

    deviceL = tempPtr;
    f_portL = tempPort;

    std::cout << "Cameras indexes has been inverted!" << std::endl;
}

/// TODO Invert for LOGITECH

/// Zamknięcie połączenia z kamerą
void CloseDevice(DevicePtr &device)
{
    /// Zatrzymanie pracy kamery
    device->Stop();
    std::cout << "\nStopping device" << std::endl;

    /// Zamknięcie połączenia
    device->Close();
    std::cout << "\nClosing device" << std::endl;

    /// Reset wskaźnika kamery
    device.reset();

    /// Reset listy urządzeń
    VRmUsbCam::ResetDeviceKeyList();
    std::cout << "\nReseting device keylist" << std::endl;
}
/// Reset wskaźników i portów kamer
void resetDevice(DevicePtr &deviceL, DevicePtr &deviceR)
{
    std::cout << "Reseting cameras" << std::endl;

    /// Usunięcie ostatniego błędu kamery
    VRmUsbCamClearLastError();

    /// Zamknięcie kamer i reset wskaźników
    deviceL->Close();
    std::cout << "\nClosing device" << std::endl;

    deviceL.reset();

    deviceR->Close();
    std::cout << "\nClosing device" << std::endl;

    deviceR.reset();

    /// Reset sterowników
    VRmUsbCamCleanup();
    std::cout << "\nCelanup done" << std::endl;

    /// Reset listy wykrytych urządzeń
    VRmUsbCam::ResetDeviceKeyList();
    std::cout << "\nReseting device keylist" << std::endl;

    /// Pobranie nowej listy urządzeń
    VRmUsbCam::UpdateDeviceKeyList();
}

/// Metody kamer Logitech

/// Odczytanie obrazu lewej kamery
cv::Mat readLeftFrame()
{
    /// Deklaracja statycznego obiektu kamery
    static cv::VideoCapture leftCaputre(LEFT_INDEX);
    /// Deklaracja konteneru obrazu
    cv::Mat image;
    /// Sprawdzenie połączenia
    if(leftCaputre.isOpened())
        leftCaputre >> image;
    else {
        std::cout << "Video caputre not opened error" << std::endl;
        exit(-1);
    }
    static bool firstCall;

    if(!firstCall) {
        /// Wyłączenie opcji AutoFocus w kamerze
        leftCaputre.set(CV_CAP_PROP_AUTOFOCUS, 0);
        firstCall = true;
    }
    return image;
}

/// Odczytanie obrazu prawej kamery
cv::Mat readRightFrame()
{
    /// Deklaracja statycznego obiektu kamery
    static cv::VideoCapture rightCapture(RIGHT_INDEX);
    /// Deklaracja konteneru obrazu
    cv::Mat image;
    /// Sprawdzenie połączenia
    if(rightCapture.isOpened())
        rightCapture >> image;
    else {
        std::cout << "Video caputre not opened error" << std::endl;
        exit(-1);
    }
    static bool firstCall;

    /// Wyłączenie opcji AutoFocus w kamerze
    if(!firstCall) {
        rightCapture.set(CV_CAP_PROP_AUTOFOCUS, 0);
        firstCall = true;
    }
    return image;
}



