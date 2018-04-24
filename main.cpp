#include "camerahandle.h"
#include "stereovision.h"

using namespace cv;
using namespace std;
using namespace VRmUsbCamCPP;


const String keys =
                "{help h usage ? |                             | print this message          }"
                "{board_w        |9                            | board width                 }"
                "{board_h        |6                            | board height                }"
                "{n_boards       |12                           | number of board pic         }"
                "{squaresize     |0.026                        | size of board square         }"
                "{calibrate      |                             | Calibrate?                  }"
                "{paramsFilename |../Parameters/CameraParams.yml  | path to param file  }"
                "{reset          |                             | reset cameras               }"
                "{algorithm      |SGBM                         | select stereo algorithm     }"
                "{r              |                             | read from files?            }"
                "{webcam         |                             | UseWebCam                   }"
;

int exposure_slider;


int main(int argc, char const *argv[])
{
    std::cout<< "========================================================" << std::endl
             << "===               Stereo Vision Project              ===" << std::endl
             << "========================================================" << std::endl;


    CommandLineParser parser (argc, argv, keys);
    parser.about("Stereo Vision application");

    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }


    int board_w = parser.get<int>("board_w");    // number of corners in row
    int board_h = parser.get<int>("board_h");    // number of corners in column
    cv::Size board_sz = cv::Size( board_w, board_h);
    int n_boards = parser.get<int>("n_boards");
    float squaresize = parser.get<float>("squaresize");
    string algorithm = parser.get<string>("algorithm");
    bool readFile = parser.has("r");
    bool useWebCam = parser.has("webcam");
    bool calibrate = parser.has("calibrate");
    bool resetCameras = parser.has("reset");
    string paramsFilename = parser.get<string>("paramsFilename");

    cout << paramsFilename << endl;

    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }

    DevicePtr deviceL;
    DevicePtr deviceR;
    VRmDWORD portL = 0;
    VRmDWORD portR = 0;

    if (!useWebCam) {
        /// Initialize camera pointers
        if (!readFile) {
            openCamera(deviceL, deviceR);
            std::cout << "Both cameras opened!" << std::endl;

            deviceL->LoadConfig(1);
            deviceR->LoadConfig(1);

            ///get source format of first attached sensor
            ImageFormat source_format_L = deviceL->get_SourceFormat(deviceL->get_SensorPortList()[0]);
            ImageFormat source_format_R = deviceR->get_SourceFormat(deviceR->get_SensorPortList()[0]);
        }

        ///Desired target format to convert to
        VRmColorFormat screen_color_format = VRM_ARGB_4X8;

        /// init camera, change some settings...
        /// we get a target_format in return, which is necessary to initialize our
        /// viewer window
        ImageFormat target_format;

        if (!readFile) {
            initCamera(deviceL, portL, screen_color_format, target_format);
            std::cout << "Left camera initiated" << std::endl;
            initCamera(deviceR, portR, screen_color_format, target_format);
            std::cout << "Right camera initiated" << std::endl;
        }

        if (resetCameras) {
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

            return 0;
        }

        if (!readFile) {
            while (!deviceL->get_Running()) {
                deviceL->ResetFrameCounter();
                deviceL->Start();
            }
            std::cout << "Left camera started " << std::endl;

            while (!deviceR->get_Running()) {
                deviceR->ResetFrameCounter();
                deviceR->Start();
            }
            std::cout << "Right camera started " << std::endl;
        }


        /// Camera parameters control window
        exposure_slider = 100;

        namedWindow("Camera exposure control", WINDOW_AUTOSIZE);

        char exposure[50];
        sprintf(exposure, "Exposure %d", 500);
        createTrackbar(exposure, "Camera parameters control", &exposure_slider, 500, trackbarLink);

        /// Adjust both camera exposure
        char key = waitKey(1) & 0xff;
        Mat leftCam, rightCam;

        if (!readFile) {
            std::cout << "Adjusting exposure time for both cameras \n press 'n' to move on" << std::endl;

            if ((boost::get<bool>(deviceL->get_PropertyValue(VRM_PROPID_PLUGIN_IMAGE_PROCESSING_B))) &&
                (boost::get<bool>(deviceR->get_PropertyValue(VRM_PROPID_PLUGIN_IMAGE_PROCESSING_B))))
                std::cout << "Image processing Enabled" << std::endl;

            while (!(key == 'n')) {

                leftCam = readSingleFrame(deviceL, portL);
                rightCam = readSingleFrame(deviceR, portR);

                float previousSample;

                if (previousSample != exposure_slider) {
                    deviceL->set_PropertyValue(VRM_PROPID_CAM_EXPOSURE_TIME_F, (float) exposure_slider);
                    deviceR->set_PropertyValue(VRM_PROPID_CAM_EXPOSURE_TIME_F, (float) exposure_slider);

                }
                previousSample = exposure_slider;

                imshow("LeftCam", leftCam);
                imshow("RightCam", rightCam);

                key = waitKey(1) & 0xff;
                if (key == 'i') invertCameras(deviceL, portL, deviceR, portR);
                else if (key == 'q') {
                    CloseDevice(deviceL);
                    CloseDevice(deviceR);
                    return 0;
                }
            }
        }
        destroyAllWindows();
    } else {

    }

    ///initialize stereovision object
    stereovision stereoV(deviceL, deviceR, portL, portR, squaresize, board_sz, n_boards,useWebCam);
    Mat leftCam, rightCam;

    ///Left camera Calibration
    if(!readFile) {
        if (calibrate) {
            if (!stereoV.calibrateStereo(paramsFilename)) {
                std::cout << "Stereo calibration failed \n Exiting program" << std::endl;
            }
        } else stereoV.loadCalibParam(paramsFilename);
    }

    destroyAllWindows();

    /// Display Trackbar
    stereoV.displayTrackbar();

    /// Set up mouse callback
    namedWindow("Disparity result", CV_WINDOW_NORMAL);
    setWindowProperty("Disparity result", CV_WND_PROP_ASPECTRATIO , CV_WINDOW_KEEPRATIO );
    cv::setMouseCallback("Disparity result", onMouseRectCallback, (void*)&stereoV.mouseInput);

    /// Camera Rectify and Undistort
    bool firstCall = true;

    for(;;)
    {
        double programTime = (double)getTickCount();
        Mat leftRectImage, rightRectImage;
        Mat bothImages;


        if(!readFile) {
            try {
                double imageCaputre = (double)getTickCount();
                if(useWebCam) {
                    leftCam = readLeftFrame();
                    rightCam = readRightFrame();
                }
                else {
                    leftCam = readSingleFrame(deviceL, portL);
                    rightCam = readSingleFrame(deviceR, portR);
                }
                imageCaputre = ((double)getTickCount() - imageCaputre)/getTickFrequency();
                cout.precision(2);
                cout<<"Image caputirng time: "<<imageCaputre<<"s"<<endl;
            }
            catch (exception exc) {
                if(!useWebCam) {
                    CloseDevice(deviceL);
                    CloseDevice(deviceR);
                }
                std::cout << "Error while capturing image occured: " << exc.what() << std::endl;
                return -1;
            }
        }
        else{
            stereoV.readFromImage(leftRectImage,rightRectImage, firstCall);
            stereoV.readFromImage(leftCam,rightCam, firstCall);
        }


        double rectifyTime = (double)getTickCount();
        if(!readFile) {
            if (!stereoV.recityfImages(leftCam, rightCam, leftRectImage, rightRectImage, bothImages)) {
                std::cout << "Can not perform rectification" << std::endl;
            }
        }
        rectifyTime = ((double)getTickCount() - rectifyTime)/getTickFrequency();
        cout.precision(2);
        cout<<"Rectify time: "<<rectifyTime<<"s"<<endl;


        double algorithmTime = (double)getTickCount();
        if(algorithm == "SGBM") {
            std::cout << "Semi Global Matching Algorithm" << std::endl;
            if (!stereoV.computeSGBM(leftRectImage, rightRectImage, leftCam)) {
                std::cout << "Can not compute disparity map" << std::endl;
            }
            algorithmTime = ((double)getTickCount() - algorithmTime)/getTickFrequency();
            cout.precision(2);
            cout<<"SGBM Time: "<<algorithmTime<<"s"<<endl;
        }
        if(algorithm == "BM") {
            std::cout << "Global Matching Algorithm" << std::endl;
            if (!stereoV.computeBM(leftRectImage, rightRectImage, leftCam)) {
                std::cout << "Can not compute disparity map" << std::endl;
            }
            algorithmTime = ((double)getTickCount() - algorithmTime)/getTickFrequency();
            cout.precision(2);
            cout<<"BM Time: "<<algorithmTime<<"s"<<endl;
        }


        if(!readFile) {
            Mat bothCam;
            hconcat(leftCam,rightCam,bothCam);
            imshow("Input images", bothCam);
            imshow("Rectified images", bothImages);
        }

        double aperWidth, aperHeight, fovx, fovy, focalLength,aspectRatio;
        Point2d principPoint;
        calibrationMatrixValues(stereoV.calibParam.left_K,leftRectImage.size(),aperWidth,aperHeight,fovx,fovy, focalLength, principPoint, aspectRatio);

        setprecision(4);
        cout << "Focal length: " << focalLength << endl;

        programTime = ((double)getTickCount() - programTime)/getTickFrequency();
        cout.precision(2);
        cout<<"Application time: "<<programTime<<"s"<<endl;
        cout << endl;

        firstCall = false;
        char key = waitKey(1) & 0xff;
        if(key == 'i') invertCameras(deviceL,portL,deviceR,portR);
        else if(key =='q') break;


    }


    ///Reset and close devices
    CloseDevice(deviceL);
    CloseDevice(deviceR);

    std::cout << "exit." << std::endl;

    return 0;
}
