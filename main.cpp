/// Dodanie plików nagłówkowych
#include "camerahandle.h"
#include "stereovision.h"

/// Wczytanie przestrzeni nazw bibliotek
using namespace cv;
using namespace std;
using namespace VRmUsbCamCPP;

/// Deklaracja możliwych parametrów wejściowych
const String keys =
                "{help h usage ? |                                | Help message - Input parameters }"
                "{board_w        |9                               | Board width                     }"
                "{board_h        |6                               | Board height                    }"
                "{n_boards       |12                              | Number of board pictures        }"
                "{squaresize     |0.026                           | Size of board square [m]        }"
                "{calibrate      |                                | Calibrate cameras               }"
                "{paramsFilename |../Parameters/CameraParams.yml  | Path to parameters file         }"
                "{reset          |                                | Reset cameras                   }"
                "{algorithm      |SGBM                            | Select stereo algorithm         }"
                "{r              |                                | Read from files                 }"
                "{webcam         |                                | Use webcam                      }"
;

int main(int argc, char const *argv[])
{
    /// Deklaracja obiektu wczytującego parametry wejściowe
    CommandLineParser parser (argc, argv, keys);
    parser.about("Stereo Vision application");

    /// Obsługa pomocy użytkownika
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    else if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }

    std::cout<< "========================================================" << std::endl
             << "===               Stereo Vision Project              ===" << std::endl
             << "========================================================" << std::endl;

    /// Wczytanie parametrów do pamięci programu
    int board_w = parser.get<int>("board_w");
    int board_h = parser.get<int>("board_h");
    cv::Size board_sz = cv::Size( board_w, board_h);
    int n_boards = parser.get<int>("n_boards");
    float squaresize = parser.get<float>("squaresize");
    string algorithm = parser.get<string>("algorithm");
    bool readFile = parser.has("r");
    bool useWebCam = parser.has("webcam");
    bool calibrate = parser.has("calibrate");
    bool resetCameras = parser.has("reset");
    string paramsFilename = parser.get<string>("paramsFilename");

    cout << "Parameters file directory: " << paramsFilename << endl;

    /// Deklaracja obiektów obsługujących porty oraz kamere VRmUSB
    DevicePtr deviceL;
    DevicePtr deviceR;
    VRmDWORD portL = 0;
    VRmDWORD portR = 0;

    /// Warunek sprawdzający obsługe wybranych kamer
    if (!useWebCam && !readFile) {
        /// Inicjalizacja wskaźników kamer
        openCamera(deviceL, deviceR);
        std::cout << "Both cameras opened!" << std::endl;

        /// Wczytanie wewnętrznej konfiguracji kamer o indeksie 1;
        deviceL->LoadConfig(1);
        deviceR->LoadConfig(1);

        /// Pobranie formatu źródłowego sensora
        ImageFormat source_format_L = deviceL->get_SourceFormat(deviceL->get_SensorPortList()[0]);
        ImageFormat source_format_R = deviceR->get_SourceFormat(deviceR->get_SensorPortList()[0]);

        /// Inicjalizacja połączenia z kamerami
        /// wraz z zadaniem docelowego formatu obrazu
        ImageFormat target_format;

        initCamera(deviceL, portL, VRM_ARGB_4X8, target_format);
        std::cout << "Left camera initiated" << std::endl;
        initCamera(deviceR, portR, VRM_ARGB_4X8, target_format);
        std::cout << "Right camera initiated" << std::endl;


        /// Obsługa resetu kamer
        if (resetCameras) {
            resetDevice(deviceL,deviceR);
            return 0;
        }

        /// Właściwe połączenie z kamerami
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

        /// Funkcja obsługująca zmiane ekspozycji dla obu kamer
        adjustExposure(deviceL,portL,deviceR,portR);
    }

    /// Inicjalizacja obiektu klasy stereovision
    stereovision stereoV(deviceL, deviceR, portL, portR, squaresize, board_sz, n_boards,useWebCam);


    /// Kalibracja Kamer lub wczytanie parametrow z pliku
    if(!readFile) {
        if (calibrate) {
            if (!stereoV.calibrateStereo(paramsFilename)) {
                std::cout << "Stereo calibration failed \n Exiting program" << std::endl;
            }
        } else stereoV.loadCalibParam(paramsFilename);
    }
    destroyAllWindows();

    /// Uwtorzenie panelu sterowania parametrami algorytmów stereowizji
    stereoV.displayTrackbar();

    /// Deklaracja okna wynikowego wraz z odwołaniem do obsługi myszki
    namedWindow("Disparity result", CV_WINDOW_NORMAL);
    setWindowProperty("Disparity result", CV_WND_PROP_ASPECTRATIO , CV_WINDOW_KEEPRATIO );
    if(!readFile)
    cv::setMouseCallback("Disparity result", onMouseRectCallback, (void*)&stereoV.mouseInput);

    /// Główna pętla programu
    for(;;)
    {
        /// Zmienna do obliczenia czasu trwania jednego cyklu programu
        double programTime = (double)getTickCount();
        /// Inicjalizacja kontenerów obrazów wykorzystywanych w przetwarzaniu
        Mat leftCam, rightCam,leftRectImage, rightRectImage,bothImages;

        /// Wczytanie klatek obrazu do pamięci programu
        if(!readFile) {
            try {
                if(useWebCam) {
                    leftCam = readLeftFrame();
                    rightCam = readRightFrame();
                }
                else {
                    leftCam = readSingleFrame(deviceL, portL);
                    rightCam = readSingleFrame(deviceR, portR);
                }
            }
            catch (exception exc) {
                if(!useWebCam) {
                    CloseDevice(deviceL);
                    CloseDevice(deviceR);
                }
                std::cout << "Error while capturing image occured: " << exc.what() << std::endl;
                return -1;
            }
        } else
            {
            stereoV.readFromImage(leftRectImage,rightRectImage);
            stereoV.readFromImage(leftCam,rightCam);
            }

        /// Funkcja do usuwania zniekształceń oraz rektyfikacji obrazów
        if(!readFile) {
            if (!stereoV.rectifyImages(leftCam, rightCam, leftRectImage, rightRectImage, bothImages)) {
                std::cout << "Can not perform rectification" << std::endl;
            }
        }
        /// Algorytm pół globalny
        if(algorithm == "SGBM") {
            std::cout << "Semi Global Matching Algorithm" << std::endl;
            if (!stereoV.computeSGBM(leftRectImage, rightRectImage, leftCam)) {
                std::cout << "Can not compute disparity map" << std::endl;
            }
        }
        /// Algorytm globalny
        if(algorithm == "BM") {
            std::cout << "Global Matching Algorithm" << std::endl;
            if (!stereoV.computeBM(leftRectImage, rightRectImage, leftCam)) {
                std::cout << "Can not compute disparity map" << std::endl;
            }
        }
        /// Wyświetlenie obrazów z kamer
        if(!readFile) {
            Mat bothCam;
            hconcat(leftCam,rightCam,bothCam);
            imshow("Input images", bothCam);
            imshow("Rectified images", bothImages);
        }

        /// Odczytanie czasu trwania jednego cyklu programu
        programTime = ((double)getTickCount() - programTime)/getTickFrequency();
        cout.precision(2);
        cout<<"Application time: "<<programTime<<"s"<<endl;
        cout << endl;

        /// Obsługa klawiszy do odwrócenia indeksów kamer lub wyłączenia programu
        char key = waitKey(1) & 0xff;
        if(key == 'i') invertCameras(deviceL,portL,deviceR,portR);
        else if(key =='q') break;
    }

    /// Wyłączenie kamer VRmUSB
    if(!useWebCam && !readFile) {
        CloseDevice(deviceL);
        CloseDevice(deviceR);
    }

    std::cout << "Program finished" << std::endl;

    return 0;
}
