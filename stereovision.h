/*
 * Plik nagłówkowy zawierający implementacje algorytmu stereowizji
 */

/// Zabezpieczenie przed kilkukrotnym wczytaniem pliku nagłówkowego
#ifndef STEREOVISIONPROJECT_STEREOVISION_H
#define STEREOVISIONPROJECT_STEREOVISION_H

/// Dodanie bibliotek wykorzystywanych w projekcie
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <time.h>

#include <unistd.h>
#include <thread>
#include <iomanip>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <opencv2/ximgproc/edge_filter.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/viz.hpp>

#include <vrmusbcamcpp.h>

/// Wczytanie przestrzeni nazw bibliotek
using namespace std;
using namespace cv;
using namespace cv::ximgproc;

/// Deklaracja struktury parametrów kalibracji
typedef struct {

    Mat     left_K;
    Mat     left_D;
    Mat     right_K;
    Mat     right_D;
    Mat     R;
    Mat     E;
    Mat     F;
    Mat     left_R;
    Mat     right_R;
    Mat     left_P;
    Mat     right_P;
    Mat     Q;
    Vec3d   T;

}CalibrateParameters;

/// Deklaracja klasy stereovision
class stereovision {
public:
    /// Konstruktor inicjalizujacy klasy stereovision
    stereovision(VRmUsbCamCPP::DevicePtr _leftCamPtr,VRmUsbCamCPP::DevicePtr _rightCamPtr,VRmDWORD _leftPort,VRmDWORD _rightPort,float _squareSize, Size _board_sz, int _n_boards,bool _useWebCam
                ,int _ndisparities = 112,int _SADWindow = 9,int _minDisparity = 200,int _uniqRatio = 1,int _speckWinSize = 150,int _speckRange = 2,int _dispMaxDiff = 10000
                ,int _preFilterCap = 63, int _preFilterSize = 5, int _colorScale = 300) :
        leftCamPtr(_leftCamPtr),
        rightCamPtr(_rightCamPtr),
        leftPort(_leftPort),
        rightPort(_rightPort),
        squareSize(_squareSize),
        board_sz(_board_sz),
        n_boards(_n_boards),
        useWebCam(_useWebCam),
        ndisparities(_ndisparities),
        SADWindowSize(_SADWindow),
        minDisparity(_minDisparity),
        uniqRatio(_uniqRatio),
        speckWinSize(_speckWinSize),
        speckRange(_speckRange),
        dispMaxDiff(_dispMaxDiff),
        preFilterCap(_preFilterCap),
        preFilterSize(_preFilterSize),
        lambda(4500),
        sigma(150),
        colorScale(_colorScale)
            {}

    /// Destruktor klasy
    ~stereovision() {}

    /// Deklaracja struktury parametrów wewnątrz klasy
    CalibrateParameters calibParam;

    /// Publiczne funkcje klasy
    bool calibrateStereo (const string& _fileName);

    bool loadCalibParam(const string &_fileName);

    bool rectifyImages(Mat& leftImage, Mat& rightImage, Mat &_outLeftImage, Mat &_outRightImage, Mat &_bothImages);

    bool computeSGBM(Mat &_leftRectImage, Mat &_rightRectImage, Mat &_referenceImage);

    bool computeBM(Mat &_leftRectImage, Mat &_rightRectImage, Mat &_referenceImage);

    bool displayTrackbar();

    bool readFromImage(Mat &_leftRectImage, Mat &_rightRectImage);

    Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance);
    /// Zmienna typu Rect używane w wybieraniu badanego fragmentu obrazu
    Rect mouseInput;

private:
    /// Prywatne funnkcje klasy
    void visualizeResults(const Mat& _rawDisparity, const Mat& _filteredDisparity);

    float reprojectTo3DPoint(const Rect& ROI,const Mat& _disparityMap);

    void visualizePointCloud(const Mat& _disparityMap, const Mat& _colorMap);

    /// Zmienne kalibracyjne
    float squareSize;
    Size board_sz;
    int n_boards;

    /// Wskaźniki i porty kamer VRmUSB
    VRmUsbCamCPP::DevicePtr leftCamPtr;
    VRmUsbCamCPP::DevicePtr rightCamPtr;
    VRmDWORD leftPort;
    VRmDWORD rightPort;

    /// Bit sygnalizujący wybór rodzaju
    bool useWebCam;

    /// Zmienne wykorzystywane w budowaniu mapy dysparycji
    int ndisparities;
    int SADWindowSize;
    int minDisparity;
    int uniqRatio;
    int speckWinSize;
    int speckRange;
    int dispMaxDiff;
    int preFilterCap;
    int preFilterSize;
    int textureTreshold;
    int lambda;
    int sigma;
    int colorScale;
    Rect ROI1,ROI2;
};

/// Funkcja do obsługi zmiany parametrów w oknie edycji
void trackbarLink(int v, void *ptr);

/// Obsługa myszki w oknie "Result"
void onMouseRectCallback(int _evt, int _x, int _y, int _flags, void *_param);

#endif //STEREOVISIONPROJECT_STEREOVISION_H

