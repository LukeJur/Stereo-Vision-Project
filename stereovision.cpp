#include "stereovision.h"
#include "camerahandle.h"


Rect stereovision::computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
{
    int min_disparity = matcher_instance->getMinDisparity();
    int num_disparities = matcher_instance->getNumDisparities();
    int block_size = matcher_instance->getBlockSize();

    int bs2 = block_size/2;
    int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

    int xmin = maxD + bs2;
    int xmax = src_sz.width + minD - bs2;
    int ymin = bs2;
    int ymax = src_sz.height - bs2;

    Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
    return r;
}

void record(cv::Mat &image)
{
    static cv::VideoWriter writer;
    if (!writer.isOpened())
    {
        int codec = CV_FOURCC('M','J','P','G');
        double fps = 15.0;
        writer.open("sv_result.avi",codec,fps,image.size());
    }
    if(writer.isOpened())
    {
        writer.write(image);
    }
    else
        printf("wideo write failed\n");
}

bool stereovision::calibrateStereo(const string &_fileName)
{
    /// Left camera calibration - extracting intrinisic and distortion coeff
    Mat leftImage;
    char key;
    bool repeat;
    vector< vector<cv::Point2f> > image_points;
    vector< vector<cv::Point3f> > object_points;
    cv::Mat left_intrinsic_matrix, left_dist_coeffs;


    cout << "Calibrating left camera..." << endl;

    double last_timestamp = 0;
    do {
        while (image_points.size() < (size_t) n_boards - 4) {

            try {
                if(this->useWebCam){
                    leftImage = readLeftFrame();
                }
                else {
                    leftImage = readSingleFrame(leftCamPtr, leftPort);
                }
            }
            catch (exception exc) {
                if(!this->useWebCam) {
                    CloseDevice(leftCamPtr);
                    CloseDevice(rightCamPtr);
                }
                std::cout << "Error while capturing left image occured: " << exc.what() << std::endl;
                return -1;
            }


            Mat temp;
            Mat grayL;
            vector<cv::Point2f> cornersL;
            bool foundL;

            cv::cvtColor(leftImage, grayL, CV_BGR2GRAY);
            Mat leftImageVisual = leftImage.clone();

            foundL = cv::findChessboardCorners(leftImage, board_sz, cornersL,
                                               CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
            if (foundL) {
                cv::cornerSubPix(grayL, cornersL, cv::Size(5, 5), cv::Size(-1, -1),
                                 cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
                drawChessboardCorners(leftImageVisual,board_sz,cornersL,foundL);
            }


            double timestamp = (double) clock() / CLOCKS_PER_SEC;

            key = waitKey(1) & 0xff;


            if (foundL && ((timestamp - last_timestamp > 0.1) && key == 'n')) {
                last_timestamp = timestamp;

                vector<Point3f> obj;
                for (int i = 0; i < board_sz.height; i++)
                    for (int j = 0; j < board_sz.width; j++)
                        obj.push_back(Point3f((float) j * squareSize, (float) i * squareSize, 0.f));

                if (!cornersL.empty() && !obj.empty()) {
                    image_points.push_back(cornersL);
                    object_points.push_back(obj);
                }

                std::cout << "Collected " << (int) image_points.size() << " from " << n_boards << " required \n"
                          << std::endl;
            }

            cv::imshow("Left Camera", leftImageVisual);
            if ((cv::waitKey(25) & 255) == 257)
                return -1;
        }

        std::cout << "Left camera data collected" << std::endl;

        /// Get parameters
        double err = cv::calibrateCamera(object_points, image_points, leftImage.size(), left_intrinsic_matrix,
                                         left_dist_coeffs, cv::noArray(), cv::noArray(),
                                         cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT);


        std::cout << "Parameters Calculated with error: " << err << std::endl;
        std::cout << "Press 'r' to repeat else press 'b' " << std::endl;

        std::cin >> key;
        std::cout << key << std::endl;

        if (key == 'r') {
            std::cout <<"Repeating calibration" << std::endl;
            repeat = true;
        }


        if(key == 'b')
            repeat = false;

        image_points.clear();
        image_points.begin();
        object_points.clear();
        object_points.begin();

    }while(repeat);

    std::cout << "Parameters (Left cam) saved" << std::endl;

    destroyWindow("Left Camera");


    /// Right camera calibration - extracting intrinisic and distortion coeff

    Mat rightImage;
    last_timestamp = 0;
    cv::Mat right_intrinsic_matrix, right_dist_coeffs;

    cout << "Calibrating right camera..." << endl;

    do {

        while (image_points.size() < (size_t) n_boards - 4) {

            try {
                if(this->useWebCam) {
                    rightImage = readRightFrame();
                }
                else{
                    rightImage = readSingleFrame(rightCamPtr, rightPort);
                }
            }
            catch (exception exc) {
                if(!this->useWebCam) {
                    CloseDevice(leftCamPtr);
                    CloseDevice(rightCamPtr);
                }
                std::cout << "Error while capturing right image occured: " << exc.what() << std::endl;
                return -1;
            }

            Mat temp;
            Mat grayR;
            vector<cv::Point2f> cornersR;
            bool foundR;

            cv::cvtColor(rightImage, grayR, CV_BGR2GRAY);
            Mat rightImageVisual = rightImage.clone();

            foundR = cv::findChessboardCorners(rightImage, board_sz, cornersR,
                                               CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
            if (foundR) {
                cv::cornerSubPix(grayR, cornersR, cv::Size(5, 5), cv::Size(-1, -1),
                                 cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
                drawChessboardCorners(rightImageVisual,board_sz,cornersR,foundR);
            }

            double timestamp = (double) clock() / CLOCKS_PER_SEC;

            key = waitKey(1) & 0xff;


            if (foundR && (timestamp - last_timestamp > 0.1 && key == 'n')) {
                last_timestamp = timestamp;

                vector<Point3f> obj;
                for (int i = 0; i < board_sz.height; i++)
                    for (int j = 0; j < board_sz.width; j++)
                        obj.push_back(Point3f((float) j * squareSize, (float) i * squareSize, 0.f));

                if (!cornersR.empty() && !obj.empty()) {
                    image_points.push_back(cornersR);
                    object_points.push_back(obj);
                }
                std::cout << "Collected " << (int) image_points.size() << " from " << n_boards << " required \n"
                          << std::endl;
            }


            cv::imshow("Right Camera", rightImageVisual);

            if ((cv::waitKey(25) & 255) == 257)
                return -1;
        }

        std::cout << "Right camera data collected" << std::endl;

        /// Get parameters
        double errR = cv::calibrateCamera(object_points, image_points, rightImage.size(), right_intrinsic_matrix,
                                          right_dist_coeffs, cv::noArray(), cv::noArray(),
                                          cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT);


        std::cout << "Parameters Calculated with error: " << errR << std::endl;

        std::cout << "Press 'r' to repeat else press 'b' " << std::endl;

        std::cin >> key;
        std::cout << key << std::endl;

        if (key == 'r') {
            std::cout <<"Repeating calibration" << std::endl;
            repeat = true;
        }

        if(key == 'b')
            repeat = false;

        image_points.clear();
        image_points.begin();
        object_points.clear();
        object_points.begin();


    }while (repeat);

    std::cout << "Parameters (right cam) saved" << std::endl;

    destroyWindow("Right Camera");

    /// Proper stereo calib
    Mat temp;
    Mat grayR,grayL;
    Mat R, E, F;
    Vec3d T;
    Mat left_R, right_R, left_P, right_P, Q;
    vector<cv::Point2f> cornersL;
    vector< vector<cv::Point2f> > left_image_points;
    vector<cv::Point2f> cornersR;
    vector< vector<cv::Point2f> > right_image_points;


    cout << "Calibrating stereo system..." << endl;
    do {
        while (left_image_points.size() < (size_t) n_boards) {

            try {
                if(this->useWebCam) {
                    leftImage = readLeftFrame();
                    rightImage = readRightFrame();
                } else {
                        leftImage = readSingleFrame(leftCamPtr, leftPort);
                        rightImage = readSingleFrame(rightCamPtr, rightPort);
                }
            }
            catch (exception exc) {
                if(!this->useWebCam) {
                    CloseDevice(leftCamPtr);
                    CloseDevice(rightCamPtr);
                }
                std::cout << "Error while capturing left image occured: " << exc.what() << std::endl;
                return -1;
            }
            cv::cvtColor(leftImage, grayL, CV_BGR2GRAY);
            cv::cvtColor(rightImage, grayR, CV_BGR2GRAY);
            Mat leftImageVisual = leftImage.clone();
            Mat rightImageVisual = rightImage.clone();
            bool foundL;
            bool foundR;


            foundL = cv::findChessboardCorners(leftImage, board_sz, cornersL,
                                               CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

            foundR = cv::findChessboardCorners(rightImage, board_sz, cornersR,
                                               CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

            if (foundL && foundR) {
                cv::cornerSubPix(grayL, cornersL, cv::Size(5, 5), cv::Size(-1, -1),
                                 cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

                cv::cornerSubPix(grayR, cornersR, cv::Size(5, 5), cv::Size(-1, -1),
                                 cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

                drawChessboardCorners(leftImageVisual,board_sz,cornersL,foundL);

                drawChessboardCorners(rightImageVisual,board_sz,cornersR,foundR);
            }



            cv::imshow("Left Camera", leftImageVisual);
            cv::imshow("Right Camera", rightImageVisual);

            if ((cv::waitKey(25) & 255) == 257)
                return -1;


            double timestamp = (double) clock() / CLOCKS_PER_SEC;



             key = waitKey(1) & 0xff;

            if (foundL && foundR && timestamp - last_timestamp > 3 && key == 'n') {


                vector<Point3f> obj;
                for (int i = 0; i < board_sz.height; i++)
                    for (int j = 0; j < board_sz.width; j++)
                        obj.push_back(Point3f((float) j * squareSize, (float) i * squareSize, 0.f));

                if (!cornersR.empty() && !cornersR.empty() && !obj.empty()) {
                    left_image_points.push_back(cornersL);
                    right_image_points.push_back(cornersR);
                    object_points.push_back(obj);
                }
                std::cout << "Collected " << (int) left_image_points.size() << " from " << n_boards << " required \n"
                          << std::endl;

                last_timestamp = timestamp;
            }
        }

        destroyAllWindows();


        if (object_points.size() != 0 && left_image_points.size() != 0 && right_image_points.size() != 0) {
            double calibStereoError;

            int flag = 0;
            flag |= CALIB_FIX_K3;
            flag |= CALIB_ZERO_TANGENT_DIST;
            flag |= CALIB_FIX_INTRINSIC;

            calibStereoError = stereoCalibrate(object_points, left_image_points, right_image_points,
                                               left_intrinsic_matrix, left_dist_coeffs,
                                               right_intrinsic_matrix, right_dist_coeffs, leftImage.size(), R, T, E, F,flag);

            std::cout << "Stereo calib error: " << calibStereoError << endl;
            std::cout << "Stereo calibration. Done." << endl;

            std::cout << "Starting Rectification parameters computing\n";

            stereoRectify(left_intrinsic_matrix, left_dist_coeffs, right_intrinsic_matrix, right_dist_coeffs,
                          leftImage.size(), R, T, left_R, right_R, left_P, right_P, Q, CALIB_ZERO_DISPARITY, -1,
                          leftImage.size(), &ROI1, &ROI2);

//            stereoRectify(left_intrinsic_matrix, left_dist_coeffs, right_intrinsic_matrix, right_dist_coeffs,
//                          leftImage.size(), R, T, left_R, right_R, left_P, right_P, Q);

            std::cout << "Rectification. Done." << endl;
        }
    else
    {
        std::cout<<"Stereo calibration cannot be done!!!"<<endl<<endl;
        return false;
    }

        std::cout << "Press 'r' to repeat else press 'b' " << std::endl;
        key = 0;
        std::cin >> key;
        std::cout << key << std::endl;

        if (key == 'r') {
            std::cout <<"Repeating calibration" << std::endl;
            repeat = true;
        }


        if(key == 'b')
            repeat = false;

        left_image_points.clear();
        right_image_points.clear();
        image_points.clear();
        image_points.begin();
        object_points.clear();
        object_points.begin();

    }while(repeat);

    calibParam.left_K = left_intrinsic_matrix;
    calibParam.left_D = left_dist_coeffs;
    calibParam.right_K = right_intrinsic_matrix;
    calibParam.right_D = right_dist_coeffs;
    calibParam.R = R;
    calibParam.T = T;
    calibParam.E = E;
    calibParam.F = F;
    calibParam.left_P = left_P;
    calibParam.left_R = left_R;
    calibParam.right_P = right_P;
    calibParam.right_R = right_R;



    //Save parameters to external file
    cv::FileStorage fs(_fileName, FileStorage::WRITE);
    fs << "left_camera_matrix" << left_intrinsic_matrix << "left_distortion_coefficients" << left_dist_coeffs;
    fs << "right_camera_matrix" << right_intrinsic_matrix << "right_distortion_coefficients" << right_dist_coeffs;
    fs << "rotation_matrix" << R << "translation_vector" << T;
    fs << "essential_matrix" << E << "fundamental_matrix" << F;
    fs << "rectification_rotation_left" << left_R << "rectification_rotation_right" << right_R;
    fs << "left_projection" << left_P << "right_projection" << right_P;
    fs << "reprojection_matrix" << Q ;
    fs.release();


    return 1;

}

bool stereovision::loadCalibParam(const string &_fileName)
{

    cv::FileStorage input_file(_fileName, cv::FileStorage::READ);

    input_file["left_camera_matrix"] >> this->calibParam.left_K;
    input_file["left_distortion_coefficients"] >> this->calibParam.left_D;
    input_file["right_camera_matrix"] >> this->calibParam.right_K;
    input_file["right_distortion_coefficients"] >> this->calibParam.right_D;
    input_file["rotation_matrix"] >> this->calibParam.R;
    input_file["essential_matrix"] >> this->calibParam.E;
    input_file["fundamental_matrix"] >> this->calibParam.F;
    input_file["rectification_rotation_left"] >> this->calibParam.left_R;
    input_file["rectification_rotation_right"] >> this->calibParam.right_R;
    input_file["left_projection"] >> this->calibParam.left_P;
    input_file["right_projection"] >> this->calibParam.right_P;
    input_file["reprojection_matrix"] >> this->calibParam.Q;
    input_file["translation_vector"] >> this->calibParam.T;

    input_file.release();

    return 1;
}

bool stereovision::recityfImages(Mat &_leftImage, Mat &_rightImage, Mat &_outLeftImage, Mat &_outRightImage, Mat &_bothImages) {

    Mat leftMapX,leftMapY,rightMapX,rightMapY;
    Mat leftUndistort, rightUndistort;

    cv::initUndistortRectifyMap(this->calibParam.left_K, this->calibParam.left_D, this->calibParam.left_R,
                                this->calibParam.left_P, _leftImage.size(), CV_32FC1, leftMapX, leftMapY);
    cv::initUndistortRectifyMap(this->calibParam.right_K, this->calibParam.right_D, this->calibParam.right_R,
                                this->calibParam.right_P, _leftImage.size(), CV_32FC1, rightMapX, rightMapY);

    cv::remap(_leftImage, leftUndistort, leftMapX, leftMapY, cv::INTER_LINEAR);
    cv::remap(_rightImage, rightUndistort, rightMapX, rightMapY, cv::INTER_LINEAR);



    _outLeftImage = leftUndistort.clone();
    _outRightImage = rightUndistort.clone();

    hconcat(leftUndistort,rightUndistort,_bothImages);

    for(int lineInterval = 10; lineInterval < _bothImages.rows; lineInterval += 40)
    {
        Point2i l(0, lineInterval), r(_bothImages.cols, lineInterval);
        line(_bothImages,l,r,(255,0,0),2);
    }
    return 1;
}

bool stereovision::computeSGBM(Mat &_leftRectImage, Mat &_rightRectImage, Mat &_referenceImage){

    Mat left_for_matcher, right_for_matcher;
    Mat left_disp,right_disp;
    Mat filtered_disp;
    Mat conf_map = Mat(_leftRectImage.rows,_leftRectImage.cols,CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    Ptr<DisparityWLSFilter> wls_filter;
    double matching_time, filtering_time;

    Size img_size = _referenceImage.size();

    Mat disp,disp8;

    this->SADWindowSize = this->SADWindowSize > 3 ? this->SADWindowSize : 5;
    this->ndisparities = this->ndisparities > 0 ? this->ndisparities : ((img_size.width/8) + 15) & -16;

    if (this->SADWindowSize % 2 == 0)
        this->SADWindowSize += 1;

    while (!((this->ndisparities % 16) == 0) || this->ndisparities == 0)
        this->ndisparities += 1;

    left_for_matcher = _leftRectImage.clone();
    right_for_matcher = _rightRectImage.clone();


    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,this->ndisparities,this->SADWindowSize);

    int cn = _leftRectImage.channels();
    int P1 = 8*cn*this->SADWindowSize*this->SADWindowSize, P2 = 32*cn*this->SADWindowSize*this->SADWindowSize;

    sgbm->setP1(24*this->SADWindowSize*this->SADWindowSize);
    sgbm->setP2(96*this->SADWindowSize*this->SADWindowSize);
    sgbm->setMinDisparity(this->minDisparity - 200);
    sgbm->setUniquenessRatio(this->uniqRatio);
    sgbm->setSpeckleWindowSize(this->speckWinSize);
    sgbm->setSpeckleRange(this->speckRange);
    sgbm->setDisp12MaxDiff(this->dispMaxDiff);
    sgbm->setPreFilterCap(this->preFilterCap);
    sgbm->setMode(StereoSGBM::MODE_HH4);

    wls_filter = createDisparityWLSFilter(sgbm);

    matching_time = (double)getTickCount();
    sgbm-> compute(left_for_matcher, right_for_matcher,left_disp);
    matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();


    ROI = computeROI(left_for_matcher.size(),sgbm);
    wls_filter = createDisparityWLSFilterGeneric(false);
    wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*getSADWindowSize()));

    /// [filtering]
    wls_filter->setLambda(this->lambda);
    wls_filter->setSigmaColor(float(this->sigma/100));
    filtering_time = (double)getTickCount();

    wls_filter->filter(left_disp,_leftRectImage,filtered_disp);
    filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
    /// [filtering]
    conf_map = wls_filter->getConfidenceMap();

    cout.precision(2);
    cout<<"--Matching time:  "<<matching_time<<"s"<<endl;
    cout<<"--Filtering time: "<<filtering_time<<"s"<<endl;

   visualizeResults(left_disp,filtered_disp);


    return 1;
}

bool stereovision::computeBM(Mat &_leftRectImage, Mat &_rightRectImage, Mat &_referenceImage){

    Mat left_for_matcher, right_for_matcher;
    Mat left_disp,right_disp;
    Mat filtered_disp;
    Mat conf_map = Mat(_leftRectImage.rows,_leftRectImage.cols,CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    Ptr<DisparityWLSFilter> wls_filter;
    double matching_time, filtering_time;


    Size img_size = _referenceImage.size();


    Mat disp, disp8;

    this->SADWindowSize = this->SADWindowSize > 4 ? this->SADWindowSize : 5;
    this->ndisparities = this->ndisparities > 0 ? this->ndisparities : ((img_size.width/8) + 15) & -16;

    if (this->SADWindowSize % 2 == 0)
            this->SADWindowSize += 1;
    while (!((this->ndisparities % 16) == 0) || this->ndisparities == 0)
    {
        this->ndisparities += 1;
    }

    Ptr<StereoBM> sbm = StereoBM::create(this->ndisparities,this->SADWindowSize > 0 ? this->SADWindowSize : 9);
    sbm->setROI1(ROI1);
    sbm->setROI1(ROI2);
    sbm->setPreFilterSize(this->preFilterSize % 2 == 0 ? this->preFilterSize += 1: this->preFilterSize );
    sbm->setPreFilterCap(this->preFilterCap > 0 ? this->preFilterCap : 1 );
    sbm->setMinDisparity(this->minDisparity - 200);
    sbm->setTextureThreshold(this->textureTreshold);
    sbm->setUniquenessRatio(this->uniqRatio);
    sbm->setSpeckleWindowSize(this->speckWinSize);
    sbm->setSpeckleRange(this->speckRange);
    sbm->setDisp12MaxDiff(this->dispMaxDiff);

    left_for_matcher = _leftRectImage.clone();
    right_for_matcher = _rightRectImage.clone();

    cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY);
    cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);

    wls_filter = createDisparityWLSFilter(sbm);

    cout << getSADWindowSize() << endl;
    matching_time = (double)getTickCount();
    sbm-> compute(left_for_matcher, right_for_matcher,left_disp);
    matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();

    ROI = computeROI(left_for_matcher.size(),sbm);
    wls_filter = createDisparityWLSFilterGeneric(false);
    wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*getSADWindowSize()));

    /// [filtering]
    wls_filter->setLambda(this->lambda);
    wls_filter->setSigmaColor(float(this->sigma/100));
    filtering_time = (double)getTickCount();

    wls_filter->filter(left_disp,_leftRectImage,filtered_disp);
    filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
    /// [filtering]
    conf_map = wls_filter->getConfidenceMap();


    cout.precision(2);
    cout<<"--Matching time:  "<<matching_time<<"s"<<endl;
    cout<<"--Filtering time: "<<filtering_time<<"s"<<endl;

    visualizeResults(left_disp,filtered_disp);

    return 1;
}


void stereovision::visualizeResults(const Mat& _rawDisparity, const Mat& _filteredDisparity)
{
    Mat raw_disp_vis, filtered_disp_vis, result;

    float distance;
    if(this->mouseInput.height > 0 || this->mouseInput.width > 0) {

        distance = reprojectTo3DPoint(this->mouseInput, _filteredDisparity);
    }
    getDisparityVis(_rawDisparity,raw_disp_vis,1);
    namedWindow("raw disparity", WINDOW_AUTOSIZE);
    imshow("raw disparity", raw_disp_vis);

    getDisparityVis(_filteredDisparity,filtered_disp_vis,this->colorScale/100);
    namedWindow("filtered disparity", WINDOW_AUTOSIZE);
    imshow("filtered disparity", filtered_disp_vis);

    applyColorMap(filtered_disp_vis,result, COLORMAP_JET);

    if(this->mouseInput.size().width != 0 || this->mouseInput.size().height != 0) {
        std::ostringstream ss;
        ss << distance;
        std::string text(ss.str());

        putText(result, text, cvPoint(this->mouseInput.x, this->mouseInput.y - 10),
                FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 0), 1, CV_AA);
        rectangle(result, this->mouseInput, Scalar(0, 0, 0), 2);
    }

    imshow("Disparity result", result);
}

float stereovision::reprojectTo3DPoint(const Rect& ROI, const Mat& _disparityMap)
{
    Mat Q_32F;
    this->calibParam.Q.convertTo(Q_32F,CV_32F);


    Mat disparity32F;
    _disparityMap.convertTo(disparity32F,CV_32F,1./16);
    disparity32F = disparity32F(ROI);
    Mat measureMat = disparity32F.clone();
    Scalar m = mean(measureMat);

    cout << "Mean value"<<m[0] << endl;

    cv::Mat_<float> vec_tmp(4,1);
    vec_tmp(0)=ROI.x;
    vec_tmp(1)=ROI.y;
    vec_tmp(2)= static_cast<float>(m[0]);
    vec_tmp(3)=1;
    vec_tmp = Q_32F*vec_tmp;
    vec_tmp /= vec_tmp(3);


//    for(int y=0; y<disparity32F.rows; ++y) {
//        for(int x=0; x<disparity32F.cols; ++x) {
//            vec_tmp(0)=x; vec_tmp(1)=y; vec_tmp(2)=disparity32F.at<float>(y,x); vec_tmp(3)=1;
//            vec_tmp = Q_32F*vec_tmp;
//            vec_tmp /= vec_tmp(3);
//            cv::Vec3f &point = XYZ.at<cv::Vec3f>(y,x);
//            point[0] = vec_tmp(0);
//            point[1] = vec_tmp(1);
//            point[2] = vec_tmp(2);
//        }
//    }

    return vec_tmp(2);
}


void stereovision::visualizePointCloud(const Mat& _disparityMap, const Mat& _colorMap)
{
//    Mat colorMap = _colorMap;
//    Mat dispMap = _disparityMap;
//    static bool firstCall;
//
//    Mat Q_32F;
//    this->calibParam.Q.convertTo(Q_32F,CV_32F);
//
//    Mat disparity32F;
//    _disparityMap.convertTo(disparity32F,CV_32F,1./16);
//
//    std::vector<Point3f> XYZ_Vec;
//    std::vector<Vec3b> RGB_Vec;
//
//    cv::Mat_<cv::Vec3f> XYZ(disparity32F.rows,disparity32F.cols);   // Output point cloud
//    cv::Mat_<float> vec_tmp(4,1);
//    for(int y=0; y<disparity32F.rows; ++y) {
//        for(int x=0; x<disparity32F.cols; ++x) {
//            vec_tmp(0)=x; vec_tmp(1)=y; vec_tmp(2)=disparity32F.at<float>(y,x); vec_tmp(3)=1;
//            vec_tmp = Q_32F*vec_tmp;
//            vec_tmp /= vec_tmp(3);
//            cv::Vec3f &point = XYZ.at<cv::Vec3f>(y,x);
//            point[0] = vec_tmp(0);
//            point[1] = vec_tmp(1);
//            point[2] = vec_tmp(2);
//
//            XYZ_Vec.push_back(Point3f(vec_tmp(0),vec_tmp(1),vec_tmp(2)));
//            RGB_Vec.push_back(colorMap.at<Vec3b>(y,x));
//        }
//    }
//
//    /// Pointcloud 3d window viz
//    static cv::viz::Viz3d window("Coordinate Frame");
//
//    window.removeAllWidgets();
//    window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
////    viz::WCloud cw(XYZ, croppedColorMap);
//    viz::WCloud cw(XYZ_Vec, RGB_Vec);
//    cw.setRenderingProperty(cv::viz::POINT_SIZE, 2);
//
//    window.showWidget("Cloud Widget", cw);
////    ///Plot 3D bounding frames of detected objects
////    for(int i = 0; i < _outBBoxStruct.size(); i++) {
////
////        string text = getDetectedObject(_detectedClass[i]);
////        string Frame,ObjectType;
////
////        Frame = "Frame" + std::to_string(i);
////        ObjectType = "ObjType" + std::to_string(i);
////
////        std::vector<Point3f> cornerPoints = getRealFrame(_outBBoxStruct[i]);
////        viz::WText3D text3D(text,Point3f(cornerPoints[3].x,cornerPoints[3].y,cornerPoints[3].z), 0.17);
////        viz::WPolyLine boundingFrame(cornerPoints, cv::viz::Color::red() );
////        window.showWidget(Frame, boundingFrame);
////        window.showWidget(ObjectType, text3D);
////    }
//
//    window.spinOnce(10, true);
////    if(!firstCall) {
////        /// Let's assume camera has the following properties
////        Point3d cam_origin(3.0, 3.0, 3.0), cam_focal_point(3.0, 3.0, 2.0), cam_y_dir(0.0, 0.0, 0.0);
////
////        /// We can get the pose of the cam using makeCameraPose
////        Affine3d camera_pose = viz::makeCameraPose(cam_origin, cam_focal_point, cam_y_dir);
////
//////    Mat R(3,3,CV_32F);
////
////        Matx33f R(0.98f, -0.08f, -0.13f,
////                  0.08f, 0.99f, -0.005f,
////                  0.13f, -0.005f, 0.99f);
////
//////    R.at(0,0) = 0.98f;
//////    R.at<float>(0,1) = -0.08f;
//////    R.at<float>(0,2) = -0.13f;
////
//////    R.at<float>(1,0) = 0.08f;
//////    R.at<float>(1,1) = 0.99f;
//////    R.at<float>(1,2) = -0.005f;
////
//////    R.at<float>(2,0) = 0.13f;
//////    R.at<float>(2,1) = -0.005f;
//////    R.at<float>(2,2) = 0.99f;
////
////        cout << R << endl;
////
////        Vec3f t;
////        t[0] = 0.29f;
////        t[1] = -0.18f;
////        t[2] = -2.15f;
////
////        cout << t << endl;
////
////        Affine3d startPose(R,t);
////
////        window.setViewerPose(startPose);
////
////        firstCall = true;
////    }
//
//
//
////    Camera cam = window.getCamera();
//    Affine3d pose3d = window.getViewerPose();
//
//    cout << "\n rvec " << pose3d.rotation() << "trans" << pose3d.translation() << endl;
}

bool stereovision::displayTrackbar(){

    /// Control BM parameters Winodw
    namedWindow("Stereo parameters control",WINDOW_FREERATIO);

    Mat logo = imread("../logo.jpg");
    imshow("Stereo parameters control", logo);

    char dispNumb[50];
    sprintf(dispNumb, "Disparity range");
    createTrackbar(dispNumb, "Stereo parameters control", &ndisparities, 500, trackbarLink);

    char windowSize[50];
    sprintf(windowSize, "Window size");
    createTrackbar(windowSize, "Stereo parameters control", &SADWindowSize, 25, trackbarLink);

    char minDisp[50];
    sprintf(minDisp, "Min disparity");
    createTrackbar(minDisp, "Stereo parameters control", &minDisparity, 800, trackbarLink);

    char uniqRat[50];
    sprintf(uniqRat, "Unique ratio");
    createTrackbar(uniqRat, "Stereo parameters control", &uniqRatio, 40, trackbarLink);

    char speckWinS[50];
    sprintf(speckWinS, "Speckle wind size");
    createTrackbar(speckWinS, "Stereo parameters control", &speckWinSize, 500, trackbarLink);

    char speckR[50];
    sprintf(speckR, "Speckle range");
    createTrackbar(speckR, "Stereo parameters control", &speckRange, 10, trackbarLink);

    char dispMaxD[50];
    sprintf(dispMaxD, "Disparity max diff");
    createTrackbar(dispMaxD, "Stereo parameters control", &dispMaxDiff, 1000, trackbarLink);

    char preFilterC[50];
    sprintf(preFilterC, "Prefilter cap");
    createTrackbar(preFilterC, "Stereo parameters control", &preFilterCap, 300, trackbarLink);

    char preFilterS[50];
    sprintf(preFilterS, "Prefilter size");
    createTrackbar(preFilterS, "Stereo parameters control", &preFilterSize, 200, trackbarLink);

    char textureTresh[50];
    sprintf(textureTresh, "Texture treshold");
    createTrackbar(textureTresh, "Stereo parameters control", &textureTreshold, 600, trackbarLink);

    char lambdaV[50];
    sprintf(lambdaV, "Lambda");
    createTrackbar(lambdaV, "Stereo parameters control", &lambda, 10000, trackbarLink);

    char sigmaV[50];
    sprintf(sigmaV, "Sigma");
    createTrackbar(sigmaV, "Stereo parameters control", &sigma, 1000 , trackbarLink);

    char colorGain[50];
    sprintf(colorGain, "Color Gain");
    createTrackbar(colorGain, "Stereo parameters control", &colorScale, 1000 , trackbarLink);

    waitKey(20);
}


bool stereovision::readFromImage(Mat &_leftRectImage, Mat &_rightRectImage)
{
    static VideoCapture capL("images/I1_%06d.png"),capR("images/I2_%06d.png");

        std::cout << "Init done" << std::endl;

        if (!capL.isOpened()) {// check if we succeeded
            std::cout << "Error reading images L" << std::endl;
            return -1;
        }
        if (!capR.isOpened()) {// check if we succeeded
            std::cout << "Error reading images R" << std::endl;
            return -1;
        }

        capL >> _leftRectImage;
        capR >> _rightRectImage;

    return 1;
}

int stereovision::getNdisparities() const {
    return ndisparities;
}

void stereovision::setNdisparities(int ndisparities) {
    stereovision::ndisparities = ndisparities;
}

int stereovision::getSADWindowSize() const {
    return SADWindowSize;
}

void stereovision::setSADWindowSize(int SADWindowSize) {
    stereovision::SADWindowSize = SADWindowSize;
}

int stereovision::getMinDisparity() const {
    return minDisparity;
}

void stereovision::setMinDisparity(int minDisparity) {
    stereovision::minDisparity = minDisparity;
}

int stereovision::getUniqRatio() const {
    return uniqRatio;
}

void stereovision::setUniqRatio(int uniqRatio) {
    stereovision::uniqRatio = uniqRatio;
}

int stereovision::getSpeckWinSize() const {
    return speckWinSize;
}

void stereovision::setSpeckWinSize(int speckWinSize) {
    stereovision::speckWinSize = speckWinSize;
}

int stereovision::getSpeckRange() const {
    return speckRange;
}

void stereovision::setSpeckRange(int speckRange) {
    stereovision::speckRange = speckRange;
}

int stereovision::getDispMaxDiff() const {
    return dispMaxDiff;
}

void stereovision::setDispMaxDiff(int dispMaxDiff) {
    stereovision::dispMaxDiff = dispMaxDiff;
}

int stereovision::getPreFilterCap() const {
    return preFilterCap;
}

void stereovision::setPreFilterCap(int preFilterCap) {
    stereovision::preFilterCap = preFilterCap;
}

int stereovision::getPreFilterSize() const {
    return preFilterSize;
}

void stereovision::setPreFilterSize(int preFilterSize) {
    stereovision::preFilterSize = preFilterSize;
}

int stereovision::getTextureTreshold() const {
    return textureTreshold;
}

void stereovision::setTextureTreshold(int textureTreshold) {
    stereovision::textureTreshold = textureTreshold;
}


/// OpenCV trackbar method
void trackbarLink(int v, void *ptr) {};

void onMousePointCallback(int _evt, int _x, int _y, int _flags, void *_param) {
    if(_evt != CV_EVENT_LBUTTONDOWN)
        return;

    Point* _posPtr = (Point*)_param;
    *_posPtr = Point(_x, _y);
}

void onMouseRectCallback(int _evt, int _x, int _y, int _flags, void *_param)
{
    static bool _clicked = false;
    cv::Rect_<int>* _ROI = (cv::Rect_<int>*)_param;
    switch(_evt)
    {
        case  CV_EVENT_LBUTTONDOWN  :
            *_ROI = Rect(_x, _y, 0, 0);
            _clicked = true;
            break;

        case  CV_EVENT_LBUTTONUP    :
            _ROI->width = _x - _ROI->x;
            _ROI->height = _y - _ROI->y;
            _clicked = false;
            break;

        case  CV_EVENT_MOUSEMOVE    :
            if(_clicked){
                _ROI->width = _x - _ROI->x;
                _ROI->height = _y - _ROI->y;
            }
            break;

        default                     :   break;
    }

    if(_evt != CV_EVENT_LBUTTONDOWN)
        return;

    Point* _posPtr = (Point*)_param;
    *_posPtr = Point(_x, _y);
}
