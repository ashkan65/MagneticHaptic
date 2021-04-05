#include "Filter.hpp"

Filter::Filter(){
    kf =  new cv::KalmanFilter(stateSize, measSize, contrSize, type);
    state = new cv::Mat(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z,a_x,a_y,a_z]
    meas  = new cv::Mat(measSize, 1, type);    // [z_x,z_y,z_z]
	////////////////////////////////////////////////////////////////
	// Setting up the filter:
	///////////////////////////////////////////////////////////////
    // Transition State Matrix A
    // [ 1  0  0 dT  0  0 dT2  0   0 ]
    // [ 0  1  0  0 dT  0  0  dT2  0 ]
    // [ 0  0  1  0  0 dT  0   0  dT2]
    // [ 0  0  0  1  0  0 dT   0   0 ]
    // [ 0  0  0  0  1  0  0  dT   0 ]
    // [ 0  0  0  0  0  1  0   0  dT ]
    // [ 0  0  0  0  0  0  1   0   0 ]
    // [ 0  0  0  0  0  0  0   1   0 ]
    // [ 0  0  0  0  0  0  0   0   1 ]
    cv::setIdentity(kf->transitionMatrix);
    
    // Measure Matrix H
    // [ 1 0 0 0 0 0 0 0 0]
    // [ 0 1 0 0 0 0 0 0 0]
    // [ 0 0 1 0 0 0 0 0 0]
    kf->measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf->measurementMatrix.at<float>(0) = 1.0f;
    kf->measurementMatrix.at<float>(10) = 1.0f;
    kf->measurementMatrix.at<float>(20) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0    0     0     0    0    0     0    0   ]
    // [ 0    Ey   0     0     0    0    0     0    0   ]
    // [ 0    0    Ez    0     0    0    0     0    0   ]
    // [ 0    0    0     Ev_x  0    0    0     0    0   ]
    // [ 0    0    0     0     Ev_y 0    0     0    0   ]
    // [ 0    0    0     0     0    Ev_x 0     0    0   ]
    // [ 0    0    0     0     0    0    Ea_x  0    0   ]
    // [ 0    0    0     0     0    0    0     Ea_y 0   ]
    // [ 0    0    0     0     0    0    0     0    Ea_z]
    cv::setIdentity(kf->processNoiseCov, cv::Scalar(1e-2));
    // kf->processNoiseCov.at<float>(0)  = 1e-2;
    // kf->processNoiseCov.at<float>(10) = 1e-2;
    // kf->processNoiseCov.at<float>(20) = 1e-2;
    // kf->processNoiseCov.at<float>(30) = 5.0f;
    // kf->processNoiseCov.at<float>(40) = 5.0f;
    // kf->processNoiseCov.at<float>(50) = 5.0f;
    // kf->processNoiseCov.at<float>(60) = 20.0f;
    // kf->processNoiseCov.at<float>(70) = 20.0f;
    // kf->processNoiseCov.at<float>(80) = 20.0f;


    // Measures Noise Covariance Matrix R
    // cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-3));
    // std::cout <<kf->measurementNoiseCov<<std::endl;

    // cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-50));
    kf->measurementNoiseCov.at<float>(0)  = 1e-20;
    kf->measurementNoiseCov.at<float>(4)  = 1e-20;
    kf->measurementNoiseCov.at<float>(8) = 1e-20;

    // kf->controlMatrix.at<float>(0) = 0.0;// This is B matrix

};

Filter::~Filter(){
    // delete kf;
    // delete state;
    // delete meas;
}

void Filter::CorrectPose(cv::Mat* _pose){
    // precTick = (double) cv::getTickCount();

    precTick = tick;
    tick = (double) cv::getTickCount();
    double dT = (tick - precTick) / cv::getTickFrequency(); //seconds
    if (found)
    {
        kf->transitionMatrix.at<float>(3)  = dT;
        kf->transitionMatrix.at<float>(13) = dT;
        kf->transitionMatrix.at<float>(23) = dT;
        kf->transitionMatrix.at<float>(6)  = 0.5*dT*dT;
        kf->transitionMatrix.at<float>(16) = 0.5*dT*dT;
        kf->transitionMatrix.at<float>(26) = 0.5*dT*dT;

                
            //     // <<<< Matrix A

            //     cout << "dT:" << endl << dT << endl;

        *state = kf->predict();
    }
	// meas.at<float>(0) = (*_pose)(0);
	// meas.at<float>(1) = (*_pose)(1);
	// meas.at<float>(2) = 0;
    // meas = *_pose;
    // meas->at<float>(0,0) = _pose->at<float>(0,0);
    // meas->at<float>(1,0) = _pose->at<float>(1,0);
    // meas->at<float>(2,0) = _pose->at<float>(2,0);
	if (!found) // First detection!
	{
	    // >>>> Initialization
	    kf->errorCovPre.at<float>(0)  = 1;
	    kf->errorCovPre.at<float>(10) = 1;
	    kf->errorCovPre.at<float>(20) = 1;
	    kf->errorCovPre.at<float>(30) = 1;
	    kf->errorCovPre.at<float>(40) = 1;
	    kf->errorCovPre.at<float>(50) = 1;
	    kf->errorCovPre.at<float>(60) = 1;
	    kf->errorCovPre.at<float>(70) = 1;
	    kf->errorCovPre.at<float>(80) = 1;

	    state->at<float>(0,0) = _pose->at<float>(0,0);
	    state->at<float>(1,0) = _pose->at<float>(1,0);
	    state->at<float>(2,0) = _pose->at<float>(2,0);

	    // <<<< Initialization

	    kf->statePost = *state;
	    found = true;
	}
	else
	    kf->correct(*_pose); // Kalman Correction
};


void Filter::GetPose(cv::Mat* _pose){
    // @TDDO: Change this to crono type
    precTick = tick;
    tick = (double) cv::getTickCount();
    dT = (tick - precTick) / cv::getTickFrequency(); //seconds
    if (found)	
    {
        // >>>> Matrix A
        kf->transitionMatrix.at<float>(3)  = dT;
        kf->transitionMatrix.at<float>(13) = dT;
        kf->transitionMatrix.at<float>(23) = dT;
        kf->transitionMatrix.at<float>(6)  = 0.5*dT*dT;
        kf->transitionMatrix.at<float>(16) = 0.5*dT*dT;
        kf->transitionMatrix.at<float>(26) = 0.5*dT*dT;        
        // <<<< Matrix A
        *_pose = kf->predict();
        // std::cout <<"State --:"<< state->row(0)<<std::endl;
    } 
};

void Filter::GetROILocation(uint* _ROI_loc, uint _camera_number){

};

void Filter::ROIPoseCorrection(uint* _ROI_loc, uint _camera_number){
    
};

void Filter::RegisterCameras(){
    std::vector<cv::Vec3d> rvecsW, tvecsW, rvecsC1, tvecsC1, rvecsC2, tvecsC2;  // Rotation and translation vectors of detected points in W, C1, and C2 frames 
    std::vector<int> markerIdsW, markerIdsC1 , markerIdsC2 ;  // Markers ID in W, C1, and C2 frames
    std::vector<std::vector<cv::Point2f>> markerCornersC1, rejectedCandidatesC1, markerCornersC2, rejectedCandidatesC2;
    // Generating the dictionary for markers. Besure the 4X4_50 is the same as the used markers in the frames
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    parameters = cv::aruco::DetectorParameters::create();
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    
    
    // It is better to move everything to a xml file and read that-> more info on how to do it is in Ashkan's git 
    markerIdsW.push_back(4);
    cv::Vec3d Marck4(-0.5,9.5,-0.5);
    tvecsW.push_back(Marck4);
    //
    markerIdsW.push_back(7);
    cv::Vec3d Marck7(-0.1,9.4,-6.2);
    tvecsW.push_back(Marck7);
    //
    markerIdsW.push_back(8);
    cv::Vec3d Marck8(-0.1,1.0,-11.8);
    tvecsW.push_back(Marck8);
    //
    markerIdsW.push_back(2);
    cv::Vec3d Marck2(0.6,0.6,0.0);
    tvecsW.push_back(Marck2);
    //
    markerIdsW.push_back(5);
    cv::Vec3d Marck9(14.0,9.5,0.0);
    tvecsW.push_back(Marck9);
    //
    markerIdsW.push_back(3);
    cv::Vec3d Marck3(14.0,0.5,0.0);
    tvecsW.push_back(Marck3);
    //
    markerIdsW.push_back(10);
    cv::Vec3d Marck10(14.0,0.5,0.0);
    tvecsW.push_back(Marck10);
    //
    markerIdsW.push_back(6);
    cv::Vec3d Marck6(8.0,0.0,-11.8);
    tvecsW.push_back(Marck6);

	////////////////////////////////////////////////////////////////
    // Reading the camere calibration  
	////////////////////////////////////////////////////////////////
    cv::Mat cameraMatrixC1, distCoeffsC1, cameraMatrixC2, distCoeffsC2;
    cv::FileStorage fs2("../calibration_left_camera.xml", cv::FileStorage::READ);
    fs2["camera_matrix"] >> cameraMatrixC1;
    fs2["distortion_coefficients"] >> distCoeffsC1;
    fs2.release();

    cv::FileStorage fs1("../calibration_left_camera.xml", cv::FileStorage::READ);
    fs1["camera_matrix"] >> cameraMatrixC2;
    fs1["distortion_coefficients"] >> distCoeffsC2;
    fs1.release();
    
    //Somehow get two full frames -> either imread or full frame mode of the camera
    //If you are using imread, keed the adress in xml file. 
    cv::Mat frame1 = cv::imread("../calibration/left_6.jpg", cv::IMREAD_COLOR);
    cv::Mat frame2 = cv::imread("../calibration/right_2.jpg", cv::IMREAD_COLOR);
    // Detecting the corners of markers
    cv::aruco::detectMarkers(frame1, dictionary, markerCornersC1, markerIdsC1, parameters, rejectedCandidatesC1);
    cv::aruco::detectMarkers(frame2, dictionary, markerCornersC2, markerIdsC2, parameters, rejectedCandidatesC2);
    // Estimating the pose (Location and orientation of markers)
    cv::aruco::estimatePoseSingleMarkers(markerCornersC1, 0.2, cameraMatrixC1, distCoeffsC1, rvecsC1, tvecsC1);
    cv::aruco::estimatePoseSingleMarkers(markerCornersC2, 0.2, cameraMatrixC2, distCoeffsC2, rvecsC2, tvecsC2);
    ////////////////////////////////////////////////////////////////
    // Going through marker IDs to match them :
    ////////////////////////////////////////////////////////////////

    std::vector<cv::Vec3f> matched_points_C1, matched_points_W1, matched_points_C2, matched_points_W2;  // Matched points in W, C1, and C2. based on their IDs.  
    if (markerIdsC1.size()>3){
        for (int i = 0; i< markerIdsC1.size();i++){
            for (int j = 0; j< markerIdsW.size();j++){
                if (markerIdsC1[i] == markerIdsW[j]){
                    // std::cout <<markerIds1[i]<<"--"<<markerIdsW[j]<<std::endl;
                    matched_points_C1.push_back(tvecsC1[i]);
                    matched_points_W1.push_back(tvecsW[j]);
                    // std::cout<<markerIds1[i] << markerIdsW[j]<<std::endl;
                }  
            }   
            // if (markerIdsC1[i]==1){
            //     target_index = i;
            //     std::cout<<"Mes:"<<tvecs1[i]<<std::endl;
            //     tempp.push_back(tvecs1[i]);

            // }
        }
    }
    if (markerIdsC2.size()>3){
        for (int i = 0; i< markerIdsC2.size();i++){
            for (int j = 0; j< markerIdsW.size();j++){
                if (markerIdsC2[i] == markerIdsW[j]){
                    // std::cout <<markerIds1[i]<<"--"<<markerIdsW[j]<<std::endl;
                    matched_points_C2.push_back(tvecsC1[i]);
                    matched_points_W2.push_back(tvecsW[j]);
                    // std::cout<<markerIds1[i] << markerIdsW[j]<<std::endl;
                }  
            }   
            // if (markerIdsC1[i]==1){
            //     target_index = i;
            //     std::cout<<"Mes:"<<tvecs1[i]<<std::endl;
            //     tempp.push_back(tvecs1[i]);

            // }
        }
    }
    
    ////////////////////////////////////////////////////////////////
    // Estimating thransformation matrxies between Cameras and world frame :
    ////////////////////////////////////////////////////////////////
    
    
    cv::Mat inliersW2C1, inliersC12W, inliersW2C2, inliersC22W;
    double data[4] = { 0.0, 0.0, 0.0, 1.0 };
    cv::Mat L = cv::Mat(1, 4, CV_64F, data);                                         // Generating a row of [0 0 0 1]

    cv::estimateAffine3D(matched_points_C1,matched_points_W1, Rt_W2C1, inliersW2C1); // estimateAffine3D(A,B,T) where B = T.A
    cv::estimateAffine3D(matched_points_W1, matched_points_C1, Rt_C12W, inliersC12W); // estimateAffine3D(A,B,T) where B = T.A
    cv::vconcat(Rt_W2C1, L, H_W2C1);
    cv::vconcat(Rt_C12W, L, H_C12W);

    cv::estimateAffine3D(matched_points_C2,matched_points_W1, Rt_W2C2, inliersW2C2); // estimateAffine3D(A,B,T) where B = T.A
    cv::estimateAffine3D(matched_points_W1, matched_points_C2, Rt_C12W, inliersC22W); // estimateAffine3D(A,B,T) where B = T.A
    cv::vconcat(Rt_W2C1, L, H_W2C1);
    cv::vconcat(Rt_C12W, L, H_C12W);
};

