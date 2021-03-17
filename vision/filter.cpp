#include "filter.hpp"

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
    // Change tis to crono type
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

void Filter::ROICorrection(uint* _ROI_loc, uint _camera_number){

};

