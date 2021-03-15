/******************************************
 * OpenCV Tutorial: Ball Tracking using   *
 * Kalman Filter                          *
 ******************************************/

// Module "core"
#include <opencv2/core/core.hpp>

// Module "highgui"
#include <opencv2/highgui/highgui.hpp>

// Module "imgproc"
#include <opencv2/imgproc/imgproc.hpp>

// Module "video"
#include <opencv2/video/video.hpp>

// Output
#include <iostream>

// Vector
#include <vector>

//Aruco
#include <opencv2/aruco.hpp>

using namespace std;

// >>>>> Color to be tracked
#define MIN_H_BLUE 200
#define MAX_H_BLUE 300
// <<<<< Color to be tracked


int main()
{
    // Camera frame
    cv::Mat frame;
    //
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat current_frame;
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    int MarkerSize;
    char key;
    parameters = cv::aruco::DetectorParameters::create();
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    // >>>> Kalman Filter
    int stateSize = 9;
    int measSize = 3;
    int contrSize = 0;

    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

    cv::Mat state(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z,a_x,a_y,a_z]
    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_z]
    // cv::Mat procNoise(stateSize, 1, type)
    // [E_x,E_y,E_z,E_v_x,E_v_y,E_v_z,E_a_x,E_a_y,E_a_z]

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1  0  0 dT  0  0 dT2  0   0 ]
    // [ 0  1  0  0 dT  0  0  dT2  0 ]
    // [ 0  0  1  0  0 dT  0   0  dT2]
    // [ 0  0  0  1  0  0 dT   0   0 ]
    // [ 0  0  0  0  1  0  0  dT   0 ]
    // [ 0  0  0  0  0  1  0   0  dT ]
    // [ 0  0  0  0  0  0  1   0   0 ]
    // [ 0  0  0  0  0  0  0   1   0 ]
    // [ 0  0  0  0  0  0  0   0   1 ]
    cv::setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 0 0 0]
    // [ 0 1 0 0 0 0 0 0 0]
    // [ 0 0 1 0 0 0 0 0 0]

    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(10) = 1.0f;
    kf.measurementMatrix.at<float>(20) = 1.0f;

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
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0)  = 1e-2;
    kf.processNoiseCov.at<float>(10) = 1e-2;
    kf.processNoiseCov.at<float>(20) = 1e-2;
    kf.processNoiseCov.at<float>(30) = 5.0f;
    kf.processNoiseCov.at<float>(40) = 5.0f;
    kf.processNoiseCov.at<float>(50) = 5.0f;
    kf.processNoiseCov.at<float>(60) = 20.0f;
    kf.processNoiseCov.at<float>(70) = 20.0f;
    kf.processNoiseCov.at<float>(80) = 20.0f;


    // Measures Noise Covariance Matrix R
    // cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-3));
    // cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-50));
    kf.measurementNoiseCov.at<float>(0)  = 1e-10;
    kf.measurementNoiseCov.at<float>(7)  = 1e-10;
    kf.measurementNoiseCov.at<float>(14) = 1e-10;
    // <<<< Kalman Filter

    // Camera Index
    int idx = 0;

    // Camera Capture
    cv::VideoCapture cap(1);

    // >>>>> Camera Settings
    if (!cap.open(idx))
    {
        cout << "Webcam not connected.\n" << "Please verify\n";
        return EXIT_FAILURE;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1024);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 768);
    // <<<<< Camera Settings

    cout << "\nHit 'q' to exit...\n";

    char ch = 0;

    double ticks = 0;
    bool found = false;

    int notFoundCount = 0;

    // >>>>> Main loop
    while (ch != 'q' && ch != 'Q')
    {
        double precTick = ticks;
        ticks = (double) cv::getTickCount();

        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

        // Frame acquisition
        cap >> frame;
        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        if (found)
        {
        //     // >>>> Matrix A
            kf.transitionMatrix.at<float>(3)  = dT;
            kf.transitionMatrix.at<float>(13) = dT;
            kf.transitionMatrix.at<float>(23) = dT;
            kf.transitionMatrix.at<float>(6)  = 0.5*dT*dT;
            kf.transitionMatrix.at<float>(16) = 0.5*dT*dT;
            kf.transitionMatrix.at<float>(26) = 0.5*dT*dT;

            
        //     // <<<< Matrix A

        //     cout << "dT:" << endl << dT << endl;

            state = kf.predict();
            cout << "State post:" << endl << state << endl;

            cv::Point2f predRect((uint)state.at<float>(0) , predRect.y = (uint)state.at<float>(1));
            cv::circle( frame, predRect, 10.0, cv::Scalar(255, 0, 0 ), 3, 8 );
        }


        // >>>>> Filtering

        // <<<<< Filtering

        cout << "Balls found:" << markerCorners.size() << endl;

        // >>>>> Detection result
        for (size_t i = 0; i < markerCorners.size(); i++)
        {
        cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        }
        // <<<<< Detection result

        // >>>>> Kalman Update
        if (markerCorners.size() == 0)
        {
            notFoundCount++;
            cout << "notFoundCount:" << notFoundCount << endl;
            if( notFoundCount >= 100 )
            {
                found = false;
            }
            /*else
                kf.statePost = state;*/
        }
        else
        {
            notFoundCount = 0;

            meas.at<float>(0) = markerCorners[0][0].x;
            meas.at<float>(1) = markerCorners[0][0].y;
            meas.at<float>(2) = 0;
            

            if (!found) // First detection!
            {
                // >>>> Initialization


                kf.errorCovPre.at<float>(0)  = 1;
                kf.errorCovPre.at<float>(10) = 1;
                kf.errorCovPre.at<float>(20) = 1;
                kf.errorCovPre.at<float>(30) = 1;
                kf.errorCovPre.at<float>(40) = 1;
                kf.errorCovPre.at<float>(50) = 1;
                kf.errorCovPre.at<float>(60) = 1;
                kf.errorCovPre.at<float>(70) = 1;
                kf.errorCovPre.at<float>(80) = 1;



                state.at<float>(0) = meas.at<float>(0);
                state.at<float>(1) = meas.at<float>(1);
                state.at<float>(2) = 0;

                // <<<< Initialization

                kf.statePost = state;
                
                found = true;
            }
            else
                kf.correct(meas); // Kalman Correction

            cout << "Measure matrix:" << endl << meas << endl;
        }
        // <<<<< Kalman Update

        // Final result
        cv::imshow("Tracking", frame);

        // User key
        ch = cv::waitKey(1);
    }
    // <<<<< Main loop

    return EXIT_SUCCESS;
}