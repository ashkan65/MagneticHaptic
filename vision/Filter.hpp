#ifndef FILT_H
#define FILT_H
/*****************************************************
This library gets two poses from two cameras and using Kalman filter:
    1- estimates the location of the next ROIs for each camera 
    2- estimates the pose of the target
    Uses:
        Opencv
Ver 1.0 by Ashkan March-2021
IF you are using the Tattile cameras, please first read the notes attached to the setup!!!!!!!!!!!!!
There are so many things that can go wrong.         
*****************************************************/
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
// #include <opencv2/aruco.hpp>

class Filter{
    private:
        //////////////////////////////////////////////////////
        // Kalman Filter
        //////////////////////////////////////////////////////
        unsigned int type = CV_32F;
        int stateSize = 9;  // "six DOF with constant jerk"->18 :[x,y,z,rol,pitch,yaw,dx,dy,dz,drol,dpitch,dyaw,d2x,d2y,d2z,d2rol,d2pitch,d2yaw]
        int measSize  = 3;  // "six DOF"->6 : [x,y,z,rol,pitch,yaw]
        int contrSize = 0;  // @TODO: find out what is countersize!
        cv::KalmanFilter* kf;//(stateSize, measSize, contrSize, type);
        cv::Mat* state;//(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z,a_x,a_y,a_z]
        cv::Mat* meas;//(measSize, 1, type);    // [z_x,z_y,z_z]
        //cv::Mat procNoise(stateSize, 1, type)
        //////////////////////////////////////////////////////
        float dT;
        double precTick;    // to find dt
        double tick = 0;    // to find dt
        bool found = false; // for the first time to detect the marker first
        //////////////////////////////////////////////////////
        // Camerad register 
        ////////////////////////////////////////////////////// 
        cv::Mat Rt_W2C1, Rt_W2C2, Rt_C12W, Rt_C22W;  // Rotation and translation Matrix between W->C1, W->C2, C1->W, C2->W   it's a 3X4 matrix
        cv::Mat H_W2C1, H_W2C2, H_C12W, H_C22W;  // Transformation Matrix between W->C1, W->C2, C1->W, C2->W . Where H = [Rt;0 1]   --> You may or may not need this
        bool* filter_switch;                        // Kills the filter thread. Turns on and off from outside. 
    public:
        Filter();
        ~Filter();         
        void GetPose(cv::Mat* _pose);                               // Estimates the State (pose)  (GLOBAL)
        void CorrectPose(cv::Mat* _pose);                           // Corrects the pose          (GLOBAL)
        void GetROILocation(uint* _ROI_loc, uint _camera_number);   // Gets the location of the next ROI for the requested camera (NOT GLOBAL)
        void ROIPoseCorrection(uint* _pose, uint _camera_number);   // Corrects the state by the pose in the given camera frame (NOT GLOBAL)
        void RegisterCameras();  
        void Run();                                   // Registers cameras in world frame and generates transformation matrixes for both cameras
};
#endif // FILT_H