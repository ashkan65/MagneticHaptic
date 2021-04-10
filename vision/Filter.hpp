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
#include <atomic>
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
        cv::Mat* state;     //(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z,a_x,a_y,a_z]
        cv::Mat* meas;      //(measSize, 1, type);    // [z_x,z_y,z_z]
        cv::Mat* estimate;  // (stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z,a_x,a_y,a_z]
        //cv::Mat procNoise(stateSize, 1, type)
        //////////////////////////////////////////////////////
        float dT;
        double precTick;    // to find dt
        double tick = 0;    // to find dt
        bool found = false; // for the first time to detect the marker first
        //////////////////////////////////////////////////////
        // Camerad register 
        ////////////////////////////////////////////////////// 
        
        cv::Mat Rt_W2C1, Rt_W2C2, Rt_C12W, Rt_C22W; //  Rotation and translation Matrix between W->C1, W->C2, C1->W, C2->W   it's a 3X4 matrix
        cv::Mat H_W2C1, H_W2C2, H_C12W, H_C22W;     //  Transformation Matrix between W->C1, W->C2, C1->W, C2->W . Where H = [Rt;0 1]   --> You may or may not need this
        bool* filter_switch;                        //  Kills the filter thread. Turns on and off from outside. 
        std::atomic<bool>* new_pose_cam1;           //  Boolian Atomic flag, shows there is a new measured pose from cam1 thread
        std::atomic<bool>* new_pose_cam2;           //  Boolian Atomic flag, shows there is a new measured pose from cam1 thread
        std::atomic<bool>* new_ROI_location;        //  Boolian Atomic flag, shows there is a ROI location for both cameras -> this also trigers cameras
        cv::Mat cameraMatrixC1, distCoeffsC1, cameraMatrixC2, distCoeffsC2; // Camera calibration and distortion coefs from single camera calibraiton.  

        //////////////////////////////////////////////////////
        //  Pointers to measured markers in PoseEstimate in each camera. --> Use Filter::SetTargetAddress and PoseEstimate::GetTargetLoc_P & PoseEstimate::GetTargetRot_P to set them up 
        //////////////////////////////////////////////////////
        std::vector<cv::Vec3d>* cam1T = nullptr;
        std::vector<cv::Vec3d>* cam1R = nullptr;
        std::vector<cv::Vec3d>* cam2T = nullptr;
        std::vector<cv::Vec3d>* cam2R = nullptr;     //  Pointers to measured markers in PoseEstimate in each camera. --> Use Filter::SetTargetAddress and PoseEstimate::GetTargetLoc_P & PoseEstimate::GetTargetRot_P to set them up 
        std::vector<cv::Vec3d>  cam1loc;
        cv::Mat pose;                                // The pose of the target in 3D--> change this to orientation later. 
        std::vector<cv::Vec3d> pose_Vec_C;          // The pose of the target in 3D--> change this to orientation later.
        std::vector<cv::Vec3f> pose_vec_W;  
        ///////////////////////////////////////////////////////
        uint16_t* ROI_C1_x;                         // Location (u) of the ROI in camera 1 frame
        uint16_t* ROI_C1_y;                         // Location (v) of the ROI in camera 1 frame
        uint16_t* ROI_C2_x;                         // Location (u) of the ROI in camera 2 frame
        uint16_t* ROI_C2_y;                         // Location (v) of the ROI in camera 2 frame
    
    
    public:
        Filter();
        ~Filter();         
        void GetPose(cv::Mat* _pose);                               //  Estimates the State (pose)  (GLOBAL)
        void CorrectPose(cv::Mat* _pose);                           //  Corrects the pose           (GLOBAL)
        void GetROILocation(uint* _ROI_loc, uint _camera_number);   //  Gets the location of the next ROI for the requested camera (NOT GLOBAL)
        void ROIPoseCorrection(uint* _pose, uint _camera_number);   //  Corrects the state by the pose in the given camera frame   (NOT GLOBAL)
        void RegisterCameras();                                     //  Reading the 
        void Run();                                                 //  Registers cameras in world frame and generates transformation W->C & C->W matrixes for both cameras
        void SetTargetsAddress(std::vector<cv::Vec3d>* cam1T, std::vector<cv::Vec3d>* cam1R, std::vector<cv::Vec3d>* cam2T, std::vector<cv::Vec3d>* cam2R); // 
        void SetConnection(bool* _filter_switch, std::atomic<bool>* _new_pose_cam1, std::atomic<bool>* _new_pose_cam2, std::atomic<bool>* _new_ROI, uint16_t* _ROI_x,uint16_t* _ROI_y); 
};
#endif // FILT_H