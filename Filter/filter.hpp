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


class Filter{
	private:
		//////////////////////////////////////////////////////
		// Kalman Filter
		//////////////////////////////////////////////////////
	    int stateSize = 9;  // "six DOF with constant jerk"->18 :[x,y,z,rol,pitch,yaw,dx,dy,dz,drol,dpitch,dyaw,d2x,d2y,d2z,d2rol,d2pitch,d2yaw]
	    int measSize  = 3;	// "six DOF"->6 : [x,y,z,rol,pitch,yaw]
	    int contrSize = 0;	// @TODO: find out what is countersize!
		cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
		cv::Mat state(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z,a_x,a_y,a_z]
		cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_z]
		//cv::Mat procNoise(stateSize, 1, type)
		//////////////////////////////////////////////////////
	    unsigned int type = CV_32F;
	    double precTick; 	// to find dt
	    double Tick = 0;	// to find dt
	    bool found = false;	// for the first time to detect the marker first
	    //////////////////////////////////////////////////////
	public:
		Filter();			
		~Filter();
		void GetPose(uint* _pose);   	// Estimates of State (pose)  (GLOBAL)
		void CorrectPose(uint* _pose);	// Corrects the pose 		  (GLOBAL)
		void GetROILocation(uint* _ROI_loc, uint _camera_number);	// Gets the location of the next ROI for the requested camera (NOT GLOBAL)
		void ROICorrection(uint* _pose, uint _camera_number);		// Corrects the state by the pose in the given camera frame (NOT GLOBAL)

};
#endif // FILT_H