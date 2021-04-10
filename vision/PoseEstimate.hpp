#ifndef POSEEST_H
#define POSEEST_H
/*****************************************************
This gets a frame from a camera (either via shared memory or a passing a frame as an input). 
It uses Aruco to estimate the pose of the marker. 
    Uses:
		Opencv
		thread
Ver 1.0 by Ashkan Feb-2021
IF you are using the Tattile cameras, please first read the notes attached to the setup!!!!!!!!!!!!!
There are so many things that can go wrong.  		
*****************************************************/
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <thread>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/types.hpp>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <signal.h>
#include <opencv2/aruco.hpp>
#include <chrono>
#include <stdlib.h>  
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <mutex>
#include <sstream>
#include <condition_variable>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <atomic>

class PoseEstimate{
	private:
		//////////////////////////////////////////////////////
		// Buffer
		//////////////////////////////////////////////////////
		std::atomic<short> read_index ;				// Where in the buffer are you writing. 				 
		std::atomic<short> swap_index ;				// A temp variable to swap write with available. 
		cv::Mat** buffer;
		std::atomic<bool>* new_frame;				// Pointer to shared atomic to flag a new frame is avaiable to process 
		std::atomic<bool> new_pose;
		std::atomic<short>* available_index;		
		bool* vision_switch;						// This points to a camera switch.
		uint16_t* ROI_x;
		uint16_t* ROI_y;

		//////////////////////////////////////////////////////
		// Aruco 
		//////////////////////////////////////////////////////
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
		cv::Ptr<cv::aruco::DetectorParameters> parameters;
		cv::Mat cameraMatrix, distCoeffs;
		cv::Mat current_frame;
		std::vector<cv::Vec3d> rvecs, tvecs;
		cv::Ptr<cv::aruco::Dictionary> dictionary;
		int MarkerSize;
		char key;
		void Filter();
		//////////////////////////////////////////////////////
	public:
		PoseEstimate();
		~PoseEstimate();
		void ShowFrame(const char *_add);
		void ShowFrame(cv::Mat & _frame, const char *_add);
		void ThroughMirror(bool _mirror);
		void SetMarkerSize(int _size);
		void SetBuffer(cv::Mat**_buffer);  // This is a pointer to buffer size 3
		bool SetConnections( std::atomic<bool>* _new_frame , std::atomic<short>* _available_index , bool* vision_switch , uint16_t* _ROI_x,uint16_t* _ROI_y); //This is a 3 cell ring buffer with overwrite option.	
		void Run();
		bool SetCameraCalibration(std::string calibrationAddress);	// Reads the calibration file from the giver address
		std::vector<cv::Vec3d>* GetTargetLoc_P();	// Returns the pointer of the targets' location 
		std::vector<cv::Vec3d>* GetTargetRot_P();	// Returns the pointer of the targets' rotation
		std::atomic<bool>* GetNewPose_P();  
};
#endif // POSEEST_H