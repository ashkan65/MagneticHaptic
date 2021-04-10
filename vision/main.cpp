#include "TattileCamera.hpp"
#include "PoseEstimate.hpp"
#include "Filter.hpp"
#include <iostream>								// std::cout
#include <atomic>								// std::atomic, std::atomic_flag, ATOMIC_FLAG_INIT
#include <thread>								// std::thread, std::this_thread::yield
#include <vector>								// std::vector
#include <chrono>								// TO add artificial delays. You can remove it.
#include <atomic>								// std::atomic, std::atomic_flag, ATOMIC_FLAG_INIT
#include <thread>								// std::thread, std::this_thread::yield
#include <functional>							//std::mem_fn  -> you need this to pass fucntions to each threads
bool cam1_switch = true;						// The general switch for the cameras and all of threads  
ROI_t cam1_roi;
std::atomic<short> cam1_available_index (0);
std::atomic<bool> cam1_NewFrame (false);		// Boolian showing if a new frame is ready for the image processing thread to process
std::atomic<bool> cam1_NewROI (true);			// Boolian showing if a new ROI location is avaiable -> camera shots a frame
ROI_t ROI = {ROI_WIDTH,ROI_HEIGHT, 2176,1616};	// Location of the initial ROI
// std::atomic<cv::Vec3d> RvecsC1, TvecsC1;		// Rotaiton and location vectors of target in camera frame 1
// std::atomic<cv::Vec3d> RvecsC2, TvecsC2;		// Rotaiton and location vectors of target in camera frame 2



// std::atomic<int> &ThreadsCounter
cv::Mat cam1_Buffer[3];
// uint8_t packdfdfet[sizeof(frame_t) + IMG_WIDTH * IMG_HEIGHT];

int main() {
	// cv::Mat ROI_frame = cv::imread("1.png");
	// cv::Mat full_frame = cv::Mat::zeros(cv::Size(4096, 3072), CV_8U);
	// cv::cvtColor(full_frame,full_frame, cv::COLOR_GRAY2RGB );
	// ROI_frame.copyTo(full_frame(cv::Rect(ROI.x, ROI.y, ROI_frame.cols, ROI_frame.rows)));
	char key;
	// while (key !='q'){
	// 	cv::namedWindow( "FullFrame", cv::WINDOW_NORMAL );	
	// 	cv::resizeWindow( "FullFrame" , 600,800 );
	// 	cv::imshow("FullFrame", full_frame);
	// 	key = cv::waitKey(30);
	// }
	// int A = (sizeof(frame_t));
	// std::cout<<A<<std::endl;
	// long AA = IMG_WIDTH * IMG_HEIGHT;
	// std::cout<<AA<<std::endl;
	PoseEstimate Pose1;
	TattileCamera Cam1;
	Filter filter;
	Cam1.SetupIPAddress("192.168.1.6");
	Cam1.SetConnections(&cam1_NewFrame, &cam1_available_index , &cam1_switch, &cam1_NewROI, &ROI);
	std::cout<<"HERE-----------------------------------"<<std::endl;
// 	while (key !='q'){
// 		cv::namedWindow( "FullFrame_image", cv::WINDOW_NORMAL );	
// 		cv::resizeWindow( "FullFrame_image" , 600,800 );
// // //	*(Cam1.GetFullFrame());
// 		// cv::imwrite("FULL.jpg" , Cam1.GetFullFrame());
// 		cv::imshow("FullFrame_image",*(Cam1.GetFullFrame()));
// 		key = cv::waitKey(30);
// 	}
	Pose1.SetCameraCalibration("../calibration/cam_left.yml");
	Pose1.SetBuffer(Cam1.GetBuffer());
	Pose1.SetConnections(&cam1_NewFrame, &cam1_available_index , &cam1_switch, &(ROI.x), &(ROI.y));	
	filter.SetConnection(&cam1_switch, Pose1.GetNewPose_P(), Pose1.GetNewPose_P(), &cam1_NewROI, &(ROI.x), &(ROI.y));
	filter.SetTargetsAddress(Pose1.GetTargetLoc_P(), Pose1.GetTargetRot_P(), Pose1.GetTargetLoc_P(), Pose1.GetTargetRot_P());
	
	std::thread thread1(std::mem_fn(&TattileCamera::Run), &Cam1);
	std::thread thread2(std::mem_fn(&PoseEstimate::Run), &Pose1);
	std::thread thread3(std::mem_fn(&Filter::Run), &filter);
	int a;
	std::cin>>a;
	cam1_switch = false;
	thread1.join();	
	thread2.join();	
	thread3.join();	
	return 0;
}