#include "TattileCamera.hpp"
#include "PoseEstimate.hpp"
#include <iostream>       // std::cout
#include <atomic>         // std::atomic, std::atomic_flag, ATOMIC_FLAG_INIT
#include <thread>         // std::thread, std::this_thread::yield
#include <vector>         // std::vector
#include <chrono>	  // TO add artificial delays. You can remove it.
#include <atomic>         // std::atomic, std::atomic_flag, ATOMIC_FLAG_INIT
#include <thread>         // std::thread, std::this_thread::yield
#include <functional>	//std::mem_fn
bool cam1_switch = true;  
ROI_t cam1_roi;
std::atomic<short> cam1_available_index (0);
std::atomic<bool> cam1_NewFrame (false);
std::atomic<bool> cam1_NewROI (true);
ROI_t ROI = {ROI_WIDTH,ROI_HEIGHT, 1600,1200};



// std::atomic<int> &ThreadsCounter
cv::Mat cam1_Buffer[3];

int main() {
	// cv::Mat ROI_frame = cv::imread("1.png");
	// cv::Mat full_frame = cv::Mat::zeros(cv::Size(4096, 3072), CV_8U);
	// cv::cvtColor(full_frame,full_frame, cv::COLOR_GRAY2RGB );
	// ROI_frame.copyTo(full_frame(cv::Rect(ROI.x, ROI.y, ROI_frame.cols, ROI_frame.rows)));
	// char key;
	// while (key !='q'){
	// 	cv::namedWindow( "FullFrame", cv::WINDOW_NORMAL );	
	// 	cv::resizeWindow( "FullFrame" , 600,800 );
	// 	cv::imshow("FullFrame", full_frame);
	// 	key = cv::waitKey(30);
	// }
	int a;
	PoseEstimate Pose1;
	TattileCamera Cam1;
	Cam1.SetupIPAddress("192.168.1.6");
	Cam1.SetConnections(&cam1_NewFrame, &cam1_available_index , &cam1_switch, &cam1_NewROI, &ROI);
	std::cout<<"HERE-----------------------------------"<<std::endl;
	Pose1.SetCameraCalibration("../calibration/cam_left.yml");
	Pose1.SetBuffer(Cam1.GetBuffer());
	Pose1.SetConnections(&cam1_NewFrame, &cam1_available_index , &cam1_switch, &cam1_NewROI, &(ROI.x), &(ROI.y));	
	std::thread thread1(std::mem_fn(&TattileCamera::Run), &Cam1);
	std::thread thread2(std::mem_fn(&PoseEstimate::Run), &Pose1);
	thread1.join();	
	thread2.join();	

	return 0;
}


