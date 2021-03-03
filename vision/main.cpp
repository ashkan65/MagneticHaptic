#include "TattileCamera.hpp"
#include "PoseEstimate.hpp"
#include <iostream>       // std::cout
#include <atomic>         // std::atomic, std::atomic_flag, ATOMIC_FLAG_INIT
#include <thread>         // std::thread, std::this_thread::yield
#include <vector>         // std::vector
#include <chrono>	  // TO add artificial delays. You can remove it.
bool cam1_switch = true;  
ROI_t cam1_roi;
std::atomic<short> cam1_available_index (0);
std::atomic<bool> cam1_NewFrame (false);
std::atomic<bool> cam1_NewROI (true);
ROI_t ROI = {ROI_WIDTH,ROI_HEIGHT, 1000,1000};



// std::atomic<int> &ThreadsCounter
cv::Mat cam1_Buffer[3];

int main() {
	int a;
	PoseEstimate Pose1;
	cv::Mat inputImage = cv::imread	("full_img.png");
	TattileCamera Cam1;
	Cam1.SetupIPAddress("192.168.1.6");
	Cam1.SetConnections(&cam1_NewFrame, &cam1_available_index , &cam1_switch, &cam1_NewROI, &ROI);
	std::cout<<"HERE-----------------------------------"<<std::endl;
	Pose1.SetBuffer(Cam1.GetBuffer());
	// Cam1.SetConnections(&cam1_NewFrame, &cam1_available_index , &cam1_switch, &cam1_NewROI, &ROI);
	// Cam1.SetupIPAddress("192.168.1.6");
	// Cam1.Run();	
	// std::thread t1(Cam1.Run());
	// std::thread t2(Pose1.Run());
	// t1.join();
	// t2.join();
	// Pose_1.ShowFrame(inputImage, "This name----");
	return 0;
}


