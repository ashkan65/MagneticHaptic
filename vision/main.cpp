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
ROI_t ROI = {ROI_WIDTH,ROI_HEIGHT, 1200,1200};



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
	Pose1.SetCameraCalibration("../calibration/cam_left.yml");
	Pose1.SetBuffer(Cam1.GetBuffer());
	Pose1.SetConnections(&cam1_NewFrame, &cam1_available_index , &cam1_switch, &cam1_NewROI, &(ROI.x), &(ROI.y));
	
	// Cam1.SetConnections(&cam1_NewFrame, &cam1_available_index , &cam1_switch, &cam1_NewROI, &ROI);
	// Cam1.SetupIPAddress("192.168.1.6");
	// Cam1.Run();	
	std::thread thread1(std::mem_fn(&TattileCamera::Run), &Cam1);
	std::thread thread2(std::mem_fn(&PoseEstimate::Run), &Pose1);
	thread1.join();	
	thread2.join();	
	// Pose_1.ShowFrame(inputImage, "This name----");
	return 0;
}


