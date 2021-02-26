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

cv::Mat cam1_Buffer[3];

int main() {
	int a;

	PoseEstimate Pose_1;
	TattileCamera Cam1;
	Pose
	Cam1.SetupIPAddress("192.168.1.6");
	Pose_1.ShowFraem(Cam1.GetCurrentFrame(), "This name----")
	return 0;
}


