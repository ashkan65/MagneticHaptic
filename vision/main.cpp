#include "TattileCamera.hpp"
#include <iostream>

bool cam1_switch = true;  
ROI_t cam1_roi;
std::atomic<short> cam1_available_index (0);
std::atomic<bool> cam1_NewFrame (false);
cv::Mat cam1_Buffer[3];

int main() {
	int a;


	TattileCamera Cam1;
	Cam1.SetupIPAddress("192.168.1.6");
	return 0;
}


