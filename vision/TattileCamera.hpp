#ifndef TATTCAM_H
#define TATTCAM_H
/*****************************************************
common.h  This is the basic funciutons from the company: 

    Uses:
		common
		Opencv
Ver 1.0 by Ashkan Feb-2021
IF you are using the Tattile cameras, please first read the notes attached to the setup!!!!!!!!!!!!!
There are so many things that can go wrong.  		
*****************************************************/
#include <chrono>
#include "common.h"
#include "common.cpp"
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
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <stdlib.h>  
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <mutex>
#include <sstream>
#include <condition_variable>
#include <fstream>


// #include <iostream>
// #include <sstream>


class TattileCamera{
	private:
	// Add the IP stuff and roi


	public:
		TattileCamera();
		~TattileCamera();
		// TattileCamera(); Use this to connect to the IP address 
		int Initial();
		void PrintCameraInfo(FlyCapture2::CameraInfo *pCamInfo);
		void UpdateFrame(); //Updates the frame
		void GetCurrentFrame(cv::Mat * frame); //Returns the current frame from the camera (Type : opencv Mat) 
		cv::Mat GetCurrentFrame();	//Returns the current frame from the camera (Type : opencv Mat)
		void SetROI(ROI_t * roi);
};
#endif // TATTCAM_H