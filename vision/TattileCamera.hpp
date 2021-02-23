#ifndef TATTCAM_H
#define TATTCAM_H
/*****************************************************
common.h  This is the basic funciutons from the camer company: 

    Uses:
		common
		Opencv
		thread
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


// #include <iostream>
// #include <sstream>


class TattileCamera{
	private:
	uint8_t packet[sizeof(frame_t) + IMG_WIDTH * IMG_HEIGHT];  //packet for the full frame (12 MP)
	volatile bool done = false;
	struct sockaddr_in si_server;  //standard struct for socket package
	int sock, ret;
	struct sigaction action;
	
	// ROI 
	ROI_t roi;
	roi.x = 0;
	roi.y = 0;
	roi.width = ROI_WIDTH;
	roi.height = ROI_HEIGHT;


	public:
		TattileCamera();
		~TattileCamera();
		// TattileCamera(); Use this to connect to the IP address 
		bool Initial();
		bool SetupIPAddress(); // Sets the camera's IP address. You can find this form your router page (probably: 192.168.1.1--> pass : admin)
		bool SetRingBuffer(); //This is a 3 cell ring buffer with overwrite option (wont wait for the vision code)
		void PrintCameraInfo();
		void UpdateFrame(); //Updates the frame
		void GetCurrentFrame(cv::Mat * frame); //Returns the current frame from the camera (Type : opencv Mat) 
		cv::Mat GetCurrentFrame();	//Returns the current frame from the camera (Type : opencv Mat)
		void SetROI(ROI_t * roi); // This sets the next ROI @todo: make the ROI atomic or mutex!
		void sig_handler(int);
};
#endif // TATTCAM_H