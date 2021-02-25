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

class PoseEstimate{
	private:
		short read_loc; // Set this to 2 in the constructor
		cv::Mat frame;
		
	public:
		PoseEstimate();
		~PoseEstimate();
		ShowFrame();

};
#endif // POSEEST_H