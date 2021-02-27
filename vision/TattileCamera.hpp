#ifndef TATTCAM_H
#define TATTCAM_H
/*****************************************************
While you can use GetCurrentFrame to get the frame, to reach ~1000fps you need to define a buffer
from the outside and pass the pointer. More details on thiss will come later 

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

// ROI Dimensions
#define ROI_WIDTH	152
#define ROI_HEIGHT	152

#define L_ROI_WIDTH	 252
#define L_ROI_HEIGHT 252

// Full Image Dimensions
#define IMG_WIDTH	4096
#define IMG_HEIGHT	3072

// The port we'll use
#define UDP_PORT	2368

// The magic code
#define CAM_MAGIC	(0xCEDAC0DE)

// The maximum size of the UDP Packets we're sending
// IP max len - IP header - UDP header
#define MAX_UDP_SIZE	(65535 - 20 - 8)

// ROI Type
typedef struct {
	uint16_t width;
	uint16_t height;
	uint16_t x;
	uint16_t y;
} ROI_t;

typedef struct {
	// Magic code
	uint32_t magic;
	// ROI Dimensions
	ROI_t roi;
} frame_t;



class TattileCamera{
	private:
		uint8_t packet[sizeof(frame_t) + ROI_WIDTH * ROI_HEIGHT];  //packet for the full frame (12 MP)
		volatile bool done = false;
		struct sockaddr_in si_server;  //standard struct for socket package
		int sock, ret;
		struct sigaction action;
		bool * cam_switch;
		ROI_t roi;
		const char *IP;
		short write_loc;
		// int camera_index;
	public:
		inline static std::vector<TattileCamera *> camera_instances;
		TattileCamera();
		~TattileCamera();
		// TattileCamera(); Use this to connect to the IP address 
		ROI_t* GetROI_P();
		void SetupIPAddress(const char *_add); // Sets the camera's IP address. You can find this form your router page (probably: 192.168.1.1--> pass : admin)
		bool SetConnections(cv::Mat (&buffer)[3], std::atomic<bool> &NewFrame , std::atomic<short> &available_index, bool &cam_switch, std::atomic<int> (&ROI)[2]); //This is a 3 cell ring buffer with overwrite option (wont wait for the vision code)
		void PrintCameraInfo();
		void Run(); //This runs the camera until you turn the switch off 
		void UpdateFrame(); //Updates the frame
		void GetCurrentFrame(cv::Mat * frame); //Returns the current frame from the camera (Type : opencv Mat) 
		cv::Mat GetCurrentFrame();	//Returns the current frame from the camera (Type : opencv Mat)
		void SetROI(ROI_t * roi); // This sets the next ROI @todo: make the ROI atomic or mutex!
		void sendROI(const int sock, const struct sockaddr *sa, volatile ROI_t *roi); // Sending ROI to the caemera
		void sendStop(int sock, struct sockaddr *sa);
		bool rx_frame(const int sock, const struct sockaddr_in *si_server, volatile ROI_t *roi, uint8_t *buffer);
		static void sig_handler2 (int);
		// static A * signal_object;
};
#endif // TATTCAM_H

// @todo: Finish (TATTI!) camera . 