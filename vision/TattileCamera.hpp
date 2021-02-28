#ifndef TATTCAM_H
#define TATTCAM_H
/*****************************************************
This is a modified version of Tattile camera code that the company provided.
To keep the FPS high we are using a buffer ring of 3. There is shared cv::mat [3] buffer which bosh camera and pose estimator use to pass data

Shared variables between caemra and pose estimate:

|--------|--------|--------|
| cv:Mat | cv:Mat | cv:Mat |  cv::Mat Buffer [3]
|--------|--------|--------|

							  short avaiable  Is there any new frame available for the pose estimator for. 
							  bool switch --> cam_switch and estimate_switch are both pointing at the sate location
							  bool Avaiable_frame 
							  short ROI[2]   The u,v of the corner of the ROI. The camera uses ROI_t struct but we only pass the [u,v] and later copy it to a locaol ROI_t

While you can use GetCurrentFrame to get the frame, to reach ~1000fps you need to define a buffer
from the outside and pass the pointer. More details on thiss will come later 

    Uses:
		Opencv
		thread
Ver 2.0 by Ashkan Feb-2021
TO avoid coping every frame, the Burref is not part of the camera calss
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

// The magic code     This is from camera api! leave it. 
#define CAM_MAGIC	(0xCEDAC0DE)

// The maximum size of the UDP Packets we're sending
// IP max len - IP header - UDP header
#define MAX_UDP_SIZE	(65535 - 20 - 8)

// Since we are overwriting on the unsued frames, 3 should be enough 
#define NUM_BUF			(3)


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

// Requested ROI and Image struct
typedef volatile struct {
	ROI_t roi;
	cv::Mat *m;
} RoiImg_t;


class TattileCamera{
	private:
		/////////////////////////////////////////////////////////
		// Camera server related variables  (UDP- TCP socket to the servern in the camera):
		// uint8_t packet[sizeof(frame_t) + ROI_WIDTH * ROI_HEIGHT];  //packet for the ROI frame (150 X 150)
		volatile bool done = false;		// another unclear cariable form the camera lib
		struct sockaddr_in si_server;	//standard struct for socket package
		int sock, ret;					// The socket to the camera (Tattile is basically a UDP cient that brodcasts the frames as a packet) 
		struct sigaction action;		// This is due to the bad codding from the camera company --> deal with it!
		ROI_t roi;						// ROI is a struct from camera company. To keep the atomic variables simple, we pass ints as roi.x and roi.y and then update the ROI
		const char *IP;					// IP to the camera. You can find the camera IP on router page (probably 192.168.1.1  admin ;)
		/////////////////////////////////////////////////////////
		// Camera output and control related variables:		
		
		short write_index;				//Where in the buffer are you writing. 				 
		cv::Mat* buffer;				// A pointer to an array of 3 cv::mat roi (150X150)--> we ended up using roi_m instead. 
		std::atomic<bool>* new_frame;	// Where on the buffer the camera should write the next frame
		std::atomic<short>* available_index;// Where on the buffer are you writing the current frame
		bool* cam_switch;				// This is the bainary switch that stops the camera. Turn it off and on from the outside. 
		// std::atomic<int>* ROI;			// A pointer to int[2] with u,v of the corner of the roi's location. 
		cv::Mat camera_roi;				// The arrived roi --> everytime we copy it to the buffer location before updating the next frame. 
		std::atomic<ROI_t>* ROI		// !!!! init it in setcinnections A pointer to requested roi. fill this from the pose estiamte or main --> add Kalman filter to improve the performance.
		/////////////////////////////////////////////////////////
		// The flow of the data is as follow: 
		// Firrst in constructore:
		// We alocate 3 (NUM_BUF) memory for three socket
		// We make 3 cv::Mats with sockets as the values and roi_width and heights --> in cosntructor
		// We make 3 RoiImg_t that has two members, roi infos and points to 2 cv::Mats we just made.
		// The index of the buffer is an atomic short which is shared with the pose estimator.
		// During the run:
		// From outside we set the roi.x and roi.y
		// We go to the correct index (available_index) in the img and update its roi -- also lock the index meaning it is not avaibale anymore
		// move the socket to the correct index of the 
		// We send the roi to the camera
		//
		cv::Mat *frame_buffer[NUM_BUF];	// Buffer of rois 
		uint8_t packetBuf[NUM_BUF][sizeof(frame_t) + IMG_WIDTH * IMG_HEIGHT]; // Allocate packet buffers (max size for packet)
		RoiImg_t img[NUM_BUF];			// The buffer of ROIImg_ts with cv::mat pointers to frame_buffer. 

	public:
		inline static std::vector<TattileCamera *> camera_instances;
		TattileCamera();
		~TattileCamera();
		ROI_t* GetROI_P();
		void SetupIPAddress(const char *_add); // Sets the camera's IP address. You can find this form your router page (probably: 192.168.1.1--> pass : admin)
		bool SetConnections( std::atomic<bool>* _new_frame , std::atomic<short>* _available_index , bool* _cam_switch , std::atomic<ROI_t>* _ROI); //This is a 3 cell ring buffer with overwrite option (wont wait for the vision code)
		void PrintCameraInfo();			// Do this later : Not sure what to print yet :D
		void Run();						//This runs the camera until you turn the switch off 
		void GetCurrentFrame(cv::Mat * frame); //Returns the current frame from the camera (Type : opencv Mat) 
		cv::Mat GetCurrentFrame();		//Returns the current frame from the camera (Type : opencv Mat)
		void sendROI(const int sock, const struct sockaddr *sa, volatile ROI_t *roi); // Sending ROI to the caemera
		void sendStop(int sock, struct sockaddr *sa);
		bool rx_frame(const int sock, const struct sockaddr_in *si_server, volatile ROI_t *roi, uint8_t *buffer);
		static void sig_handler2 (int);

		// static A * signal_object;
};
#endif // TATTCAM_H

// @todo: Finish (TATTI!) camera . 