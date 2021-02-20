///////////////////////////////////////////////////////////////////////////////
/// 2018 - ConcurrentEDA
///////////////////////////////////////////////////////////////////////////////

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
#include "common.h"
#include <stdlib.h>  
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <mutex>
#include <sstream>
#include <condition_variable>
#include <fstream>
using namespace std;
// Unfortunately, this code had to be written this way to allow inlining.
#include "common.cpp"

// Global done signal
volatile bool done = false;

static const char *const out_dir = "sweep/";
static const char *const add_dir = "address/";

// Sets the done flag true so that the stop command will be sent
static void sig_handler(int) {
	done = true;
}
//////////////////////////////////////////////////
// ARUCO stuff
//////////////////////////////////////////////////
std::vector<int> markerIds;
std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
cv::Mat cameraMatrix, distCoeffs;
std::vector<cv::Vec3d> rvecs, tvecs;
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
int TargetSize = 10;
//////////////////////////////////////////////////
// Timing stuff
//////////////////////////////////////////////////
std::chrono::duration<double> diff;
double FPS;
//////////////////////////////////////////////////

uint8_t packet[sizeof(frame_t) + IMG_WIDTH * IMG_HEIGHT];

// Main loop
int main(int argc, char **argv) {
auto start = std::chrono::system_clock::now();
auto end = std::chrono::system_clock::now();


	mkdir(out_dir, 0755);
	mkdir(add_dir, 0755);

	struct sockaddr_in si_server;
	int sock, ret;
	cv::Point min_loc, max_loc;

	if (argc != 2) {
		printf("Usage:\n\t%s <server to connect to>\n", argv[0]);
		exit(-1);
	}

	// ROI
	ROI_t roi;
	roi.x = 0;
	roi.y = 0;
	roi.width = ROI_WIDTH;
	roi.height = ROI_HEIGHT;

	/* Setup signal handler for stopping the program */
	struct sigaction action;
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_flags = 0;
	action.sa_handler = sig_handler;
	sigaction(SIGINT, &action, nullptr);
	sigaction(SIGTERM, &action, nullptr);

	// Create the socket for getting data
	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock == -1) {
		printf("Unable to open the socket!\n");
		exit(-1);
	}

	// Increase the socket buffer size to reduce the chance of losing data
	const int rxbuf_sz = IMG_WIDTH * IMG_HEIGHT + sizeof(frame_t);
	ret = setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rxbuf_sz, sizeof(int));
	if (-1 == ret) {
		printf("Unable to set the rx buffer size\n");
		exit(-1);
	}

	// Set up the address
	memset(&si_server, 0, sizeof(struct sockaddr_in));
	si_server.sin_family = AF_INET;
	si_server.sin_port = htons(UDP_PORT);
	if (!inet_aton(argv[1], &si_server.sin_addr)) {
		printf("Unable to look up the server address: %s!\n", argv[1]);
		exit(-1);
	}

	cv::Mat full_m(ROI_WIDTH, ROI_WIDTH, CV_8UC1, (void *)(packet + sizeof(frame_t)), sizeof(uint8_t) * ROI_WIDTH);
	cv::Mat show_frame;
	roi.x = 2800;
	roi.y = 1200;
	roi.x = roi.x - roi.x % 16;
	roi.y = roi.y - roi.y % 16;
	
	roi.width = ROI_WIDTH;
	roi.height = ROI_WIDTH;
	sendROI(sock, (struct sockaddr *)&si_server, &roi);
	// 2.1.3.5.b
	ret = rx_frame(sock, &si_server, &roi, packet);
	if (!ret) {
		printf("Unable to get the full frame!\n");
	}
	cv::namedWindow( "Display window", CV_WINDOW_NORMAL ); // Create a window for display.
	cv::resizeWindow("Display window", 1200,1600);
	char key =0;
while(key != 'q') {
		





		////////////////////////////////////////////////////////////////
		/// Image loop
		////////////////////////////////////////////////////////////////
		// sendROI(sock, (struct sockaddr *)&si_server, &roi);
		// ret = rx_frame(sock, &si_server, &roi, packet);
		// full_m.copyTo(show_frame);
		// // show_frame.convertTo(show_frame,CV_8UC3);

		// cv::addWeighted(show_frame, 6.0, show_frame, 6.0, -100, show_frame);
		// cv::aruco::detectMarkers(show_frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
		// //////////////////////////////////////////////////////////////////////////
		// // Adding the 
		// std::cout << markerIds.size() << std::endl;
		// if (markerIds.size() > 0){
		// std::cout << markerCorners[0][0]  << std::endl;
		// // cv::aruco::estimatePoseSingleMarkers(markerCorners, TargetSize, cameraMatrix, distCoeffs, rvecs, tvecs);  // DO NOT RUN THIS without calibration.
		// cv::cvtColor(show_frame,show_frame, cv::COLOR_GRAY2RGB );

		// // Only keep one:
		// 	cv::aruco::drawDetectedMarkers(show_frame, markerCorners, markerIds);
		// 	// cv::aruco::drawAxis(show_frame, cameraMatrix, distCoeffs, rvecs, tvecs, 20);
		// }
		//////////////////////////////////////////////////////////////////////////





// Checking FPS: Only use this to find the latency:
		start = std::chrono::system_clock::now();
		for(int i = 0; i < 1000 ; i++) {
		sendROI(sock, (struct sockaddr *)&si_server, &roi);
		ret = rx_frame(sock, &si_server, &roi, packet);
		full_m.copyTo(show_frame);
		cv::addWeighted(show_frame, 6.0, show_frame, 6.0, -100, show_frame);
		cv::aruco::detectMarkers(show_frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
		} 
		end = std::chrono::system_clock::now();
		diff = end - start;
		FPS = 1000/diff.count();
		std::cout << FPS << "   (FPS) " << std::endl;
////////////////////////////////////////////////////

		// cv::imshow( "Display window", show_frame);
		key = cv::waitKey(30);
	}
	
	sendStop(sock, (struct sockaddr *)&si_server);
	printf("Sent stop packet\n");
	return 0;
}