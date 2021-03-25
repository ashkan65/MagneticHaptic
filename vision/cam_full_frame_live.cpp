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

uint8_t packet[sizeof(frame_t) + IMG_WIDTH * IMG_HEIGHT];

// Main loop
int main(int argc, char **argv) {

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

	cv::Mat full_m(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, (void *)(packet + sizeof(frame_t)), sizeof(uint8_t) * IMG_WIDTH);
	cv::Mat show_frame;
	roi.x = 0;
	roi.y = 0;
	roi.width = IMG_WIDTH;
	roi.height = IMG_HEIGHT;
	sendROI(sock, (struct sockaddr *)&si_server, &roi);
	// 2.1.3.5.b
	ret = rx_frame(sock, &si_server, &roi, packet);
	if (!ret) {
		printf("Unable to get the full frame!\n");
	}
	cv::namedWindow( "Display window", CV_WINDOW_NORMAL ); // Create a window for display.
	cv::resizeWindow("Display window", 900,1200);
	char key =0;
	int x = 1;
while(key != 'q') {
		////////////////////////////////////////////////////////////////
		/// Your Code Starts Here
		////////////////////////////////////////////////////////////////
		// 2.1.3.2
		sendROI(sock, (struct sockaddr *)&si_server, &roi);
		ret = rx_frame(sock, &si_server, &roi, packet);
		// cv::imwrite("sweep/dbg_full_img2.png", full_m);
		// show_frame.convertTo(full_m,CV_8UC3);
		full_m.copyTo(show_frame);

		// show_frame.release();
		cv::imshow( "Display window", show_frame);
		if (key == 'c')
        {
        	std::cout<<"frames captured!\n";
            char filename1[200];
            sprintf(filename1, "%sleft%d.%s", "images/", x, "jpg");
            cv::imwrite(filename1, show_frame);
        }
		// cv::resizeWindow("Display window", 600,800);
		key = cv::waitKey(30);
		// 2.1.3.5
		// Now get 10 frames at full resolution without the setup time
	}
	
	sendStop(sock, (struct sockaddr *)&si_server);
	printf("Sent stop packet\n");
	return 0;
}