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
#include <thread>
#include <mutex>
#include <condition_variable>

#include "common.h"
// We must include the C++ file so that the code is inlined properly
#include "common.cpp"

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




// ROI sized frames
#define ROI_FRAME_COUNT		(10000)
// Full sized frame count
#define FULL_FRAME_COUNT	(10)
// Number of buffers
#define NUM_BUF			(3)

// Boolean indicating that the program is done
volatile bool done = false;

// Requested ROI and Image struct
typedef volatile struct {
	ROI_t roi;
	cv::Mat *m;
} RoiImg_t;

// The images and ROI for the images, triple buffered
static RoiImg_t img[NUM_BUF];
static volatile int roiCount = 0;

// The lock that indicates when there's a new image available
static std::mutex matLock;
static std::condition_variable matCV;

// Allocate packet buffers (max size for packet)
uint8_t packetBuf[NUM_BUF][sizeof(frame_t) + IMG_WIDTH * IMG_HEIGHT];

// NOTE: You MUST send a stop command, otherwise the camera must
// be rebooted. This handler takes care of shutting things down
// cleanly, ensuring that a stop gets sent.
static void sig_handler(int) {
	done = true;
	matCV.notify_all();
}

// Macro for moving to the next ROI
#define NEXT_ROI(inc_val) {\
	inc_val++;\
	if (inc_val >= NUM_BUF)\
			inc_val = 0;\
	roiCount++;\
}

// Macro for moving to the next Image
#define NEXT_IMG(inc_val) {\
	inc_val++;\
	if (inc_val >= NUM_BUF)\
			inc_val = 0;\
}

// Macro for setting up an img
#define SET_ROI(img, w,h,xval,yval) {\
	img.m = NULL;\
	img.roi.width = w;\
	img.roi.height = h;\
	img.roi.x = xval;\
	img.roi.y = yval;\
}

// 2.2.2 Processing Thread
void process(int sock, struct sockaddr *si_server) {
	// currImg holds the offset for the IMG we're processing this cycle
	// currROI holds the offset for the ROI we're setting this cycle
	int frame = 0, currImg = 0, currROI = 0, roiX = 0, roiY = 0;
	// m is the cv::Mat pointer for the current image
	cv::Mat *m = NULL;

	// Setup the clocking for benchmarking
	auto start = std::chrono::system_clock::now();
	auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> diff;

	// Allocate the cv::Mats used in the example OpenCV code
	cv::Mat mx(ROI_WIDTH, ROI_HEIGHT, CV_8UC1);
	cv::Mat my(ROI_WIDTH, ROI_HEIGHT, CV_8UC1);

	/// Prime the ROI pipeline on the camera with two initial ROI requests
	// 2.2.2.1
	// Set up the initial ROI request to set up the pipeline
	SET_ROI(img[currROI], ROI_WIDTH, ROI_HEIGHT, 0, 0);
	// Send the initial ROI request
	sendROI(sock, si_server, &(img[currROI].roi));
	NEXT_ROI(currROI);

	// 2.2.2.2
	// Set the next pipeline ROI
	SET_ROI(img[currROI], ROI_WIDTH, ROI_HEIGHT, 0, 0);
	// Send the first pipelined ROI request
	sendROI(sock, si_server, &(img[currROI].roi));
	NEXT_ROI(currROI);

	// 2.2.2.3 - Loop
	while(!done) {
		///////////////////////////////////////////////////////////////////////
		// Your code starts here
		///////////////////////////////////////////////////////////////////////

		// Get an ROI frame with the setup time
		start = std::chrono::system_clock::now();

		// 2.2.2.3.a
		// Wait for the resulting image
		// We wait on the matLock to notify us that there's an image
		m = NULL;
		while((!done) && (m == NULL)) {
			std::unique_lock<std::mutex> lck(matLock);
			matCV.wait(lck);

			// Get the image
			m = img[currImg].m;
		}
		if (done)
			break;
		NEXT_IMG(currImg);

		end = std::chrono::system_clock::now();
		diff = end - start;
		printf("Mode Switch to ROI + 1 Frame  | Elapsed seconds: %.9f\n", diff.count());

		// Start the FPS counter for the 10k ROI Frame loop
		start = std::chrono::system_clock::now();

		// 2.2.2.3.b
		// Send the next pipelined ROI value
		SET_ROI(img[currROI], ROI_WIDTH, ROI_HEIGHT, 0, 0);
		sendROI(sock, si_server, &(img[currROI].roi));
		NEXT_ROI(currROI);

		// Reset the ROI
		roiX = 0;
		roiY = 0;

		// 2.2.2.3.c
		// Loop until we're done!
		while((!done) && (frame < ROI_FRAME_COUNT)) {
			// 2.2.2.3.c.i
			m = NULL;
			while ((!done) && (m == NULL)) {
				// We wait on the matLock to notify us that there's an image
				std::unique_lock<std::mutex> lck(matLock);
				matCV.wait(lck);

				// Get the image
				m = img[currImg].m;
			}
			if (done)
				break;

			////////////////////////////////////////////////////////////////
			/// Note: This code writes out the image. It will not meet
			/// timing with this uncommented.
			////////////////////////////////////////////////////////////////
			// cv::imwrite("dbg_img.png", m);
			cv::aruco::detectMarkers(*m, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
			////////////////////////////////////////////////////////////////
			/// Note: This code must complete in about 450uS to maintain
			/// a frame rate of 1000 FPS. On CPUs with slower single
			/// threaded performance, this time may be lower. You will need
			/// to do some experimentation with your CPU to find this time.
			/// Replacing the work with usleep() calls is also a useful way
			/// to find the amount of time you have available.
	                ////////////////////////////////////////////////////////////////
			/// A note on supported ROI sizes:
			///
			/// This code only supports changing the ROI size to either:
			/// ROI_WIDTH x ROI_HEIGHT (152 x 152)
			///
			/// or
			///
			/// IMG_WIDTH x IMG_HEIGHT (4096 x 3072)
			///
			/// Other ROI sizes are not supported in order to meet timing
			////////////////////////////////////////////////////////////////
			/// WARNING: Changing the frame size has severe penalties for
			/// the frame rate, taking as much as a second to complete the
			/// operation.
			////////////////////////////////////////////////////////////////

			// 2.2.2.3.c.ii
			// Run sobel on the image in both directions
#if 1
			cv::Sobel(*m, mx, -1, 1, 0, 3);
			cv::Sobel(*m, my, -1, 0, 1, 3);
#else
			usleep(450);
#endif

			// 2.2.2.3.c.iii
			// Pipeline the full frame ROI requests
			if (frame >= (ROI_FRAME_COUNT - 2)) {
				SET_ROI(img[currROI], IMG_WIDTH, IMG_HEIGHT, 0, 0);
			// This demo just ignores the previous calculations and walks
			} else {
				roiX += ROI_WIDTH;
				if (roiX + ROI_WIDTH > IMG_WIDTH) {
					roiX = 0;
					roiY += ROI_HEIGHT;

					if (roiY + ROI_HEIGHT > IMG_HEIGHT)
						roiY = 0;
				}
				SET_ROI(img[currROI], ROI_WIDTH, ROI_HEIGHT, roiX, roiY);
			}

			// 2.2.2.3.c.iv
			// Send the ROI request
			sendROI(sock, si_server, &(img[currROI].roi));

			// Increment to the next ROI
			NEXT_ROI(currROI);

			// Increment to the next image
			NEXT_IMG(currImg);

			// Increment the frame counter
			frame++;
		}
		if (done)
			break;
		// 2.2.2.3.d
		end = std::chrono::system_clock::now();
		diff = end - start;
		printf("10000 ROI Frames              | Elapsed seconds: %.9f Average FPS: %.2f\n", diff.count(), frame / diff.count());
		start = end;
		frame = 0;

		// 2.2.2.3.e
		// Now get a single frame at full resolution, with the setup time
		start = std::chrono::system_clock::now();
		// Prepare the next full frame ROI
		SET_ROI(img[currROI], IMG_WIDTH, IMG_HEIGHT, 0, 0);
		// Send the ROI request
		sendROI(sock, si_server, &(img[currROI].roi));
		// Increment to the next ROI
		NEXT_ROI(currROI);

		// We wait on the matLock to notify us that there's an image
		m = NULL;
		while((!done) && (m == NULL)) {
			std::unique_lock<std::mutex> lck(matLock);
			matCV.wait(lck);

			// Get the image
			m = img[currImg].m;
		}
		if (done)
			break;

		// Increment to the next image
		NEXT_IMG(currImg);
		end = std::chrono::system_clock::now();
		diff = end - start;
		printf("Mode Switch to Full + 1 Frame | Elapsed seconds: %.9f\n", diff.count());

		// 2.2.2.3.f
		// Now get 10 frames at full resolution without the setup time
		start = std::chrono::system_clock::now();
		while((!done) && (frame < FULL_FRAME_COUNT)) {
			// We wait on the matLock to notify us that there's an image
			m = NULL;
			while((!done) && (m == NULL)) {
				std::unique_lock<std::mutex> lck(matLock);
				matCV.wait(lck);

				// Get the image
				m = img[currImg].m;
			}
			if (done)
				break;

			////////////////////////////////////////////////////////////////
			/// Note: This code writes out the image. It will not meet
			/// timing with this uncommented.
			////////////////////////////////////////////////////////////////
			// cv::imwrite("dbg_full_img.png", m);

			// Prepare the next ROI window frame
			if (frame >= (FULL_FRAME_COUNT - 2)) {
				SET_ROI(img[currROI], ROI_WIDTH, ROI_HEIGHT, 0, 0);
			// Prepare the next full frame ROI
			} else {
				SET_ROI(img[currROI], IMG_WIDTH, IMG_HEIGHT, 0, 0);
			}
			// Send the ROI request
			sendROI(sock, si_server, &(img[currROI].roi));
			NEXT_ROI(currROI);
			NEXT_IMG(currImg);
			frame++;
		}
		if (done)
			break;
		// 2.2.2.3.g
		end = std::chrono::system_clock::now();
		diff = end - start;
		printf("10 Full Frames		      | Elapsed seconds: %.9f Average FPS: %.2f\n\n", diff.count(), frame/diff.count());
		frame = 0;

		///////////////////////////////////////////////////////////////////////
		// Your code ends here
		///////////////////////////////////////////////////////////////////////
	}
	return;
}

///////////////////////////////////////////////////////////////////////////////
// Main loop
// Note: You should not need to modify this code!
///////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
	struct sockaddr_in si_server;
	int sock, ret, curImg = 0, i;
	uint8_t *packet = NULL;
	RoiImg_t *ri;

	// Check the parameters
	if (argc != 2) {
		printf("Usage:\n\t%s <server to connect to>\n", argv[0]);
		exit(-1);
	}

	/* Setup signal handler for stopping the program */
	struct sigaction action;
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_flags = 0;
	action.sa_handler = sig_handler;
	sigaction(SIGINT, &action, nullptr);
	sigaction(SIGTERM, &action, nullptr);

	// 2.2.3.1
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

	// Print a connection message so the user knows that we're connected
	printf("Connected to server: %s\n", argv[1]);

	// 2.2.3.2
	// Startup the processing thread
	std::thread proc(process, sock, (struct sockaddr *)&si_server);

	// 2.2.3.3
	///////////////////////////////////////////////////////////////////////
	// Set up thread affinity
	// Your CPU may need to have different affinities set for more steady
	// performance. We found on our machine that tying the main thread to
	// CPU 2 and the worker thread to CPU 3 helped.
	///////////////////////////////////////////////////////////////////////
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(3, &cpuset);

	// Set the thread affinity
	ret = pthread_setaffinity_np(proc.native_handle(), sizeof(cpu_set_t), &cpuset);
	if (ret != 0) {
		printf("Unable to set the thread afinity!\n");
		exit(-1);
	}

	// 2.2.3.4
	// Pre-allocate the cv::Mat for ROI sized images
	cv::Mat *roi_m[NUM_BUF];
	for (i=0; i<NUM_BUF; i++) {
		packet = packetBuf[i];
		roi_m[i] = new cv::Mat(ROI_HEIGHT, ROI_WIDTH, CV_8UC1, (void *)(packet + sizeof(frame_t)), sizeof(uint8_t) * ROI_WIDTH);
	}
	// Pre-allocate the cv::Mat for FULL FRAME sized images
	cv::Mat *ff_m[NUM_BUF];
	for (i=0; i<NUM_BUF; i++) {
		packet = packetBuf[i];
		ff_m[i] = new cv::Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, (void *)(packet + sizeof(frame_t)), sizeof(uint8_t) * IMG_WIDTH);
	}

	// 2.2.3.5
	while(!done) {
		// 2.2.3.5.a
		// Busy loop...
		if (roiCount == 0)
			continue;

		// Get a pointer to the image
		ri = &(img[curImg]);

		// Get the ROI associated with this Image
		volatile ROI_t *r = &(ri->roi);
		roiCount--;

		// Assign the correct CV Mat to the struct
		if (r->width == ROI_WIDTH)
			ri->m = roi_m[curImg];
		else
			ri->m = ff_m[curImg];

		// 2.2.3.5.b
		// Receive the frame into the correct buffer
		packet = (uint8_t *)(packetBuf[curImg]);
		ret = rx_frame(sock, &si_server, r, packet);
		if (!ret) {
			// If we didn't receive a frame, signal and exit...
			printf("Didn't receive a frame! Cowardly exiting...\n");
			done = true;
			continue;
		}

		// 2.2.3.5.c
		// Let the processing thread know that we have data to send
		matCV.notify_one();

		// Move to the next buffer
		NEXT_IMG(curImg);
	}

	// Notify the thread that we're done
	matCV.notify_all();
	// Join up
	proc.join();

	// Send a stop command
	sendStop(sock, (struct sockaddr *)&si_server);
	printf("Sent stop packet\n");

	for (i=0; i<NUM_BUF; i++) {
		delete(ff_m[i]);
		delete(roi_m[i]);
	}
	return 0;
}
