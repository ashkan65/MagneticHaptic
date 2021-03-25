#include "TattileCamera.hpp"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  
    // @todo: Finish the code here 
    // @body: The interface needs more work!
// extern int camera_index[100];
// extern bool done[100];
// extern void sig_handler(int) {done[1] = true;}

TattileCamera::TattileCamera(){

	TattileCamera::camera_instances.push_back(this);
	// Setup signal handler for stopping the program 
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_flags = 0;
	// signal_object = this;
	action.sa_handler = sig_handler2;
	sigaction(SIGINT, &action, nullptr);
	sigaction(SIGTERM, &action, nullptr);
	write_index.store (1);
	swap_index.store (0);
	// Create the socket descriptor for getting data
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
	// Seting up the buffer and pre alocating the memory for NUM_BUFs frames and rois. 
	// Not the best structure and names which is due to the camera libs

	for (int i=0; i<NUM_BUF; i++) {
		packet = packetBuf[i];
		// Pre-allocate the cv::Mat for ROI sized images
		frame_buffer[i] = new cv::Mat(ROI_HEIGHT, ROI_WIDTH, CV_8UC1, (void *)(packet + sizeof(frame_t)), sizeof(uint8_t) * ROI_WIDTH);
		img[i].m = frame_buffer[i];
		img[i].roi.x = 0;
		img[i].roi.y = 0;
		img[i].roi.width = ROI_WIDTH;
		img[i].roi.height = ROI_HEIGHT;
	}
};
TattileCamera::~TattileCamera(){
	sendStop(sock, (struct sockaddr *)&si_server);
	std::cout<< "Camera "<< *IP<< std::endl;
	for (int i=0; i<NUM_BUF; i++) {
		// delete(ff_m[i]);
		delete(frame_buffer[i]);
	}

};
void TattileCamera::SetupIPAddress(const char *_add){
	IP = _add;
	// Set up the address
	memset(&si_server, 0, sizeof(struct sockaddr_in));
	si_server.sin_family = AF_INET;
	si_server.sin_port = htons(UDP_PORT);
	if (!inet_aton(_add, &si_server.sin_addr)) {
		printf("Unable to look up the server address: %s!\n", _add);
		exit(-1);
	}
};

void TattileCamera::Run(){
	// Switch is a bool pointer that truns on and off from outside of the code. 
	while(*cam_switch){
	// for(int i = 1; i<20000;i++){			// Gets 2000 frames 
		// Swapping the available index and writing index:
		// std::cout<<"W: "<<write_index<<" a: "<<*available_index <<std::endl;
		swap_index.store((*available_index).load());
		(*available_index).store(write_index.load());
		write_index.store(swap_index.load());
		// Updating the ROI x and y in the correct index
		// std::cout<<*cam_switch<<std::endl;
		img[write_index].roi.x = ROI->x;
		img[write_index].roi.y = ROI->y;
		packet = (uint8_t *)(packetBuf[write_index]);								// Updating the packet to point at the correct buffer cell
		sendROI( sock, (struct sockaddr *)&si_server, &(img[write_index].roi));		// Sendign the ROI to the camera
		ret = rx_frame(sock, &si_server, &(img[write_index].roi), packet);			// Capturing the incomming frame
		*new_frame = true;
	}
	*cam_switch = false;

};
volatile ROI_t* TattileCamera::GetROI_P(){
	return &(img[write_index].roi);
};

// For 1000fps do not use GetCurrentFrame functions. insetead read them from the buffer. --> check the TattileCamera.hpp file 
void TattileCamera::GetCurrentFrame(cv::Mat * frame){
	frame  = frame_buffer[write_index];
}; //Returns the current frame from the camera (Type : opencv Mat) 
cv::Mat TattileCamera::GetCurrentFrame(){
	return (*frame_buffer[write_index]);
};	//Returns the current frame from the camera (Type : opencv Mat)


bool TattileCamera::SetConnections(std::atomic<bool>* _new_frame , std::atomic<short>* _available_index , bool* _cam_switch , std::atomic<bool>* _new_ROI, ROI_t* _ROI){
	cam_switch = _cam_switch;
	ROI = _ROI;
	new_frame = _new_frame;
	available_index = _available_index;
};

// Send an ROI command
void TattileCamera::sendROI(const int sock, const struct sockaddr *sa, volatile ROI_t *roi)  {
	// Set up the frame to send
	frame_t f;
	f.magic = htonl(CAM_MAGIC);
	f.roi.x = roi->x;
	f.roi.y = roi->y;
	f.roi.width = roi->width;
	f.roi.height = roi->height;
	// These packets are short and should always fit in the buffer
	ssize_t status = sendto(sock, &f, sizeof(frame_t), 0, sa, sizeof(struct sockaddr_in));
	if (status < 0) {
		printf("Unable to send ROI packet, error code: %zd - %s!\n", status, strerror(errno));
		exit(-1);
	} else if (status != sizeof(frame_t)) {
		printf("Unable to send full frame_t: %zd bytes of %zu\n", status, sizeof(frame_t));
	}
};

// Send a stop command
void TattileCamera::sendStop(int sock, struct sockaddr *sa)  {
	// Set up the frame to send
	frame_t f;
	f.magic = htonl(CAM_MAGIC);
	memset(&(f.roi), 0, sizeof(ROI_t));
	// These packets are short and should always fit in the buffer
	ssize_t status = sendto(sock, &f, sizeof(frame_t), 0, sa, sizeof(struct sockaddr_in));
	if (status < 0) {
		printf("Unable to send STOP packet, error code: %zd - %s!\n", status, strerror(errno));
		exit(-1);
	} else if (status != sizeof(frame_t)) {
		printf("Unable to send full frame_t: %zd bytes of %zu\n", status, sizeof(frame_t));
	}
};

// Receive function
bool TattileCamera::rx_frame(const int sock, const struct sockaddr_in *si_server, volatile ROI_t *roi, uint8_t *buffer)  {
	struct timeval to;
	// Receive the data
	bool rcvd = false;
	int to_rx = sizeof(frame_t), rxd = 0, rx_len, to_rx_now;
	socklen_t sock_len = sizeof(struct sockaddr);
	frame_t *f = (frame_t *)buffer;

	to_rx += roi->width * roi->height;

	while ((!rcvd) && (!done)) {
		fd_set readfds;
		FD_ZERO(&readfds);
		FD_SET(sock, &readfds);

		// Set up the time out timeval, we will wait for 3 seconds
		to.tv_sec = 3;
		to.tv_usec = 0;

		// Check to see if there's data available
		int status = select(sock + 1, &readfds, nullptr, nullptr, &to);
		if (-1 == status) {
			if (EINTR == errno)
				continue;
			printf("Select failed\n");
			exit(-1);
		// We timed out
		} else if (0 == status) {
			printf("RX Frame timeout, got %d bytes\n", rxd);
			return false;
		}

		// Recieve data...
		if (to_rx > MAX_UDP_SIZE)
			to_rx_now = MAX_UDP_SIZE;
		else
			to_rx_now = to_rx;

		rx_len = recvfrom(sock, &(buffer[rxd]), to_rx_now, 0, (struct sockaddr *)&si_server, &sock_len);
		if (rx_len < 0) {
			printf("Error receiving: %d!\n" , rx_len);
			exit(-1);
		}

		// Figure out how much more data we have to rx
		to_rx -= rx_len;
		rxd += rx_len;

		// If we have received the whole packet, we will exit the loop
		if (to_rx == 0) {
			// Verify that the packet has the correct magic, if not, reset the packet
			if (!(ntohl(f->magic) == CAM_MAGIC))  {
				printf("Packet has incorrect magic, expected 0xCEDAC0DE, got %X\n", ntohl(f->magic));
				return false;
			} else {
				rcvd = true;
			}
		}
	}
	return true;
};

void TattileCamera::sig_handler2 (int) // calls the handlers
{
	for (int i = 0; i< camera_instances.size() ; i++){
		camera_instances[i]->done = true;
	}
	// this->done = true;
};

cv::Mat** TattileCamera::GetBuffer(){
	return frame_buffer;
};