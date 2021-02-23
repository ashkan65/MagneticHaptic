#include "TattileCamera.hpp"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  
    // @todo: Finish the code here 
    // @body: The interface needs more work!


TattileCamera::TattileCamera(){
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
};
TattileCamera::~TattileCamera(){
// @todo: Be sure to stop the camera, close the port ...
};
int TattileCamera::Initial(){

};
void TattileCamera::PrintCameraInfo(FlyCapture2::CameraInfo *pCamInfo){

};
void TattileCamera::UpdateFrame(){

}; //Updates the frame
void TattileCamera::GetCurrentFrame(cv::Mat * frame){

}; //Returns the current frame from the camera (Type : opencv Mat) 
cv::Mat TattileCamera::GetCurrentFrame(){

};	//Returns the current frame from the camera (Type : opencv Mat)
void TattileCamera::SetROI(ROI_t * roi){

};
void TattileCamera::sig_handler(int _i) {
// Sets the done flag true so that the stop command will be sent
	done = true;
};
