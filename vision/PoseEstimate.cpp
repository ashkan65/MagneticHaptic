#include "PoseEstimate.hpp"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  


PoseEstimate::PoseEstimate(){
	parameters = cv::aruco::DetectorParameters::create();
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	MarkerSize = -1;
	read_index = 2;
	buffer = NULL; 
};

PoseEstimate::~PoseEstimate(){
};

void PoseEstimate::ShowFrame(const char *_name){
		
		cv::imshow(_name, *buffer[read_index]);
		key = cv::waitKey(30);
};

void PoseEstimate::ShowFrame(cv::Mat & _frame, const char *_name){
		cv::imshow(_name, _frame);
		key = cv::waitKey(30);
};

void PoseEstimate::ThroughMirror(bool _mirror){

};

void PoseEstimate::SetMarkerSize(int _size){
	MarkerSize = _size;
};

bool PoseEstimate::SetConnections( std::atomic<bool>* _new_frame , std::atomic<short>* _available_index , bool* _vision_switch , std::atomic<bool>* _new_ROI, int* _ROI){
	new_frame = _new_frame;
	available_index = _available_index;
	vision_switch = _vision_switch;
	ROI = _ROI;
	new_ROI = _new_ROI;
};

bool PoseEstimate::SetCameraCalibration(){

};

void PoseEstimate::SetBuffer(cv::Mat**_buffer){
	buffer = _buffer;
}; 

void PoseEstimate::Run(){
	while(*vision_switch){
		if ((*new_frame).load()){
			swap_index = *available_index;
			*available_index = read_index;
			read_index = swap_index;
			// *buffer[read_index]; This is the current frame. To keep things clean do the processing in the filter function 
			Filter();
			cv::imshow("Something", *buffer[read_index]);
			key = cv::waitKey(30);
			*new_frame = false;
		}
	}
};

void PoseEstiamte::Filter(){
	// If you are detecting markers through a mirror you have to print the fliped marker
	// otherwise the code cannot detect them.
	// *buffer[read_index] // This is the current frame
	cv::addWeighted(*buffer[read_index], 6.0, *buffer[read_index], 6.0, -100, current_frame);// Manually increase the ISO
	cv::aruco::detectMarkers(current_frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);  // Detecting Aruco markers
	if (markerIds.size() > 0){	// Check to see if any marker is been detected
		// cv::aruco::estimatePoseSingleMarkers(markerCorners, TargetSize, cameraMatrix, distCoeffs, rvecs, tvecs);  // (DO NOT RUN THIS) without calibration.
		///////////////////////////////////////////////////////////////////////////////////
		// Use this block to draw detected markers and estimated axis on the curretn frame.
		///////////////////////////////////////////////////////////////////////////////////
		// cv::cvtColor(current_frame,current_frame, cv::COLOR_GRAY2RGB );	Ggray2RGB conversion. The current_frame is gray scale and the drawing will become BW
		//// I would only keep one of these drawings. 
		// cv::aruco::drawDetectedMarkers(current_frame, markerCorners, markerIds);	Drawing on the detected marker	
		// cv::aruco::drawAxis(show_frame, cameraMatrix, distCoeffs, rvecs, tvecs, 20);
	}
};