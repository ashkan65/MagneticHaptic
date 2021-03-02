#include "PoseEstimate.hpp"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  


PoseEstimate::PoseEstimate(){
	parameters = cv::aruco::DetectorParameters::create();
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	MarkerSize = -1;
	read_loc = 2;
};

PoseEstimate::~PoseEstimate(){

};

void PoseEstimate::ShowFrame(const char *_name){
		
		cv::imshow(_name, frame);
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

bool PoseEstimate::SetConnections(bool * cam_switch, short &roi_x, short &roi_y, bool NewFrame){

};

bool PoseEstimate::SetCameraCalibration(){

};