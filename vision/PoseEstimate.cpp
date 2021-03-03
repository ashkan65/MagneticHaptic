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
			cv::imshow("Something", *buffer[read_index]);
			key = cv::waitKey(30);
		}
	}
};