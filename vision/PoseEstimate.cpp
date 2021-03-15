#include "PoseEstimate.hpp"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  


PoseEstimate::PoseEstimate(){
	parameters = cv::aruco::DetectorParameters::create();
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	MarkerSize = -1;
	read_index.store (2);
	swap_index.store (0);
	buffer = NULL; 
};

PoseEstimate::~PoseEstimate(){
};

void PoseEstimate::ShowFrame(const char *_name){
		
		cv::imshow(_name, *buffer[read_index.load()]);
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

bool PoseEstimate::SetConnections( std::atomic<bool>* _new_frame , std::atomic<short>* _available_index , bool* _vision_switch , std::atomic<bool>* _new_ROI, uint16_t* _ROI_x, uint16_t* _ROI_y){
	new_frame = _new_frame;
	available_index = _available_index;
	vision_switch = _vision_switch;
	ROI_x = _ROI_x;
	ROI_y = _ROI_y;
	new_ROI = _new_ROI;
};

bool PoseEstimate::SetCameraCalibration(){

};

void PoseEstimate::SetBuffer(cv::Mat**_buffer){
	buffer = _buffer;
}; 

void PoseEstimate::Run(){
	//////////// FPS calculator:
	int frame_count;
	auto start = std::chrono::system_clock::now();
	auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> diff;
	///////
	start = std::chrono::system_clock::now();
	while(*vision_switch){
		if ((*new_frame).load()){
			// std::cout<<"R: "<<-1*read_index<<"     a: "<<-1**available_index<<std::endl;
			swap_index.store((*available_index).load());
			(*available_index).store(read_index.load());
			read_index.store(swap_index.load());

			// *buffer[read_index]; This is the current frame. To keep things clean do the processing in the filter function 
			Filter();
			// cv::imshow("Something1", *buffer[read_index]);
			// key = cv::waitKey(30);
			frame_count++;
			(*new_frame).store(false);
		}
	}
	end = std::chrono::system_clock::now();
	diff = end - start;
	std::cout<<"FPS:"<<frame_count/diff.count()<<std::endl;
};

void PoseEstimate::Filter(){
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
		cv::cvtColor(current_frame,current_frame, cv::COLOR_GRAY2RGB );	//Ggray2RGB conversion. The current_frame is gray scale and the drawing will become BW
		// std::cout<<markerCorners[0][0]<<std::endl;
		//// I would only keep one of these drawings. 
		cv::aruco::drawDetectedMarkers(current_frame, markerCorners, markerIds);	//Drawing on the detected marker	
		// cv::aruco::drawAxis(show_frame, cameraMatrix, distCoeffs, rvecs, tvecs, 20);
	}
	// cv::imshow("Something", current_frame);
	// key = cv::waitKey(30);
};