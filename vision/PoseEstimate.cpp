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
	new_pose.store(false);
	// *new_pose = false;
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

bool PoseEstimate::SetConnections( std::atomic<bool>* _new_frame , std::atomic<short>* _available_index , bool* _vision_switch , uint16_t* _ROI_x, uint16_t* _ROI_y){
	new_frame = _new_frame;
	available_index = _available_index;
	vision_switch = _vision_switch;
	ROI_x = _ROI_x;
	ROI_y = _ROI_y;
};

bool PoseEstimate::SetCameraCalibration(std::string calibrationAddress){
	cv::FileStorage fs2(calibrationAddress, cv::FileStorage::READ);
	fs2["Camera Matrix"] >> cameraMatrix;
	fs2["Dist Coeffs"] >> distCoeffs;
	std::cout<< "Camera Matrix" << cameraMatrix<<std::endl;
	std::cout<< "Distortion " << distCoeffs<<std::endl;
	fs2.release();
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
	// cv::namedWindow	(	"FullFrame",cv::WINDOW_AUTOSIZE );
	cv::Mat ROI_frame;
	cv::addWeighted(*buffer[read_index], 6.0, *buffer[read_index], 6.0, -100, current_frame);// Manually increase the ISO
	cv::aruco::detectMarkers(current_frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);  // Detecting Aruco markers
	cv::Mat full_frame = cv::Mat::zeros(cv::Size(4096, 3072), CV_8U);
	current_frame.copyTo(full_frame(cv::Rect(*ROI_x, *ROI_y, current_frame.cols, current_frame.rows)));

	if (markerIds.size() > 0){	// Check to see if any marker is been detected
		// std::cout<<*ROI_x<<std::endl;
		// current_frame.copyTo(full_frame(cv::Rect(*ROI_x, *ROI_y,current_frame.cols, current_frame.rows)));
		///////////////////////////////////////////////////////////////////////////////////
		// Use this block to draw detected markers and estimated axis on the curretn frame.
		///////////////////////////////////////////////////////////////////////////////////
		// Shifting the ditected corners from ROI frame to FULL frame 
		for (int i= 0; i < markerIds.size(); i++){
			markerCorners[i][0].x += *ROI_x;
			markerCorners[i][0].y += *ROI_y;
			markerCorners[i][1].x += *ROI_x;
			markerCorners[i][1].y += *ROI_y;
			markerCorners[i][2].x += *ROI_x;
			markerCorners[i][2].y += *ROI_y;
			markerCorners[i][3].x += *ROI_x;
			markerCorners[i][3].y += *ROI_y;
		};
		cv::aruco::estimatePoseSingleMarkers(markerCorners, 1.5, cameraMatrix, distCoeffs, rvecs, tvecs);  // (DO NOT RUN THIS) without calibration.
		cv::cvtColor(current_frame,current_frame, cv::COLOR_GRAY2RGB );	//Ggray2RGB conversion. The current_frame is gray scale and the drawing will become BW
		//// I would only keep one of these drawings. 
		cv::aruco::drawDetectedMarkers(full_frame, markerCorners, markerIds);	//Drawing on the detected marker	
		cv::cvtColor(full_frame,full_frame, cv::COLOR_GRAY2RGB );
		cv::aruco::drawAxis(full_frame, cameraMatrix, distCoeffs, rvecs, tvecs, 20);
		// cv::resize(full_frame, full_frame, cv::Size(), 0.2, 0.2);
		new_pose = true;
		// std::cout<<tvecs[0]<<std::endl;
		// void cv::projectPoints	(tvecs,	rvec, tvec,	cameraMatrix, distCoeffs, imagePoints);	
	}
	cv::namedWindow( "FullFrame", cv::WINDOW_NORMAL );	
	cv::resizeWindow( "FullFrame" , 600,800 );
	cv::imshow("FullFrame", full_frame);
	cv::imshow("ROI", current_frame);
	if (key =='q'){
		*vision_switch = false;
	}
	key = cv::waitKey(30);
};

std::vector<cv::Vec3d>* PoseEstimate::GetTargetLoc_P(){
	// std::cout<<"This is Tvec:   "<<tvecs[0]<<std::endl;
	return &tvecs;
};

std::vector<cv::Vec3d>* PoseEstimate::GetTargetRot_P(){
	return &rvecs;
};

std::atomic<bool>* PoseEstimate::GetNewPose_P(){
	return &new_pose;
};  

