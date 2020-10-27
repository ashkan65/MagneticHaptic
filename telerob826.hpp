#ifndef TEL826_H
#define TEL826_H
/*****************************************************
type.h  (requires opencv and spinlib library)  defines matrix types: 

    Inherits:
		Flycapture
		Opencv
Ver 1.0 by Ashkan July-2019		
*****************************************************/
#include <chrono>
#include "826api.h"
#include <thread>


class Telerob826{
	private:
	    /////////////////////////////////////////////
		// int GrabNextImageByTrigger(INodeMap& nodeMap, CameraPtr pCam);
		// int ResetTrigger(INodeMap& nodeMap);
		// int ConfigureTrigger(INodeMap& nodeMap);
	public:
		Telerob826();
		~Telerob826();
		// Telerob826(); 
		int Initial();
		void PrintCardInfo();
		void UpdateD2A(); //Updates the frame
		void SetD2A();
		// cv::Mat GetCurrentD2A();	//Returns the current frame from the camera (Type : opencv Mat)
		// cv::Mat GetCurrentA2D();	//Returns the current frame from the camera (Type : opencv Mat)
		// cv::Mat FilterSetup();	//Returns the current frame from the camera (Type : opencv Mat)
		// cv::Mat FilterSignal();
		// cv::Point GetTargetPose();
		void SaveCurrentFrame(std::string fileName); // Updates and saves the current frame to the filename. You can draw the coordiante for the marker and also add the markers ID for ALL OF THE IDs.   
		void FilterCurrentFrame(); // This is a place holder for the filter process if case you want to implement something later   
};
#endif // TEL826_H