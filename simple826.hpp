#ifndef SIMP826_H
#define SIMP826_H
/*****************************************************
A wrapper for sensoray card (826) to handle the ms updating rate!
Ver 1.0 by Ashkan Oct-2020		
*****************************************************/
#include <chrono>
#include "826api.h"
#include "type.hpp"
#include <thread>  //Might be better to do the threading outside!


class Simple826{
	protected:
		/////////////////////////////////////////////
		// int GrabNextImageByTrigger(INodeMap& nodeMap, CameraPtr pCam);
		// int ResetTrigger(INodeMap& nodeMap);
		// int ConfigureTrigger(INodeMap& nodeMap);
		// Helpful macros for DIOs
		uint board;                        // change this if you want to use other than board number 0
		int errcode;  
		int boardflags;        // open 826 driver and find all 826 boards

	public:
		Simple826();
		~Simple826();
		void SetDacOutput(uint *chan, double *volts);  //chan->channel number  volt->voltage
		void GetDacOutput(uint *chan, double *volts);  //chan->channel number  volt->voltage !!!! THIS IS NOT A ANALOG READ (this just returns the output current value)----
		int GetError();  // Retruns error 
		void PrintError(); //Prints error
		// cv::Mat GetCurrentD2A();	//Returns the current frame from the camera (Type : opencv Mat)
		// cv::Mat GetCurrentA2D();	//Returns the current frame from the camera (Type : opencv Mat)
		// cv::Mat FilterSetup();	//Returns the current frame from the camera (Type : opencv Mat)
		// cv::Mat FilterSignal();
		// cv::Point GetTargetPose();
		void SaveCurrentFrame(std::string fileName); // Updates and saves the current frame to the filename. You can draw the coordiante for the marker and also add the markers ID for ALL OF THE IDs.   
		void FilterCurrentFrame(); // This is a place holder for the filter process if case you want to implement something later   
};
#endif // SIMP826_H