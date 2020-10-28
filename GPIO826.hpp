#ifndef GPIO826_H
#define GPIO826_H
/*****************************************************
GPIO (Sensoray 826 interface for Mahnetic haptic device)
This class Inherents from simple826 class (git: https://github.com/ashkan65/SensorayWrapper)
Lots of 
Ver 1.0 by Ashkan Oct-2020		
*****************************************************/
#include <chrono>
#include <iostream>
#include "826api.h"
#include "simple826.hpp" //Parent class fot this class 
#include "type.hpp"
#include <thread>  //Might be better to do the threading outside!


class GPIO826: public Simple826 {
	private:
		///////////////////////////////////////////// 
		// There are eight coils attached to the card (coil 0-7)->DAOut(vetor<8>), also there are two thermolcoples per coil (channels 0-15)->ADin(vector<16>)
		bool status;
		vec_current _CoilVolts;				//voltages VECTOR set to send
		vec_voltage _CoilCurrents;			//currents VECTOR set to send
		vec_temp_C _CoilTemperatures;		//Temps VECTOR measured from coils
		// Eigen::VectorXi CoilVoltChannels(8);
		// Eigen::VectorXi CoilTempChannels(16);



	public:
		void SetProp(); //TODO
		void Init();	//TODO
		void SysOn();	//TODO
		void SysOff();	//TODO
		void AnalogWrite(vec_current  coilcurrents);	//Sends the vectors of Currents to the device (This turns the coils on)

		// // cv::Mat GetCurrentA2D();	//Returns the current frame from the camera (Type : opencv Mat)
		// // cv::Mat FilterSetup();	//Returns the current frame from the camera (Type : opencv Mat)
		// // cv::Mat FilterSignal();
		// // cv::Point GetTargetPose();
		// void SaveCurrentFrame(std::string fileName); // Updates and saves the current frame to the filename. You can draw the coordiante for the marker and also add the markers ID for ALL OF THE IDs.   
		// void FilterCurrentFrame(); // This is a place holder for the filter process if case you want to implement something later   
};
#endif // GPIO826_H