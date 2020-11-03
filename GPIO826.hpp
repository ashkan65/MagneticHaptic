#ifndef GPIO826_H
#define GPIO826_H
/*****************************************************
GPIO (Sensoray 826 interface for Mahnetic haptic device)
This class Inherents from simple826 class (git: https://github.com/ashkan65/SensorayWrapper)
 
Ver 1.0 by Ashkan Oct-2020		
*****************************************************/
#include <chrono>
#include <iostream>
#include "826api.h"
#include "simple826.hpp" //Parent class fot this class 
#include "type.hpp"
// #include <thread>  //Might be better to do the threading outside!


class GPIO826: public Simple826 {
	private:
		///////////////////////////////////////////// 
		// There are eight coils attached to the card (coil 0-7)->DAOut(vetor<8>), also there are two thermolcoples per coil (channels 0-15)->ADin(vector<16>)
		bool status;
		vec_voltage * _PCoilVolts;				//voltages VECTOR set to send
		vec_current * _PCoilCurrents;			//currents VECTOR set to send
		vec_temp_C * _PCoilTemperatures;		//Temps VECTOR measured from coils
		bool ON = true;
		bool OFF = false;
		const int  NUMCOILS = 8;						// How many coils we have!
		uint COIL_INHIBIT_MAP[8] = {0,1,2,3,4,5,6,7};	//Channel number for inhibit for Amplifiers		
		uint COIL_OUT_MAP[8] = {0,1,2,3,4,5,6,7};		//  List of controll pins per coils : 0,1,2,3,4,5,6,7;	//The pin number with the correct order
 		uint  COIL_TEMP_MAP_A[8] = {0,2,4,6,8,10,12,14};//  List of Temp pins per coils; Channel A 
		uint  COIL_TEMP_MAP_B[8] = {1,3,5,7,9,11,13,15};//  List of Temp pins per coils; Channel B
	public:
		void SetProp(); //TODO
		void Init();	//TODO
		void SysOn();									//Turns the inhebits on 
		void SysOff();									//Turns the inhebits off
		void AnalogWrite(vec_voltage * coilvolts);	//Sends the vectors of Currents to the device (This turns the coils on)
		void AnalogRead(vec_temp_C * coiltemps);
		
		// These should be somewhere else:
		void Current2Volt();
		void GetCurrentTemp();
		void GetCoilsCurrent();
		void GetCoilsVoltage();
		void SetCoilsCurrent();
		void SetCoilsVoltage();

		// // cv::Mat GetCurrentA2D();	//Returns the current frame from the camera (Type : opencv Mat)
		// // cv::Mat FilterSetup();	//Returns the current frame from the camera (Type : opencv Mat)
		// // cv::Mat FilterSignal();
		// // cv::Point GetTargetPose();
		// void SaveCurrentFrame(std::string fileName); // Updates and saves the current frame to the filename. You can draw the coordiante for the marker and also add the markers ID for ALL OF THE IDs.   
		// void FilterCurrentFrame(); // This is a place holder for the filter process if case you want to implement something later   
};
#endif // GPIO826_H