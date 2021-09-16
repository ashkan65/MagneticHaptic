#ifndef GPIO826_H
#define GPIO826_H
/*****************************************************
GPIO (Sensoray 826 interface for Magnetic haptic device)
This class Inherents from simple826 class (git: https://github.com/ashkan65/SensorayWrapper)
 
Ver 1.0 by Ashkan Oct-2020		
*****************************************************/
#include <chrono>
#include <iostream>
#include "826api.h"
#include "simple826.hpp" //Parent class for this class 
#include "type.hpp"
// #include <thread>  //Might be better to do the threading outside!


class GPIO826: public Simple826 {
	private:
		///////////////////////////////////////////// 
		// There are eight coils attached to the card (coil 0-7)->DAOut(vetor<8>), also there are two thermocouples per coil (channels 0-15)->ADin(vector<16>)
		bool status;
		const int  NUMCOILS = 8;						// How many coils we have!

		vec_voltage * _PCoilVolts;				//voltages VECTOR set to send
		vec_current * _PCoilCurrents;			//currents VECTOR set to send
		vec_temp_C * _PCoilTemperatures;		//Temps VECTOR measured from coils
		vec_voltage * _P_TempVolt;				// Pointer to voltage vector for book keeping in SetCoilsCurrent.
		vec_voltage _TempVolt;
		vec_current _TempCurrent;				// Pointer to Current vector for book keeping in SetCoilsCurrent.
		vec_voltage _thermocouple_volt; 		// voltage vector for thermocouple
		vec_voltage * _offset; 					// Amplifier offset
		vec_voltage * _scale; 					// Amplifier scale

		bool ON = true;
		bool OFF = false;
		uint COIL_INHIBIT_MAP[8] = {0,1,2,3,4,5,6,7};	//Channel number for inhibit for Amplifiers		
		uint COIL_OUT_MAP[8] = {0,1,2,3,4,5,6,7};		//  List of control pins per coils : 0,1,2,3,4,5,6,7;	//The pin number with the correct order
 		uint  COIL_TEMP_MAP_A[8] = {0,2,4,6,8,10,12,14};//  List of Temp pins per coils; Channel A 
		uint  COIL_TEMP_MAP_B[8] = {1,3,5,7,9,11,13,15};//  List of Temp pins per coils; Channel B
		int adcbuf[16]; // Pointer to a buffer that will receive ADC results for the sixteen possible slots. The buffer must be large enough to
        //accommodate sixteen values regardless of the number of active slots. Each slot is represented by a 32-bit value,
        //which is stored at buf[SlotNumber].
		double data[16] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // data for each channel/slot

		// @TODO: Change these thermocouple calibration constants
		double a = 1.0;
		double b = 1.0;
		double c = 1.0;
		double d = 1.0;
		double e = 1.0;


	public:
		void SetProp(); 										//TODO
		void Init();											//TODO
		void SysOn();											//Turns the inhibits on 
		void SysOff();											//Turns the inhibits off
		void AnalogRead(vec_voltage * _voltage);				// Reads analog inputs from Sensoray card
		void GetCoilsTemperature(vec_temp_C * _temperature);	// Reads the temperature of the coils
		
		// These should be somewhere else:
		void GetCurrentTemp();
		void GetCoilsCurrent(vec_current * coilcurrent);
		void GetCoilsVoltage(vec_voltage & coilvolts);
		void SetCoilsCurrent(vec_current * coilcurrent);
		void SetCoilsVoltage();
		void SetOffset(vec_voltage * offset);
		void SetScale(vec_voltage * scale);
		void Current2Volt(vec_current * coilcurrent);	// Converts the current to voltage and saves it on _TempVolt!!!!
		void Volt2Current(vec_voltage * _coilvoltage, vec_current * coilcurrent);
		void AnalogWrite(vec_voltage * coilvolts);	//Sends the vectors of Currents to the device (This turns the coils on)


		// // cv::Mat GetCurrentA2D();	//Returns the current frame from the camera (Type : opencv Mat)
		// // cv::Mat FilterSetup();	//Returns the current frame from the camera (Type : opencv Mat)
		// // cv::Mat FilterSignal();
		// // cv::Point GetTargetPose();
		// void SaveCurrentFrame(std::string fileName); // Updates and saves the current frame to the filename. You can draw the coordiante for the marker and also add the markers ID for ALL OF THE IDs.   
		// void FilterCurrentFrame(); // This is a place holder for the filter process if case you want to implement something later   
};
#endif // GPIO826_H