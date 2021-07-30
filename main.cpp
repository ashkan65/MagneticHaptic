#include <iostream>
#include <vector>
#include "telerobcamera.hpp"
#include "GPIO826.hpp"
#include "controller.hpp"
#include <unistd.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/types.hpp>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main function.

int main()
{
	Mat_actuation A = Eigen::MatrixXd::Random(5, 8);
	vec_wrench WR(5,1);
	WR << 0.0 , 0.0 , 0.0 , 0.0 , 15.0;
	// std::cout<<A<<std::endl;
	// std::cout<<WR<<std::endl;
	vec_current I(8,1);
	Controller ex_C(8, 10.0, 50.0, 5);
	vec_temp_C temps(8,1);
	temps<< 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 155.0 ;
	ex_C.PINV(WR,  A, I);
	//std::cout<<"PINV I : "<<I<<"----------------"<<std::endl;
	
	// std::cout<<"PINV OUT: "<<A* I<<"----------------"<<std::endl;
	
	ex_C.RPINV(WR,  A, I);

	// std::cout<<"RPINV I: "<<I<<"----------------"<<std::endl;

	// std::cout<<"RPINV OUT: "<<A* I<<"----------------"<<std::endl;


	ex_C.RWPINV(WR,  A, temps , I);

	// std::cout<<"RWPINV I: "<<I<<"----------------"<<std::endl;

	// std::cout<<"RWPINV OUT: "<<A* I<<"----------------"<<std::endl;

/////////////////////////////////////////////
	double volt = -6.32;
	double volt2;
	uint channel = 5;
	GPIO826 Card1;

	// B826.SetDacOutput(&channel, &volt);
	// B826.GetDacOutput(&channel, &volt2);
	


	Card1.Init();
	vec_voltage voltage(8,1); 
	vec_voltage output_voltage(8,1);
	vec_current current(8,1);
	vec_current output_current(8,1);
	vec_voltage input_volt(16,1);
	vec_temp_C temperature(16,1);
	vec_voltage offset (8,1);
	offset << 0.008812, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // Amplifier offset
	Card1.SetOffset(&offset);
	
	
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Testing AMC Amplifiers 7.21.21
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	current << 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	voltage << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	// Output the appropriate signal from the DAC
	// Card1.AnalogWrite(&voltage);

	Card1.SetCoilsCurrent(&current);


	// Tell the user the "current" signals sent to the DAC
	//Card1.GetCoilsCurrent(&current);
	//std::cout << "This is the commanded current:\n" << current << std::endl;

	// Tell the user the voltage signals sent to the DAC
	// Card1.GetCoilsVoltage(&voltage);
	// std::cout << "This is the commanded voltage:\n" << voltage << std::endl;

	int a;
	std::cin>>a;
	

///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////


	Card1.PrintError(); 
	// Card1.GetDacOutput(&channel, &volt2);


	// B826.SetDacOutput(&channel, &volt);
	
	// Card1.GetDacOutput(&channel, &volt2);

	// Card1.GetCoilsVoltage(&output_voltage);

	// Card1.GetCoilsCurrent(&output_current);

	//Card1.AnalogRead();
	// Card1.AnalogRead(&input_volt);

	// Card1.GetCoilsTemperature(&temperature);

	// uint dios[] = {         // Specify DIOs that are to be turned on:
	// 	(1 << 7) + (1 << 13), //   DIOs 7 & 13 are in first 24-bit mask (DIOs 0-23),
	// 	(1 << (38 - 24))      //   DIO 38 is in second 24-bit mask (DIOs 24-47).
	// };
	// std::cout<<dios[0]<<std::endl;
	Card1.SysOff();
    return 0;
}



//////////////////////////////////