#include <iostream>
#include <vector>
#include "telerobcamera.hpp"
#include "GPIO826.hpp"
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

int main(int argc, char **argv)
{
	double volt = -6.32;
	double volt2;
	uint channel = 5;
	GPIO826 Card1;

	// B826.SetDacOutput(&channel, &volt);
	// B826.GetDacOutput(&channel, &volt2);
	

	vec_voltage current; 

	current << 1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8;
// std::cout << "Here is the matrix a:\n" << a[0] << std::endl;

	Card1.AnalogWrite(&current);
	Card1.PrintError(); 
	Card1.GetDacOutput(&channel, &volt2);
	std::cout<<"Check this:     "<<volt2<<std::endl;
	current << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
// std::cout << "Here is the matrix a:\n" << a[0] << std::endl;

	Card1.AnalogWrite(&current);
	// B826.SetDacOutput(&channel, &volt);
	Card1.PrintError(); 
	Card1.GetDacOutput(&channel, &volt2);
	std::cout<<"Check this:     "<<volt2<<std::endl;
	uint dios[] = {         // Specify DIOs that are to be turned on:
		(1 << 7) + (1 << 13), //   DIOs 7 & 13 are in first 24-bit mask (DIOs 0-23),
		(1 << (38 - 24))      //   DIO 38 is in second 24-bit mask (DIOs 24-47).
	};
	std::cout<<dios[0]<<std::endl;
    return 0;
}



//////////////////////////////////