#include <iostream>
#include <vector>
#include "telerobcamera.hpp"
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

#ifndef _LINUX
#include "826api.h"
#else
#include "826api.h"
#endif

// #include <opencv2/viz.hpp>
// #include "print.h"  // This has the functions to print the camera and library version. TODO: move it somewhere else: 

// Helpful macros for DIOs
#define DIO(C)                  ((uint64)1 << (C))                          // convert dio channel number to uint64 bit mask
#define DIOMASK(N)              {(uint)(N) & 0xFFFFFF, (uint)((N) >> 24)}   // convert uint64 bit mask to uint[2] array
#define DIOSTATE(STATES,CHAN)   ((STATES[CHAN / 24] >> (CHAN % 24)) & 1)    // extract dio channel's boolean state from uint[2] array


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ERROR HANDLING
// These examples employ very simple error handling: if an error is detected, the example functions will immediately return an error code.
// This behavior may not be suitable for some real-world applications but it makes the code easier to read and understand. In a real
// application, it's likely that additional actions would need to be performed. The examples use the following X826 macro to handle API
// function errors; it calls an API function and stores the returned value in errcode, then returns immediately if an error was detected.

#define X826(FUNC)   if ((errcode = FUNC) != S826_ERR_OK) { printf("\nERROR: %d\n", errcode); return errcode;}

int errcode;

int main () {
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //Setup for the Cameras: --- DON'T MODIFIY ---
    FlyCapture2::Error error;      
	FlyCapture2::BusManager busMgr;

	unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        std::cout<< "something is wrong!\n"; 
        return -1;
    }
	std::cout << "Number of cameras detected: " << numCameras << std::endl;
	FlyCapture2::Camera *pCameras = new FlyCapture2::Camera[numCameras];
	for (unsigned int i = 0; i < numCameras; i++)
    {
        FlyCapture2::PGRGuid guid;
        error = busMgr.GetCameraFromIndex(i, &guid);
        if (error != FlyCapture2::PGRERROR_OK)
        {
            error.PrintErrorTrace();
            std::cout << "Press Enter to exit." << std::endl;
            delete[] pCameras;
            std::cin.ignore();
            return -1;
        }

        // Connect to a camera
        error = pCameras[i].Connect(&guid);
        if (error != FlyCapture2::PGRERROR_OK)
        {
            error.PrintErrorTrace();
            delete[] pCameras;
            std::cout << "Press Enter to exit." << std::endl;
            std::cin.ignore();
            return -1;
        }
        std::cout<<i;	
    }
	///////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    cv::FileStorage cal_file("../../../CalibrationMaster/build/cam_stereo.yml", cv::FileStorage::READ); // This is the location for the calibration file!
    cv::Mat cameraMatrix1, cameraMatrix2, Projection1, Projection2, Rect1, Rect2, distCoeffs1, distCoeffs2; 
    cal_file["Camera Matrix1"] >> cameraMatrix1;
    cal_file["Camera Matrix2"] >> cameraMatrix2;
    cal_file["Projection Matrix1"] >> Projection1;
    cal_file["Projection Matrix2"] >> Projection2;
    cal_file["Rectification Matrix1"] >> Rect1;
    cal_file["Rectification Matrix2"] >> Rect2;
    cal_file["Dist Coeffs1"] >> distCoeffs1;
    cal_file["Dist Coeffs2"] >> distCoeffs2;
// std::cout<< Rect2;
    cv::Mat pnts3D(1,1,CV_64F);
    cv::Mat point3D(1,1,CV_64F);

    //////////////////////////////////////////////
	TelerobCamera cam1(pCameras[0]);
	TelerobCamera cam2(pCameras[1]);

    cv::Point Pt1, Pt2;
	char key = 0;
    while(key != 'q')
    {
		cam1.UpdateFrame();
		cam2.UpdateFrame();
        cam1.FilterCurrentFrame();
        cam2.FilterCurrentFrame();
        // std::cout <<"This is one frame : ";
        Pt1 = cam1.GetTargetPose();
        Pt2 = cam2.GetTargetPose();
        cv::Mat Taret1 = (cv::Mat_<double>(2,1) << Pt1.x, Pt1.y);
        cv::Mat Taret2 = (cv::Mat_<double>(2,1) << Pt2.x, Pt2.y);
        

		// cv::triangulatePoints(Projection1,Projection2,Taret1,Taret2,pnts3D);
  //       point3D = pnts3D;
  //       point3D.at<double>(0, 0) = point3D.at<double>(0, 0)/pnts3D.at<double>(3, 0);
  //       point3D.at<double>(1, 0) = point3D.at<double>(1, 0)/pnts3D.at<double>(3, 0);
  //       point3D.at<double>(2, 0) = point3D.at<double>(2, 0)/pnts3D.at<double>(3, 0);
  //       std::cout << point3D<<std::endl;

  //       std::cout << point3D<<std::endl;
        cv::imshow("image1", cam1.GetCurrentFrame()); 
        // cv::imshow("Object Detection1", cam1.GetCurrentFilteredFrame());
        // cv::imshow("Object Detection2", cam2.GetCurrentFilteredFrame());
        cv::imshow("image2", cam2.GetCurrentFrame());
        X826( S826_DacDataWrite(0, 0, 32767, 0));
        key = cv::waitKey(30);        
    }
    X826( S826_DacDataWrite(0, 0, 32767, 0));
    return 0;
}




//////////////////////////////////

