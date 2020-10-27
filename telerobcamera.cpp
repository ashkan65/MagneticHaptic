#include <iostream>
#include "telerobcamera.hpp"

TelerobCamera::TelerobCamera(){
    // Will add latter
};

TelerobCamera::TelerobCamera(FlyCapture2::Camera& pcams){
    pCam = &pcams;
    int I = Initial();
};

TelerobCamera::~TelerobCamera(){
        pCam->StopCapture();
        pCam->Disconnect();
        // delete pCam;
};

void TelerobCamera::PrintCameraInfo(FlyCapture2::CameraInfo *pCamInfo)
{
    std::cout << std::endl;
    std::cout << "*** CAMERA INFORMATION ***" << std::endl;
    std::cout << "Serial number - " << pCamInfo->serialNumber << std::endl;
    std::cout << "Camera model - " << pCamInfo->modelName << std::endl;
    std::cout << "Camera vendor - " << pCamInfo->vendorName << std::endl;
    std::cout << "Sensor - " << pCamInfo->sensorInfo << std::endl;
    std::cout << "Resolution - " << pCamInfo->sensorResolution << std::endl;
    std::cout << "Firmware version - " << pCamInfo->firmwareVersion << std::endl;
    std::cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << std::endl
         << std::endl;
};

int TelerobCamera::Initial(){
    // Get the camera information
    error = pCam->GetCameraInfo(&camInfo);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        delete pCam;
        std::cout << "Press Enter to exit." << std::endl;
        std::cin.ignore();
        return -1;
    }

    PrintCameraInfo(&camInfo);
    // Turn trigger mode off
    trigMode.onOff = false;
    error = pCam->SetTriggerMode(&trigMode);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        delete pCam;
        std::cout << "Press Enter to exit." << std::endl;
        std::cin.ignore();
        return -1;
    }
    // Turn Timestamp on
    imageInfo.timestamp.onOff = true;
    error = pCam->SetEmbeddedImageInfo(&imageInfo);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        delete pCam;
        std::cout << "Press Enter to exit." << std::endl;
        std::cin.ignore();
        return -1;
    }
    // Start streaming on camera
    error = pCam->StartCapture();

    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        delete pCam;
        std::cout << "Press Enter to exit." << std::endl;
        std::cin.ignore();
        return -1;
    }
	return 0;
};


void TelerobCamera::UpdateFrame(){
    error = pCam->RetrieveBuffer( &FCImage_raw );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
    	error.PrintErrorTrace();
        std::cout << "capture error" << std::endl;
    }
}; //Updates the frame


cv::Mat TelerobCamera::GetCurrentFrame(){
	FCImage_raw.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &FCImage_RGB );
	// convert to OpenCV Mat
	unsigned int rowBytes = (double)FCImage_RGB.GetReceivedDataSize()/(double)FCImage_RGB.GetRows();       
	// cv::Mat CVImage_RGB;
	CVImage_RGB = cv::Mat( FCImage_RGB.GetRows(), FCImage_RGB.GetCols(), CV_8UC3, FCImage_RGB.GetData(),rowBytes);
	return CVImage_RGB;
};

cv::Mat TelerobCamera::GetRGBFrame(){
    return CVImage_RGB;
};

cv::Mat TelerobCamera::GetCurrentFilteredFrame(){
    FilterCurrentFrame();
    return CVImage_RGB;
};	
void TelerobCamera::SaveCurrentFrame( std::string fileName){

}; 

void TelerobCamera::FilterCurrentFrame(){
    // This is the covertion to opencv
    FCImage_raw.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &FCImage_RGB );
    // convert to OpenCV Mat
    unsigned int rowBytes = (double)FCImage_RGB.GetReceivedDataSize()/(double)FCImage_RGB.GetRows();       
    // cv::Mat CVImage_RGB;
    CVImage_RGB = cv::Mat( FCImage_RGB.GetRows(), FCImage_RGB.GetCols(), CV_8UC3, FCImage_RGB.GetData(),rowBytes);
    // This is the filter part
    cv::cvtColor( CVImage_RGB, CVImage_Filtered, cv::COLOR_BGR2HSV);
    cv::inRange( CVImage_Filtered, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), CVImage_Filtered);
    cv::erode( CVImage_Filtered, CVImage_Filtered, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate( CVImage_Filtered, CVImage_Filtered, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate( CVImage_Filtered, CVImage_Filtered, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11)) );
    cv::erode( CVImage_Filtered, CVImage_Filtered, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11)) );
    cv::findContours( CVImage_Filtered, contour, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    std::vector<cv::Moments> mu(contour.size() );
    std::vector<cv::Point2f> mc( contour.size() );
    for (int i=0; i<contour.size();i++)
    {
        mu[i] = cv::moments( contour[i], false );
        mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
        cv::circle(CVImage_RGB, mc[i], 4, cv::Scalar(0,255,0), -1, 8, 0 );
        Target = mc[i];   
    }

};

cv::Point TelerobCamera::GetTargetPose(){
    return Target;
};

