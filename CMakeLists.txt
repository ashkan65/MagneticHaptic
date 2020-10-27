cmake_minimum_required(VERSION 3.10)
project(MAgneticHaptic_Project)
find_package( OpenCV REQUIRED )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")
link_directories("/home/ashkan/sensoray/driver")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(-DOSTYPE_linux)


add_library(
	telerobcamera
	telerobcamera.cpp
	telerobcamera.hpp
	)

add_executable(Haptic_demo main.cpp)

INCLUDE_DIRECTORIES("/home/ashkan/sensoray/driver")
INCLUDE_DIRECTORIES(/home/ashkan/eigen, /usr/local/lib, /usr/include/flycapture)

target_link_libraries(telerobcamera PRIVATE  "/usr/lib/libflycapture.so" ${OpenCV_LIBS})
# target_link_libraries(pigi PRIVATE "/home/ashkan/Desktop/naghi_C/build/libghazam.a" )
target_link_libraries(Haptic_demo PRIVATE telerobcamera "/usr/lib/libflycapture.so"  ${OpenCV_LIBS} "/home/ashkan/sensoray/demo/lib826_64.a" m)

# target_link_libraries(SystemRun PRIVATE omnimagnet type "/usr/local/lib/libcomedi.a")

# add_executable(CameraCalibration camera_calibration.cpp)

# target_link_libraries(CameraCalibration "/usr/lib/libSpinnaker.so"  ${OpenCV_LIBS})
