cmake_minimum_required(VERSION 3.10)
project(MAgneticHaptic_Project_Tatile)
find_package( OpenCV REQUIRED )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -D_LINUX -no-pie -lm")
# link_directories("/home/ashkan/sensoray/driver")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(-DOSTYPE_linux)


add_executable(ROI_tracking live_ROI.cpp)
add_executable(Threaded_ROI live_ROI_MultiThreaded.cpp)


target_link_libraries(ROI_tracking PRIVATE  ${OpenCV_LIBS} m)
target_link_libraries(Threaded_ROI PRIVATE  ${OpenCV_LIBS} m)

