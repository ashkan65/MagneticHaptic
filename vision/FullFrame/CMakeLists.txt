cmake_minimum_required(VERSION 3.10)
project(MAgneticHaptic_Project_Tatile)
set (CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )
find_package (Threads)





add_executable(full_frame cam_full_frame_live.cpp)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -D_LINUX -no-pie -lm")
# link_directories("/home/ashkan/sensoray/driver")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(-DOSTYPE_linux)

target_link_libraries(full_frame PRIVATE ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} m)