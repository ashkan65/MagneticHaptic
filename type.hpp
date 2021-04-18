// Use this for type defs!
#ifndef TYPE_H
#define TYPE_H

// #include "system_config.hpp"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/QR> 
#define PI   3.1415926535
#define PI_2 1.570796327

typedef double current;
typedef double voltage;
typedef double temperature;
typedef Eigen::VectorXd vec_wrench;
typedef Eigen::VectorXd vec_current;
typedef Eigen::VectorXd vec_voltage;
typedef Eigen::VectorXd vec_temp_C; //Two sensors per coil ^\circ {C}
typedef Eigen::MatrixXd Mat_actuation; // 




//////// 
// Stuf for sensoray 826 card
#define DIO(C)                  ((uint64)1 << (C))                          // convert dio channel number to uint64 bit mask
#define DIOMASK(N)              {(uint)(N) & 0xFFFFFF, (uint)((N) >> 24)}   // convert uint64 bit mask to uint[2] array
#define DIOSTATE(STATES,CHAN)   ((STATES[CHAN / 24] >> (CHAN % 24)) & 1)    // extract dio channel's boolean state from uint[2] array
#define X826(FUNC)   if ((errcode = FUNC) != S826_ERR_OK) { printf("\nERROR: %d\n", errcode); return errcode;}

#endif // TYPE_H
