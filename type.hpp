// Use this for type defs!
#ifndef TYPE_H
#define TYPE_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/QR> 
#define PI  3.1415926535
typedef double current;
typedef double voltage;
typedef double temperature;
const uint NumCoils = 8;
typedef Eigen::Matrix<current, NumCoils, 1> CURRENT_V;
typedef Eigen::Matrix<current, NumCoils, 1> VOLTAGE_V;
typedef Eigen::Matrix<current, 2*NumCoils, 1> TEMPERATURE_V; //Two sensors per coil




//////// 
// Stuf for sensoray 826 card
#define DIO(C)                  ((uint64)1 << (C))                          // convert dio channel number to uint64 bit mask
#define DIOMASK(N)              {(uint)(N) & 0xFFFFFF, (uint)((N) >> 24)}   // convert uint64 bit mask to uint[2] array
#define DIOSTATE(STATES,CHAN)   ((STATES[CHAN / 24] >> (CHAN % 24)) & 1)    // extract dio channel's boolean state from uint[2] array
#define X826(FUNC)   if ((errcode = FUNC) != S826_ERR_OK) { printf("\nERROR: %d\n", errcode); return errcode;}

#endif // TYPE_H