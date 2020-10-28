// This is the configuration of the system
// !!!!!! Be sure this is correct before running the system 
#ifndef SYSCONF_H
#define SYSCONF_H

const unsigned int  NUMCOILS = 8;	// How many coils we have!
const unsigned int  INHIBITCHANNEL = 1; //Channel number for inhibit for Amplifier
//  List of controll pins per coils : 0,1,2,3,4,5,6,7;	//The pin number with the correct order
const unsigned int  COIL_OUT_MAP[NUMCOILS] = {0,1,2,3,4,5,6,7};
//  List of temperature pins per coils : 0,1,2,3,4,5,6,7;	//The pin number with the correct order

const unsigned int  COIL_TEMP_MAP_A[NUMCOILS] = {0,2,4,6,8,10,12,14};
const unsigned int  COIL_TEMP_MAP_B[NUMCOILS] = {1,3,5,7,9,11,13,15};

#endif // SYSCONF_H
