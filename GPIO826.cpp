#include "GPIO826.hpp"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The const for the board. 
    // @todo: Finish the code here 
    // @body: The interface needs more work!

void GPIO826::Init()
{
	*_PCoilVolts<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	AnalogWrite(_PCoilVolts);	// Turns the signal channels off --> be sure there is not HIGHVOLT output
	bool val = true;
	for(int i = 0; i < NUMCOILS; i++) {
		SetDioOutput(&COIL_INHIBIT_MAP[i], &val);	//Turns the amplifiers on--> Be sure the signals are zero before turning these on.	
	}
	PrintError();
};
void GPIO826::AnalogWrite(vec_voltage * coilvolts)
{
    _PCoilVolts = coilvolts;
	for(int i = 0; i < NUMCOILS; i++) {
		SetDacOutput(&COIL_OUT_MAP[i] , &((*coilvolts)(i)));
	}
};

