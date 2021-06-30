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
	// bool val = true;
	// for(int i = 0; i < NUMCOILS; i++) {
	// 	SetDioOutput(&COIL_INHIBIT_MAP[i], &val);	//Turns the amplifiers on--> Be sure the signals are zero before turning these on.	
	// }
	SysOn();
	PrintError();
};
void GPIO826::AnalogWrite(vec_voltage * coilvolts)
{
    _PCoilVolts = coilvolts;
	for(int i = 0; i < NUMCOILS; i++) {
		SetDacOutput(&COIL_OUT_MAP[i] , &((*coilvolts)(i)));
	}
};

void GPIO826::SysOn()
{
	
	for(int i = 0; i < NUMCOILS; i++) {
		SetDioOutput(&COIL_OUT_MAP[i] , &ON);
	}
};

void GPIO826::SysOff()
{
	
	for(int i = 0; i < NUMCOILS; i++) {
		SetDioOutput(&COIL_OUT_MAP[i] , &OFF);
	}
};

void GPIO826::SetCoilsCurrent(vec_current * coilcurrent)
{	
	Current2Volt(coilcurrent);
	std::cout << "been here3" << std::endl;
  	AnalogWrite(&_TempVolt);
};

void GPIO826::Current2Volt(vec_current * _coilcurrent)
{
	_TempVolt = (*_coilcurrent)*(1.0/3.0);	//	@TODO: This 1/3 needs to be adjusted
};

void GPIO826::GetCoilsCurrent(vec_current * coilcurrent)
{
	vec_voltage localVoltage(8,1);
	GetCoilsVoltage(localVoltage);
	std::cout<<"Local Volatage:"<<localVoltage<<std::endl;
	Volt2Current(&localVoltage, coilcurrent);
};

void GPIO826::GetCoilsVoltage(vec_voltage & coilvolts)
{
	for(int i = 0; i < NUMCOILS; i++) {
		GetDacOutput(&COIL_OUT_MAP[i] , &((coilvolts)(i)));
	}
};


void GPIO826::Volt2Current(vec_voltage * _coilvoltage, vec_current * _P_TempCurrent)
{
	(*_P_TempCurrent) = (*_coilvoltage)*(3.0);	//	@TODO: This 3 needs to be adjusted
};


void GPIO826::AnalogRead(vec_voltage * _volt)
{
	ReadAdcOutput(adcbuf, data); 
	for (int i = 0; i<16;i++){
		(*_volt)(i) = data[i];
	}

};

void GPIO826::GetCoilsTemperature(vec_temp_C * _temperature)
{
	ReadAdcOutput(adcbuf, data); 
	for (int i = 0; i<16;i++){
		// @TODO: a, b, c, d, e need actual values for thermocouple calibration curve
		(*_temperature)(i) = a*pow(data[i],4) + b*pow(data[i],3) + c*pow(data[i],2) + d*data[i] + e;
	}	
};	
