#include <iostream>
#include "simple826.hpp"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The const for the board. 
Simple826::Simple826(){
    board      = 0;                        // change this if you want to use other than board number 0
    errcode     = S826_ERR_OK;  
    boardflags  = S826_SystemOpen();        // open 826 driver and find all 826 boards
    

    if (boardflags < 0)
        errcode = boardflags;                       // problem during open
    else if ((boardflags & (1 << board)) == 0) {
        int i;
        printf("TARGET BOARD of index %d NOT FOUND\n",board);         // driver didn't find board you want to use
        for (i = 0; i < 8; i++) {
            if (boardflags & (1 << i)) {
                printf("board %d detected. try \"./s826demo %d\"\n", i, i);
            }
        }
    }
    PrintError();
    


    // S826_DacRangeWrite(0, 0, S826_DAC_SPAN_10_10, 1);
    for (uint aout = 0; aout < S826_NUM_DAC; aout++) {         // Program safemode analog output condition:
        S826_DacRangeWrite(0, aout, S826_DAC_SPAN_10_10, 0);  //   output range
        S826_DacDataWrite(0, aout, 0, 0);                   //   output voltage
    }
    PrintError();
    // for (uint ain = 0; ain < 2*S826_NUM_DAC; ain++)                              //This needs more work!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //     S826_AdcSlotConfigWrite(0, uint slot, ain, // analog input channel number
    //     uint tsettle, // settling time in microseconds
    //     uint range // input range code
    //     );
    // }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The dist for the board. 

Simple826::~Simple826(){
    for (uint aout = 0; aout < S826_NUM_DAC; aout++) {         // Program safemode analog output condition:
        S826_DacDataWrite(0, aout, 0, 0);                   //   output voltage
    }
    uint mask = 0;
    S826_DioOutputWrite(0,&mask, 0); // Turning digital pins off. --> This turns the HIGHVOLTAGE aplifiers off. Always use this before exit!!!!!!!
    S826_SystemClose();
};



int Simple826::GetError(){ //return error code   0: No error, 1: there is and error 
    return errcode; 
};

void Simple826::PrintError(){ //return error code 
    switch (errcode)
    {
        case S826_ERR_OK:           break;
        case S826_ERR_BOARD:        std::cout<<"Illegal board number"<<std::endl; break;
        case S826_ERR_VALUE:        std::cout<<"Illegal argument"<<std::endl; break;
        case S826_ERR_NOTREADY:     std::cout<<"Device not ready or timeout"<<std::endl; break;
        case S826_ERR_CANCELLED:    std::cout<<"Wait cancelled"<<std::endl; break;
        case S826_ERR_DRIVER:       std::cout<<"Driver call failed"<<std::endl; break;
        case S826_ERR_MISSEDTRIG:   std::cout<<"Missed adc trigger"<<std::endl; break;
        case S826_ERR_DUPADDR:      std::cout<<"Two boards have same number"<<std::endl; break;S826_SafeWrenWrite(board, 0x02);
        case S826_ERR_BOARDCLOSED:  std::cout<<"Board not open"<<std::endl; break;
        case S826_ERR_CREATEMUTEX:  std::cout<<"Can't create mutex"<<std::endl; break;
        case S826_ERR_MEMORYMAP:    std::cout<<"Can't map board"<<std::endl; break;
        default:                    std::cout<<"Unknown error"<<std::endl; break;
    }
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AnalogWrite. 

void Simple826::SetDacOutput(uint *chan, double *volts){ // DAC for one channel ----> DOTO add a function for vector update!
    // std::cout<<"Check this guy: "<< *volts<<std::endl;
    errcode = S826_DacDataWrite(board, *chan, (int)(*volts * 0xFFFF / 20) + 0x8000, 0);  // program DAC output and return error code
    // errcode = S826_DacDataWrite(board, *chan, 0xFFFF, 0);  // program DAC output and return error code

};

void Simple826::GetDacOutput(uint *chan, double *volts){   //Reads the current voltage for a given channel.
    uint range, setpoint;
    errcode = S826_DacRead(board, *chan, &range, &setpoint, 0);
    *volts = (setpoint - 32767.0) * (20.0 / 65535.0);
    PrintError(); //Incase of error!!!
}; 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DigitalWrite. 
// (condition) ? (if_true) : (if_false) 
void Simple826::SetDioOutput(uint *chan, bool *val){        //chan->channel number  val->voltage ==============DIGITALOUT============
    uint mask[] = {uint(pow(2,*chan)), 0};
    *val ? S826_DioOutputWrite(0, mask, 2) : S826_DioOutputWrite(0, mask, 1);
};   

void Simple826::ReadAdcOutput(int* adcbuf, double *data){ // Abalog input read for every channel from 1 to 16
    errcode = S826_AdcRead(board, adcbuf, NULL, &slotlist, 1000) ; // read adc data from 16 slots
        // Converting buffer value to voltage in each slot (data*10volt/2^8) setting: -10V to 10V, -2^8 to 2^8 bits : 
        for (int slot = 0; slot < 16; slot++){ 
            data[slot] = (short)( adcbuf[slot] & 0xFFFF );
            data[slot] = (double)(data[slot]*10)/(32768);
        };     


  }