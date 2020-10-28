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
    S826_DacRangeWrite(0, 0, S826_DAC_SPAN_10_10, 1);
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The Dist for the board. 

Simple826::~Simple826(){
    S826_SystemClose();
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AnalogWrite. 

void Simple826::SetDacOutput(uint *chan, double *volts){ // DAC for one channel ----> DOTO add a function for vector update!
    errcode = S826_DacDataWrite(board, *chan, (int)(*volts * 0xFFFF / 20) + 0x8000, 0);  // program DAC output and return error code
};







void Simple826::GetDacOutput(uint *chan, double *volts){   //Reads the current voltage for a given channel.
    uint range, setpoint;
    errcode = S826_DacRead(board, *chan, &range, &setpoint, 0);
    *volts = (setpoint - 32767.0) * (20.0 / 65535.0);
    PrintError(); //Incase of error!!!
}; 


int Simple826::GetError(){ //return error code 
    return errcode; 
};

void Simple826::PrintError(){ //return error code 
    switch (errcode)
    {
        case S826_ERR_OK:           break;
        case S826_ERR_BOARD:        printf("Illegal board number"); break;
        case S826_ERR_VALUE:        printf("Illegal argument"); break;
        case S826_ERR_NOTREADY:     printf("Device not ready or timeout"); break;
        case S826_ERR_CANCELLED:    printf("Wait cancelled"); break;
        case S826_ERR_DRIVER:       printf("Driver call failed"); break;
        case S826_ERR_MISSEDTRIG:   printf("Missed adc trigger"); break;
        case S826_ERR_DUPADDR:      printf("Two boards have same number"); break;S826_SafeWrenWrite(board, 0x02);
        case S826_ERR_BOARDCLOSED:  printf("Board not open"); break;
        case S826_ERR_CREATEMUTEX:  printf("Can't create mutex"); break;
        case S826_ERR_MEMORYMAP:    printf("Can't map board"); break;
        default:                    printf("Unknown error"); break;
    }
};
