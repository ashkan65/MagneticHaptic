#include <iostream>
#include "simple826.hpp"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The constructor for the board. 
Simple826::Simple826(){
    board       = 0;                        // change this if you want to use other than board number 0
    errcode     = S826_ERR_OK;  
    boardflags  = S826_SystemOpen();        // open 826 driver and find all 826 boards
    

    if (boardflags < 0)
        errcode = boardflags;                       // problem during open
    else if ((boardflags & (1 << board)) == 0) {
        int i;
        printf("TARGET BOARD of index %d NOT FOUND\n",board);         // driver didn't find the board you want to use
        for (i = 0; i < 8; i++) {
            if (boardflags & (1 << i)) {
                printf("board %d detected. try \"./s826demo %d\"\n", i, i);
            }
        }
    }
    PrintError();
    


    // S826_DacRangeWrite(0, 0, S826_DAC_SPAN_10_10, 0);
    for (uint aout = 0; aout < S826_NUM_DAC; aout++) {          // Program safemode analog output condition:
        S826_DacRangeWrite(0, aout, S826_DAC_SPAN_10_10, 0);    // Output range from -10 to +10 Volts
        S826_DacDataWrite(0, aout, 0, 0);                       // Outputs voltage
    }
    PrintError();



    uint tsettle = 1500;  // settling time in microseconds. Can range from 0 (inactive slot) to approximately 335544 microseconds in one-microsecond increments
    uint range = 0;         // input range code. 0 means gain of 1 for analog input range of -10 to +10 volts
    uint slot;
    //uint slotlist;
    uint mode = 0;          // 0 = write, 1 = clear bits, 2 = set bits (see “Atomic read-modify-write”)    
    for (uint ain = 0; ain < S826_NUM_ADC; ain++) {                                //This needs more work!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        slot = ain;                                                              // Since we are using every channel we will assign each timeslot to each channel.
        S826_AdcSlotConfigWrite(board, slot, ain, tsettle, range);               // Register the three attributes (analog channel, settling time, and gain by setting the range) for each slot.
    }
    S826_AdcSlotlistWrite(board, slotlist, mode);             // Register which slots (0-15) are active or inactive  

    //     S826_AdcTrigModeWrite(uint board, uint trigmode);  // May not be needed. Lets you pick if you want triggered or continuous mode. Trigger mode will wait for trigger, continuous will not.
    S826_AdcTrigModeWrite(board, 0);  // select continuous (untriggered) mode
    S826_AdcEnableWrite(board, 1);     

     PrintError();
     
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The destructor for the board. 

Simple826::~Simple826(){
    for (uint aout = 0; aout < S826_NUM_DAC; aout++) {          // Program safemode analog output condition:
        S826_DacDataWrite(0, aout, 0, 0);                       // output voltage
    }
    uint mask = 0;
    S826_DioOutputWrite(0,&mask, 0); // Turning digital pins off. --> This turns the HIGHVOLTAGE amplifiers off. Always use this before exit!!!!!!!
    S826_SystemClose();
};



int Simple826::GetError(){      //return error code   0 means there is no error, 1 means there is an error 
    return errcode; 
};

void Simple826::PrintError(){   //return error code 
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

void Simple826::ReadAdcOutput(int* adcbuf, double *data){ // Analog input read for every channel from 0 to 15
    errcode = S826_AdcRead(board, adcbuf, NULL, &slotlist, 1000) ; // read adc data from 16 slots
        // Converting buffer value to voltage in each slot (data*10volt/2^8) setting: -10V to 10V, -2^8 to 2^8 bits : 
        for (int slot = 0; slot < 16; slot++){ 
            std::cout<<adcbuf[slot]<<std::endl;
            data[slot] = (short)( adcbuf[slot] & 0xFFFF );
            data[slot] = (double)(data[slot]*10)/(32768);
        };     
  }