#include <iostream>
#include <vector>
#include "telerobcamera.hpp"
#include "telerob826.hpp"
#include <unistd.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/types.hpp>

#ifndef _LINUX
#include "826api.h"
#else
#include "826api.h"
#endif


// Helpful macros for DIOs
#define DIO(C)                  ((uint64)1 << (C))                          // convert dio channel number to uint64 bit mask
#define DIOMASK(N)              {(uint)(N) & 0xFFFFFF, (uint)((N) >> 24)}   // convert uint64 bit mask to uint[2] array
#define DIOSTATE(STATES,CHAN)   ((STATES[CHAN / 24] >> (CHAN % 24)) & 1)    // extract dio channel's boolean state from uint[2] array



#define X826(FUNC)   if ((errcode = FUNC) != S826_ERR_OK) { printf("\nERROR: %d\n", errcode); return errcode;}


int GetDacOutput(uint board, uint chan, double *volts)
{
  uint range, setpoint;
  int errcode = S826_DacRead(board, chan, &range, &setpoint, 0);     // Get DAC output range & setpoint.
  std::cout<<"Reading:"<<setpoint;
  std::cout<<range<<std::endl;
  switch (range) {                                                   // Convert binary setpoint to volts:
    case S826_DAC_SPAN_0_5:   *volts = setpoint            * ( 5.0 / 0xFFFF); break;  // 0 to +5V
    case S826_DAC_SPAN_0_10:  *volts = setpoint            * (10.0 / 0xFFFF); break;  // 0 to +10V
    case S826_DAC_SPAN_5_5:   *volts = (setpoint - 0x8000) * ( 5.0 / 0xFFFF); break;  // -5V to +5V
    case S826_DAC_SPAN_10_10: *volts = (setpoint - 32767.0) * (20.0 / 65535.0); break;  // -10V to +10V
  }
  return errcode;
}

#define PI  3.1415926535

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int SetDacOutput(uint board, uint chan, uint range, double volts)
{
  uint setpoint;
  std::cout<<range<<std::endl;
  switch (range) {  // conversion is based on dac output range:
    case S826_DAC_SPAN_0_5:   setpoint = (int)(volts * 0xFFFF /  5);          break; // 0 to +5V
    case S826_DAC_SPAN_0_10:  setpoint = (int)(volts * 0xFFFF / 10);          break; // 0 to +10V
    case S826_DAC_SPAN_5_5:   setpoint = (int)(volts * 0xFFFF / 10) + 0x8000; break; // -5V to +5V
    case S826_DAC_SPAN_10_10: setpoint = (int)(volts * 0xFFFF / 20) + 0x8000; break; // -10V to +10V
    default:                  return S826_ERR_VALUE;                                  // invalid range
  }
    std::cout<<"Writing:"<<setpoint;

  return S826_DacDataWrite(board, chan, setpoint, 0);  // program DAC output and return error code
  // return S826_DacRawWrite(board, chan, setpoint);  // program DAC output and return error code
}






////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main function.

int main(int argc, char **argv)
{
  // double volt = 10.0;
  // Telerob826 B826;
  // std::cout<<"here";
  // B826.SetDacOutput(0, &volt);
  // B826.GetDacOutput(0, &volt);












    uint board      = 0;                        // change this if you want to use other than board number 0
    int errcode     = S826_ERR_OK;  
    int boardflags  = S826_SystemOpen();        // open 826 driver and find all 826 boards
    if (argc > 1)
        board = atoi(argv[1]);

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
    } else  {
        // Execute the demo functions. Uncomment any functions you want to run.
        S826_DacRangeWrite(0, 0, S826_DAC_SPAN_10_10, 1);
        int errcode = SetDacOutput(0, 0, S826_DAC_SPAN_10_10, 10.00);

        // X826( DemoSinewaveGenerator(board)  );      // analog sinewave output
        // X826( TestDacRW(board) );
        double volts;
        if (GetDacOutput(0, 0, &volts) == S826_ERR_OK)
          printf("dac0 output is set to %f volts\n", volts);
        else
          printf("error reading dac0");
        errcode = SetDacOutput(0, 0, S826_DAC_SPAN_10_10, -10.00);

        // X826( DemoSinewaveGenerator(board)  );      // analog sinewave output
        // X826( TestDacRW(board) );
        if (GetDacOutput(0, 0, &volts) == S826_ERR_OK)
          printf("dac0 output is set to %f volts\n", volts);
        else
          printf("error reading dac0");


    }

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

    
#ifndef _LINUX  
    printf("\nKeypress to exit ...\n\n");
    while (!_kbhit());
    _getch();
#endif

    S826_SystemClose();
    return 0;
}



//////////////////////////////////