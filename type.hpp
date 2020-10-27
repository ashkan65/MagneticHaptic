// Use this for type defs!
#include <vector>
#define PI  3.1415926535
typedef double current;
typedef double voltage;
typedef double temperature;
//////// 
// Stuf for sensoray 826 card
#define DIO(C)                  ((uint64)1 << (C))                          // convert dio channel number to uint64 bit mask
#define DIOMASK(N)              {(uint)(N) & 0xFFFFFF, (uint)((N) >> 24)}   // convert uint64 bit mask to uint[2] array
#define DIOSTATE(STATES,CHAN)   ((STATES[CHAN / 24] >> (CHAN % 24)) & 1)    // extract dio channel's boolean state from uint[2] array
#define X826(FUNC)   if ((errcode = FUNC) != S826_ERR_OK) { printf("\nERROR: %d\n", errcode); return errcode;}