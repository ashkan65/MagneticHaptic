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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main function.

int main(int argc, char **argv)
{
  double volt = -6.32;
  double volt2;
  uint b = 0;
  Telerob826 B826;
  B826.SetDacOutput(&b, &volt);
  B826.GetDacOutput(&b, &volt2);
  std::cout<<"CHeck this:     "<<volt2<<std::endl;

    return 0;
}



//////////////////////////////////