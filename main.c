#include "include.h"

void main()
{
  WDT_A_hold(WDT_A_BASE);
  _DINT();
  Hardware_Init();
  _EINT();
  
  while(1)
  {
     PollingKernel();
  }
}
