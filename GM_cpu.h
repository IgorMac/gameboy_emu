#include <iostream>
#include "cpu_def.h"


class GM_cpu{
    
  public:
    GM_cpu();
    
  private:
    int8 gregs[8]; //general registers
    int8 fregs;    //flag register
    int16 sp;      //stack pointer
    int16 pc;      //program counter
};
