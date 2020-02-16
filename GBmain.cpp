#include <iostream>
#include "GameboyCpu.h"
#include "GM_constants.h"

using namespace std;

int main(){
  GameboyMem gb_mem;
  GameboyCpu gb_cpu(&gb_mem);
  
  while(gb_cpu.run());
  
  return 0;
}