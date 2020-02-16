#ifndef _GAMEBOY_H_
#define _GAMEBOY_H_

#include <cstdint>
#include "GameboyMem.h"
#include "GameboyLcd.h"
#include "GameboyCpu.h"

class Gameboy{
  public:
  Gameboy();
  
  private:
  GameboyMem gb_mem;
  GameboyCpu gb_cpu(&gb_mem);
  GameboyLcd gb_lcd(&gb_mem);
}

#endif