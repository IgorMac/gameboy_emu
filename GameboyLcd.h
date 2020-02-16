#ifndef _GAMEBOY_LCD_H_
#define _GAMEBOY_LCD_H_

#include <cstdint>
#include "GM_constants.h"
#include "GameboyMem.h"

class GameboyLcd{
  
  public:
  GameboyLcd(GameboyMem *gb_mem);
  
  void refresh();                                           //filling the whole screen buffer of the GameBoy (256x256 pixels)
  void drawTile(uint8_t x, uint8_t y, Pattern *p);        //filling screen tile (x,y) with pattern content
  void setScreenPix(uint8_t x, uint8_t y, Pixel color);   //setting the value of the pixel (x,y) with color value
  
  private:                            
  GameboyMem *gb_mem;                 //Adress space of the GameBoy

  Pixel screen[256][256];             //screen buffer of 256x256 pixels (32x32 tiles of 8x8 pix)
  
  
  
};

#endif