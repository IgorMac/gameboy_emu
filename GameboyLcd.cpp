#include <iostream>
#include <cstdint>
#include "GameboyLcd.h"
#include "pattern.h"

GameboyLcd::GameboyLcd(GameboyMem *gb_mem){
  gb_mem = gb_mem;
  for(int i = 0; i < NBLINES * NBCOLS; i++){
    screen[i / NBLINES][i % NBCOLS] = 0;
  }
}

/* fill the whole screen buffer of the GameBoy (256x256 pixels) */
void GameboyLcd::refresh(){
  uint8_t pat_id; 
  uint8_t sp_x, sp_y, sp_flags;       
  uint8_t lcdc = gb_mem->readData(LCDC_REG);                 //LCD Control register
  uint8_t scx  = gb_mem->readData( SCX_REG);                 //position x of the upper left corner of the real screen on the screen buffer
  uint8_t scy  = gb_mem->readData( SCY_REG);                 //position y of the upper left corner of the real screen on the screen buffer
  uint8_t wx   = gb_mem->readData(  WX_REG);                 //position x of the upper left corner of the Window on the real screen ((SCX,SCY) as origin)
  uint8_t wy   = gb_mem->readData(  WY_REG);                 //position y of the upper left corner of the Window on the real screen ((SCX,SCY) as origin)
  Pattern *p;             
  
  
  if (lcdc & BIT0_MASK){                                     //if BG & Window drawing is enabled
    for(int i = 0; i < 32; i++){                             //first drawing the background tile by tile 
      for(int j = 0; j < 32; ij++){ 
        pat_id = gb_mem->getBgPatternId(i, j);               //get the pattern id in the Pattern Data Table of tile (i,j)
        p = gb_mem->getBgWinPattern(pat_id);                 //get the pattern content 
        drawTile(8*i, 8*j, p);                               //drawing screen tile (i,j) at position (8*i,8*j) on the screen buffer with pattern content
        delete p;                                            //freeing memory allocated in getBgWinPattern method for Pattern
      }
    }                                                  
    if((lcdc & BIT5_MASK) && (wx <= 166) && (wy <= 143)){    //if Window display is enabled (and it's in the screen)
      for(int i = 0; 8*i+scx+wx-7 < 256; i++){               //Window upper left pixel is at (scx+wx-7, scy+wy) relative to the screen buffer 
        for(int j = 0; 8*j+scy+wy < 256; j++){               //draw tiles until you reach the end of the screen buffer
          pat_id = gb_mem->getWinPatternId(i, j);            //get the pattern id in the Pattern Data Table of tile (i,j)
          p = gb_mem->getBgWinPattern(pat_id)   ;            //get the pattern content 
          drawTile(8*i+scx+wx-7, 8*j+scy+wy, p) ;            //drawing screen tile (i,j) at position (8*i,8*j) on the screen buffer with pattern content
          delete p;                                          //freeing memory allocated in getBgWinPattern method for pattern
        }
      }
    }
  } 
  if (lcdc & BIT1_MASK){                                     //if sprite display is enabled
    /* TODO : priority and 10 max sprites by scan-line */
    for(int i = 0; i < MAX_SPRITES; i++){
      sp_x     = gb_mem->getSpriteX(i)           ;
      sp_y     = gb_mem->getSpriteY(i)           ;
      sp_flags = gb_mem->getSpriteFlags(i)       ;
      pat_id   = gb_mem->getSpritePatternId(i)   ;
      p        = gb_mem->getSpritePattern(pat_id);
      
      if(sp_flags & BIT5_MASK)
        p->flipX();
      if(sp_flags & BIT6_MASK)
        p->flipY();
      
      //TODO: palette and transparency 
      drawTile(scx+sp_x-8, scy+sp_y-16, p);
      delete p;
    }
  }
}

/* Drawing tile (x,y) with pattern content on the screen buffer.
 * The screen buffer is composed of 32x32 tiles of 8x8 pixels (indexed by pixels)
 * A pattern of 8x8 pixels is composed of 2 bytes by lines, each bits in a byte represent
 * a column: 1st byte's bits being the msb and the 2nd byte's bits the lsb
 * of a 2 bits value representing the pixel color.
 */
void GameboyLcd::drawTile(uint8_t x, uint8_t y, Pattern *p){
  uint8_t pix;
  
  for(int i = 0; i < p->8; i++)                  
    for(int j = 0; j < p->size; j++){         
      pix = p->getPixColor(i,j);
      setScreenPix(x+i, y+j, pix);   
    }      
}

/* setting the value of the pixel (x,y) with color value.
 * Do nothing if pixel is out of boundaries.
 */
void GameboyLcd::setScreenPix(uint8_t x, uint8_t y, Pixel color){
  if((x < 256) && (y < 256))
    screen[x][y] = color;       
}