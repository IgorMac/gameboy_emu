#include <cstdint>
#include <iostream>
#include "GameboyMem.h"
#include "GM_constants.h"
#include "pattern.h"

GameboyMem::GameboyMem(){
  for(int i = 0; i < MEMORY_SIZE; i++){
    mem[i] = 0;
  }
}

/* Read 8-bits data in memory */
void GameboyMem::writeData(uint16_t addr, uint8_t val){
  
  /* memory at 0xC000 to 0xDE00 is echo of memory at 0xE000 to 0xFE00 */
  if((addr >= 0xC000) && (addr <= 0xDE00))           
    mem[addr + 0x2000] = val;
  else if((addr >= 0xE000) && (addr <= 0xFE00))       
    mem[addr - 0x2000] = val;
    
  mem[addr] = val;         
}
/* Write 8-bits data in memory */
uint8_t GameboyMem::readData(uint16_t addr){
  return mem[addr];
}

/* Write 16-bits data in memory (little-endian mode) */
void GameboyMem::writeData16b(uint16_t addr, uint16_t val){
  writeData(addr  , val);            //write LSB first in first memory address
  writeData(addr+1, (val >> 8));     //write MSB in next memory adress
}
/* Read 16-bits data in memory (little-endian mode) */
uint16_t GameboyMem::readData16b(uint16_t addr){
  uint16_t tmpl, tmpm;
  tmpl = readData(addr);             //get LSB first
  tmpm = readData(addr+1);           //get MSB next
  return (tmpm << 8) | tmpl;     
}


/* Return the pattern id in the Tile Pattern Table for the Tile (x,y) in
 * the Background Tile Map
 */
uint8_t GameboyMem::getBgPatternId(uint8_t x, uint8_t y){
  uint16_t addr;
  
  if(mem[LCDC_REG] & BIT3_MASK)   //2 starting adress is possible
    addr = 0x9C00;
  else                    
    addr = 0x9800;
    
  return mem[addr + NBLINES*y + x];
}


/* Return the pattern id in the Tile Pattern Table for the Tile (x,y) in
 * the Window Tile Map
 */
uint8_t GameboyMem::getWinPatternId(uint8_t x, uint8_t y){
  uint16_t addr;
  
  if(mem[LCDC_REG] & BIT6_MASK)   //2 starting adress is possible
    addr = 0x9C00;
  else                    
    addr = 0x9800;
    
  return mem[addr + NBLINES*y + x];
}



Pattern *GameboyMem::getBgWinPattern(uint8_t id){
  uint16_t addr;
  uint16_t color;
  Pattern *p;
  
  p = new Pattern(8);                                   //Bg & Window Tiles are always 8x8 pixels (2 bytes by line = 16)

  if(mem[LCDC_REG] & BIT4_MASK){                        //2 starting adresses is possible
    addr     = 0x9000;
    for(int i = 0; i < p->pat_size; i++){
      color = readData16b(addr + (int8_t)id*16+i);             //in this configuration id is treated as signed number  
      p->setPixColor(i, color);  
    }
  }else{                    
    addr     = 0x8000;
    for(int i = 0; i < p->pat_size; i++){ 
      color = readData16b(addr + id*16+i);             //in this configuration id is treated as unsigned number 
      p->setPixColor(i, color); 
    }
  } 
  
  return p;        
}


Pattern *GameboyMem::getSpritePattern(uint8_t id){       
  const uint16_t BASE = 0x8000;                        
  Pattern *p;
  
  if(mem[LCDC_REG] & BIT2_MASK){                 //8x16 sprite mode
    p = new Pattern(16);                         //2 bytes by lines = 32 bytes
    id = id & 0xFE;                              //ignore bit 0
  }else                                          //8x8 sprite mode
    p = new Pattern(8);                          //2 bytes by lines = 16 bytes

 
  for(int i = 0; i < p->pat_size; i++)           
    p->setByteValue(i, mem[BASE + id*16+i]);              
  
  
  return p;                                      
}



uint8_t getSpriteX        (uint8_t id){return mem[OAM_SPACE+id*4+0];}
uint8_t getSpriteY        (uint8_t id){return mem[OAM_SPACE+id*4+1];}
uint8_t getSpritePatternId(uint8_t id){return mem[OAM_SPACE+id*4+2];}
uint8_t getSpriteFlags    (uint8_t id){return mem[OAM_SPACE+id*4+3];}
    



