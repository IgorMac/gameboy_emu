#ifndef _GAMEBOY_MEM_H_
#define _GAMEBOY_MEM_H_

#include <cstdint>
#include "GM_constants.h"
#include "pattern.h"

/*Adress space of the GameBoy*/
class GameboyMem{
  public:
  
    GameboyMem();
    /* read & write data */
    void     writeData   (uint16_t addr,  uint8_t val);      //Write 8-bits data in memory
    void     writeData16b(uint16_t addr, uint16_t val);      //Write 16-bits data in memory (little-endian mode)
    uint8_t  readData    (uint16_t addr)              ;      //Read 8-bits data in memory
    uint16_t readData16b (uint16_t addr)              ;      //Read 16-bits data in memory (little-endian mode)
    
    /* VRAM structures */
    uint8_t  getBgPatternId (uint8_t x, uint8_t y);
    uint8_t  getWinPatternId(uint8_t x, uint8_t y);
    Pattern *getBgWinPattern (uint8_t id);
    Pattern *getSpritePattern(uint8_t id);
    uint8_t  getSpriteX(uint8_t id);
    uint8_t  getSpriteY(uint8_t id);
    uint8_t  getSpritePatternId(uint8_t id);
    uint8_t  getSpriteFlags(uint8_t id);
    
    
  private:
    uint8_t mem[MEMORY_SIZE];      //64K 
};


#endif