#ifndef _PATTERN_H_
#define _PATTERN_H_

#include "Color.h"



 /* A pattern is a matrix of 8x8 or 8x16 pixels which is composed of 2 bytes by lines, each bits in a byte represent
 * a column: 1st byte's bits being the msb and the 2nd byte's bits the lsb of a 2 bits value representing the pixel color.
 */
class Pattern{
  
  public:
  
  Pattern();
  ~Pattern();
  
  
  void    setPixLine(uint8_t y, uint16_t val);
  uint8_t getPixColor(uint8_t x, uint8_t y);                  
  void    flipX();                                  //Flip Pattern around X axis 
  void    flipY();                                  //Flip Pattern around Y axis 
  
  
  private:
    uint8_t      size;         //a Pattern is 8 lines in size in 8x8 mode and 16 lines in 8x16 mode (for sprites only) 
    Color  *matrix[8];         //maxtrix of 8 columns and size lines (allocated in GameboyMem methods)
};  


#endif