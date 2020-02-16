#include <cstdint>
#include <Pattern.h>



Pattern::Pattern(uint8_t s){
  size = s;
  for(int i = 0; i < 8; i++)
    matrix[i] = new Pixel[s];
}


Pattern::~Pattern(){
  for(int i = 0; i < 8; i++)
    delete matrix[i];
}


void    Pattern::setPixLine(uint8_t y, uint16_t val){
  uint8_t pix_msb, pix_lsb;   

  for(int i = 0; i < 8; i++){                 
    pix_lsb = (val  >> (8-i))  & 1;               //1st byte of the line y for the lsb
    pix_msb = (val  >> (16-i)) & 1;               //2nd byte of the line y for the msb
    matrix[i][y] = (Pixel)((pix_msb << 1) | pix_lsb);
  }
}

Pixel Pattern::getPix(uint8_t i, uint8_t j){
  return matrix[i][j];
}

/* Flip Pattern around Y axis */
void Pattern::flipY(){
  Pixel tmp;
  
  for(int j = 0; j < size; j++){
    for(int i = 0; i < 4; i++){
      tmp = matrix[i][j];
      matrix[i][j] = matrix[7-i][j];
      matrix[7-i][j] = tmp;
    }
  }
}
/* Flip Pattern around X axis */
void Pattern::flipX(){
  uint8_t  tmp;
  
  for(int i = 0; i < 8; i++){
    for(int j= 0; j < size/2; j++){
      tmp = matrix[i][j];
      matrix[i][j] = matrix[i][size-1-j];
      matrix[i][size-1-j] = tmp;
    }
  } 
}