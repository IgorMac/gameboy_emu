#include <iostream>
#include "GameboyMem.h"
#include "GameboyCpu.h"
#include "GameboyLcd.h"
#include "GM_constants.h"

using namespace std;

int main(){
  GameboyMem gb_mem;
  GameboyCpu gb_cpu(&gb_mem);
  GameboyLcd gb_lcd(&gb_mem);
  
  uint8_t id = 0xF5;
  uint16_t addr = 0x9000;
  
  
  /*Test Register set/get methods*/
  cout << "SP before setSP() : " << gb_cpu.getSP() << endl;
  gb_cpu.setSP(0xF8);
  cout << "SP after  setSP() : " << gb_cpu.getSP() << endl;
  gb_cpu.setReg(B, -2);
  cout << "B after  setReg() : " << hex << unsigned(gb_cpu.getReg(B)) << endl;
  
  /*Test Flag set/get methods*/
  cout << hex << unsigned(gb_cpu.getReg(F)) << " " << unsigned(gb_cpu.getCflag()) << endl;
  gb_cpu.setCflag(1);
  gb_cpu.setZflag(1);
  cout << "0x" << hex << unsigned(gb_cpu.getReg(F)) << " " << gb_cpu.getCflag() << " " << gb_cpu.getZflag() << endl;
  gb_cpu.setZflag(0);
  cout << "0x" << hex << unsigned(gb_cpu.getReg(F)) << " " << gb_cpu.getCflag() << " " << gb_cpu.getZflag() << endl;
  
  /*Test set/get double 8-bits registers functions*/
  gb_cpu.setRegPair(HL, 0xC8A3);
  cout << "H after  setHL(0xC8A3) : 0x" << hex << unsigned(gb_cpu.getReg(H)) << endl;
  cout << "L after  setHL(0xC8A3) : 0x" << hex << unsigned(gb_cpu.getReg(L)) << endl;
  
  /*Test isCarry functions*/
  cout << GameboyCpu::isCarry(0xFF, 1, 0) << " " << GameboyCpu::isCarry(0xFE, 1, 0) << " " << GameboyCpu::isCarry(0xCC, 0x24, 0) << endl; 
  cout << GameboyCpu::isHalfCarry(0xFF, 1, 0) << " " << GameboyCpu::isHalfCarry(0xFE, 1, 0) << " " << GameboyCpu::isHalfCarry(0xCC, 0x24, 0) << endl; 
  cout << GameboyCpu::isHalfCarry(0xFF, 1, 0) << " " << GameboyCpu::isHalfCarry(0xFE, 1, 0) << " " << GameboyCpu::isHalfCarry(0xCC, 0x24, 0) << endl; 
  
  /*Test CP instruction*/
  gb_cpu.setReg(A, 0x54); gb_cpu.setReg(C, 0xA4);
  gb_cpu.execInstr(CP_REG_OP | C);   //compare C to A
  cout << "A:0x" << unsigned(gb_cpu.getReg(A)) << " C:0x" << unsigned(gb_cpu.getReg(C)) << "   CP A, C:" << endl;
  cout << "H:" << gb_cpu.getHflag() << " C:" << gb_cpu.getCflag() << " Z:" << gb_cpu.getZflag()<< endl;
  gb_cpu.setReg(A, 0xF2); gb_cpu.setReg(C, 0x32);
  gb_cpu.execInstr(CP_REG_OP | C);   //compare C to A
  cout << "A:0x" << unsigned(gb_cpu.getReg(A)) << " C:0x" << unsigned(gb_cpu.getReg(C)) << "   CP A, C:" << endl;
  cout << "H:" << gb_cpu.getHflag() << " C:" << gb_cpu.getCflag() << " Z:" << gb_cpu.getZflag()<< endl;
  gb_cpu.setReg(A, 0x63); gb_cpu.setReg(C, 0x63);
  gb_cpu.execInstr(CP_REG_OP | C);   //compare C to A
  cout << "A:0x" << unsigned(gb_cpu.getReg(A)) << " C:0x" << unsigned(gb_cpu.getReg(C)) << "   CP A, C:" << endl;
  cout << "H:" << gb_cpu.getHflag() << " C:" << gb_cpu.getCflag() << " Z:" << gb_cpu.getZflag()<< endl;
  
  /*Test Memory accessors*/
  //gb_lcd.refresh();
  addr = addr + (int8_t)id*2;
  cout << addr << endl;
  
  return 0;
}