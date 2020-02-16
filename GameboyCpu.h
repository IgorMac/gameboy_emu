#ifndef _GAMEBOY_CPU_H_
#define _GAMEBOY_CPU_H_

#include <cstdint>
#include "GameboyMem.h"

/* Internal CPU of the classic GameBoy */
class GameboyCpu{
    
  public:
    /* Constructor */
    GameboyCpu(GameboyMem *gb_mem);
    
    // void     powerUp();                       //start CPU and initialize memory
    int      run();                           //run CPU and execute instruction pointed by PC
    uint8_t  fetchNextInstr();                //fetch next instruction pointed by PC, and advance PC
    int      execInstr(uint8_t op);           //decode opcode and execute instruction
    void     execCBinstr(uint8_t op);         //decode CB-prefixed opcode and execute instruction
    void     enableInterrupts();           
    void     disableInterrupts();   
    
    /* set methods */
    void     setReg    (uint8_t id, uint8_t  val);                  ;   //set Register (id = A,B,C,D,F,H,L)
    void     setRegPair(uint8_t id, uint16_t val, bool mode=SP_MODE);   //set Register pair (id = BC,DE,HL,x) (x = AF if mode=true else SP)
    void     setSP     (uint16_t  val)                              ;   //set Stack Pointer
    void     setPC     (uint16_t  val)                              ;   //set Program Counter
    void     setCflag  (bool f)                                     ;   //set Carry flag to 1 or 0
    void     setHflag  (bool f)                                     ;   //set Half Carry flag to 1 or 0
    void     setZflag  (bool f)                                     ;   //set Substract flag to 1 or 0
    void     setNflag  (bool f)                                     ;   //set Zero flag to 1 or 0
                                               
    
    /* get methods */
    uint8_t  getReg    (uint8_t id                   );   //get Register (id = A,B,C,D,F,H,L)
    uint16_t getRegPair(uint8_t id, bool mode=SP_MODE);   //get Register pair (id = BC,DE,HL,x)(x = AF if mode=true else SP)
    uint16_t getSP     ()                             ;   //get Stack Pointer
    uint16_t getPC     ()                             ;   //get Program Counter
    bool     getCflag  ()                             ;   //get Carry flag (true = 1; false = 0)
    bool     getHflag  ()                             ;   //get Half Carry flag (true = 1; false = 0)
    bool     getNflag  ()                             ;   //get Substract flag (true = 1; false = 0)
    bool     getZflag  ()                             ;   //get Zero flag (true = 1; false = 0)
    
    
    /* stack methods */
    void     push(uint16_t val);                          //push the data onto the stack at address pointed by SP and then decrease SP by 2
    uint16_t pop ()            ;                          //pop the data from the stack at address pointed by SP and then increase SP by 2
    
    
    /* static functions */
    static uint8_t  regDstField (uint8_t op  );                          //return the field containing the destination reg id
    static uint8_t  regSrcField (uint8_t op  );                          //return the field containing the source reg id
    static uint8_t  regPairField(uint8_t op  );                          //return the field containing the register pair id
    static uint8_t  bitField    (uint8_t op  );                          //return the field containing the bit id for BIT, RES and SET instructions
    static bool     isZero      (uint8_t val );                          //true if value = 0 else false
    static bool     isCarry     (uint8_t op1, uint8_t op2, bool cin=0);  //true if op1 + op2 is generating a carry
    static bool     isHalfCarry (uint8_t op1, uint8_t op2, bool cin=0);  //true if op1 + op2 is generating a half carry
    static uint8_t  lowNibble   (uint8_t byte);                          // return the 4 bits lsb of byte
    static uint8_t  highNibble  (uint8_t byte);                          // return the 4 bits msb of byte
    static uint8_t  rotateRByte (uint8_t byte);                          //rotate byte right
    static uint8_t  rotateLByte (uint8_t byte);                          //rotate byte left
    static uint16_t extendSign  (uint8_t byte);                          //extend the sign for signed arithmetic
    
    
    
  private:                     
    GameboyMem *gb_mem;          //CPU adress space     
    
    
    uint8_t    a_reg ;          //Accumulator register
    uint8_t    f_reg ;          //Flags register
    uint8_t    b_reg ;          //B register
    uint8_t    c_reg ;          //C register
    uint8_t    d_reg ;          //D register
    uint8_t    e_reg ;          //E register
    uint8_t    h_reg ;          //H register
    uint8_t    l_reg ;          //L register
    uint16_t   sp_reg;          //stack pointer
    uint16_t   pc_reg;          //program counter
    
    
    /* private static functions */
    static bool isCarryN(uint8_t op1, uint8_t op2, bool cin, uint8_t n); 
};

#endif
