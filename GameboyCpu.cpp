#include <iostream>
#include "GameboyCpu.h"
#include "GM_constants.h"

/* Constructor */
GameboyCpu::GameboyCpu(GameboyMem *gb_mem)
{
  gb_mem  = gb_mem;    
  a_reg   =      0;
  f_reg   =      0;
  b_reg   =      0;
  c_reg   =      0;
  d_reg   =      0;
  e_reg   =      0;
  h_reg   =      0;
  l_reg   =      0;
  sp_reg  =      0;
  pc_reg  =      0;
}



/* run CPU and execute instruction pointed by PC */
int GameboyCpu::run(){
  uint8_t op;
  
  op = fetchNextInstr();   //fetch next instruction pointed by PC and then increment PC
  execInstr(op);           //decode opcode and execute instruction 
  
  return true;             //infinite loop (temporary)
} 


/* return opcode pointed by PC in memory, and advance PC */
uint8_t GameboyCpu::fetchNextInstr(){
  return gb_mem->readData(pc_reg++);
}


/*** set methods ***/
void GameboyCpu::setSP (uint16_t val){sp_reg = val;}    //set Stack Pointer
void GameboyCpu::setPC (uint16_t val){pc_reg = val;}    //set Program Counster


void GameboyCpu::setCflag  (bool f){     //set Carry flag to 1 or 0
  if(f) f_reg |=  C_MASK;                
  else  f_reg &= ~C_MASK;                
}                                        
void GameboyCpu::setHflag  (bool f){     //set Half Carry flag to 1 or 0
  if(f) f_reg |=  H_MASK;                
  else  f_reg &= ~H_MASK;                
}                                        
void GameboyCpu::setNflag  (bool f){     //set Substract flag to 1 or 0
  if(f) f_reg |=  N_MASK;                
  else  f_reg &= ~N_MASK;                
}                                        
void GameboyCpu::setZflag  (bool f){     //set Zero flag to 1 or 0
  if(f) f_reg |=  Z_MASK;
  else  f_reg &= ~Z_MASK;
  
}        

/*  set Register (id = A,B,C,D,F,H,L) */
void GameboyCpu::setReg(uint8_t id, uint8_t val){
  switch(id){
    case A : a_reg = val; break;
    case B : b_reg = val; break;
    case C : c_reg = val; break;
    case D : d_reg = val; break;
    case E : e_reg = val; break;
    case F : f_reg = val; break;
    case H : h_reg = val; break;
    case L : l_reg = val; break;
    default: 
      std::cerr << "Error: setReg: reg id " << id << " not defined." << std::endl;
  }
}

/*  set Register pair (id = BC,DE,HL,x) (x = AF if mode=true else SP) */
void GameboyCpu::setRegPair(uint8_t id, uint16_t val, bool mode){
  switch(id){
    case BC :
      b_reg = val >> 8  ;
      c_reg = val       ;
      break             ;
    case DE :           
      d_reg = val >> 8  ;
      e_reg = val       ;
      break             ;
    case HL :           
      h_reg = val >> 8  ;
      l_reg = val       ;
      break             ;
    case AF :                //AF = SP (numeric value)
      if (mode==AF_MODE){
        a_reg = val >> 8;
        f_reg = val     ;
      }
      else                   //mode=SP_MODE
        sp_reg = val    ;
      break             ;
    default: 
      std::cerr << "Error: setRegPair: reg id " << id << " not defined." << std::endl;
  }
}






/*** get methods ***/     
uint16_t GameboyCpu::getSP   (){return sp_reg;}                          //get Stack Pointer  
uint16_t GameboyCpu::getPC   (){return pc_reg;}                          //get Program Counter
bool     GameboyCpu::getCflag(){return ((f_reg >> 4) & 1) == 1;}         //get Carry flag (true = 1; false = 0)
bool     GameboyCpu::getHflag(){return ((f_reg >> 5) & 1) == 1;}         //get Half Carry flag (true = 1; false = 0)
bool     GameboyCpu::getNflag(){return ((f_reg >> 6) & 1) == 1;}         //get Substract flag (true = 1; false = 0)
bool     GameboyCpu::getZflag(){return ((f_reg >> 7) & 1) == 1;}         //get Zero flag (true = 1; false = 0)

/*  get Register (id = A,B,C,D,F,H,L) */
uint8_t GameboyCpu::getReg(uint8_t id){
  switch(id){
    case A : return a_reg;
    case B : return b_reg;
    case C : return c_reg;
    case D : return d_reg;
    case E : return e_reg;
    case F : return f_reg;
    case H : return h_reg;
    case L : return l_reg;
    default: 
      std::cerr << "Error: getReg: id (" << id << ") not defined." << std::endl;
  }
  return -1;
}



/*** stack methods ***/
/*  push the data onto the stack at address pointed by SP and then decrease SP by 2 */
void GameboyCpu::push(uint16_t val){
  uint16_t addr = sp_reg;
  gb_mem->writeData(addr-1, (val>>8));    //little-endian mode (write msb in highest adress)
  gb_mem->writeData(addr-2, val&0xFF);    //little-endian mode (write lsb in lowest adress)
  sp_reg -= 2                         ;    //point sp_reg on the last data put onto the stack
}

/*  pop the data from the stack at address pointed by SP and then increase SP by 2 */
uint16_t GameboyCpu::pop(){  
  uint8_t data_msb, data_lsb;
  uint16_t addr = sp_reg;
  data_lsb = gb_mem->readData(addr)  ;     //little-endian mode (read lsb in lowest adress)
  data_msb = gb_mem->readData(addr+1);     //little-endian mode (read msb in highest adress)  
  sp_reg  += 2                        ;     //point sp_reg on next data to be poped
  
  return (data_msb << 8) | data_lsb;
}
  




/* get Register pair (id = BC,DE,HL,x)(x = AF if mode=true else SP) */
uint16_t GameboyCpu::getRegPair(uint8_t id, bool mode){
  switch(id){
    case HL : return (h_reg << 8) | l_reg;
    case BC : return (b_reg << 8) | c_reg;
    case DE : return (d_reg << 8) | e_reg;
    case AF :                                 //AF = SP (numeric value)
      if(mode==AF_MODE)
        return (a_reg << 8) | f_reg;
      else                                    //mode=SP_MODE
        return sp_reg;
    default: 
      std::cerr << "Error: getRegPair: id (" << id << ") not defined." << std::endl;
  }
  return -1;
}


/* Decode and execute opcode of current instruction
 *@return number of CPU cycle the instruction took to execute
  */
int GameboyCpu::execInstr(uint8_t op){
  unsigned int tmp, tmp_old;                     //temporary working registers
  unsigned int          reg;                     //register id
  int              nb_cycle;                     //number of cycle of the instruction
  uint16_t             addr;                     //16-bits address register
  uint8_t          op1, op2;                     //operands register
  
  

  /*** CB prefix instructions ***/
  if(op == 0xCB){                                //First byte begin 0xCB are an extended set instructions, next byte being the real opcode                   
    tmp = gb_mem->readData(++pc_reg);             //read next op
    execCBinstr(tmp);                            //execute CB instruction and get number of cycle
    if(((op & 0xF) == 6)||((op & 0xF) == 0xE))   //opcodes whose last nibble is 0x6 and 0xE
      return 16+4;                               //takes 16 cycles to complete (involve 16-bits register HL manipulation) (+ 4 for 0xCB opcode)
    else                                         
      return  8+4;                               //else it takes 8 cycles (involve 8-bits register manipulation) (+ 4 for 0xCB opcode)
  }
  
  
  
  /*** NOP instruction ***/
  else if(op == NOP_OP)
    return 4;               //do nothing for 4 cycles
  
  
  
  /*** HALT instruction ***/
  else if(op == HALT_OP){   //need to be before LD (register and memory) clause
    //power down CPU until interrupt
    return 4;
  }
    
  
  /*** STOP instruction ***/
  else if(op == STOP_OP){  //need to be before jump clauses   
    //stop CPU & LCD until button pressed
    return 4;
  }
  
  
  
  /*** EI (Enable Interrupts) instruction ***/
  else if(op == EI_OP){
    //Enable interrupts AFTER this instruction has been executed
    return 4;
  }
  
 
  
  /*** DI (Disable Interrupts) instruction ***/
  else if(op == DI_OP){
    //Disable interrupts AFTER this instruction has been executed
    return 4;
  }
  
  

  /*** RST (Restart) instruction ***/
  /* RST (Restart at adress specified by the exp field of op) */
  else if((op & RST_MASK) == RST_OP){
    addr = op & RST_EXP_FIELD_MASK;        //get the exp field 
    push(pc_reg+1);                        //push return address
    pc_reg = addr-1;                       //PC will be incremented correctly after current instruction
    return 16;
  }
  
  
  
  
  /*** Carry bit instructions ***/
  /* CCF (Complement Carry Flag) */
  else if(op == CCF_OP){                  
    setCflag(!getCflag());                  //Inverse carry bit  
    setHflag(0);
    setNflag(0);
    return 4;
  }
  /* SCF (Set Carry Flag) */
  else if(op == SCF_OP){              
    setCflag(1);
    setHflag(0);
    setNflag(0);                    
    return 4;
  }
  
  

  /*** New GameBoy SP instructions ***/
  /* LD (Load SP to memory immediate addressing) */
  else if(op == LD_A16_SP_OP){
    addr = gb_mem->readData(++pc_reg);            //get LSB
    tmp  = gb_mem->readData(++pc_reg);            //get MSB
    addr = (tmp << 8) | addr;                    //put them together (little-endian mode)
    gb_mem->writeData16b(addr, getRegPair(SP));  
    return 20;
  }
  /* LD (Load SP + signed immediate to HL) */
  else if(op == LD_HL_SP_PLUS_I8_OP){
    tmp     = gb_mem->readData(++pc_reg);         //get 8-bit signed immediate offset
    tmp     = extendSign(tmp);                   //extend the sign of the offset for 16-bits signed addition
    tmp_old = getRegPair(SP);                   
    addr = tmp_old + tmp;                       
    setRegPair(HL, addr);                       
    setZflag(0);                                 //set flags accordingly
    setNflag(0);                                 //it's not a substraction
    setCflag(isCarry(tmp_old, tmp));
    setHflag(isHalfCarry(tmp_old,tmp)); 
    return 12;
  }
  /* LD (Load SP from HL) */
  else if(op == LD_SP_HL_OP){
    setRegPair(SP, getRegPair(HL));
    return 8;
  }
  /* ADD (Add signed immediate to SP) */
  else if(op == ADD_SP_I8_OP){
    tmp = gb_mem->readData(++pc_reg);             //get the 8-bits signed immediate
    tmp = extendSign(tmp);                       //extend the sign for 16-bits signed addition
    tmp_old = getRegPair(SP);                   
    addr = tmp_old + tmp;                       
    setRegPair(SP, addr);                        //put the result back to SP
    setZflag(0);                                 //set flags accordingly
    setNflag(0);                                 //it's not a substraction
    setCflag(isCarry(tmp_old, tmp));         
    setHflag(isHalfCarry(tmp_old,tmp)); 
    return 16;
  }
  
  
  
  
  /*** Single register instructions ***/
  /* INC (Increment (registers or memory)) */
  else if((op & INCDEC_REG_MASK) == INC_REG_OP){
    reg = GameboyCpu::regDstField(op);                    //take the id of the reg
    if(reg == M){                                      //if it's a memory inc
      tmp = gb_mem->readData(getRegPair(HL));                     //get the data at adress HL
      tmp_old = tmp;                                     //saving for the Half Carry calculation
      tmp++;                                             
      gb_mem->writeData(getRegPair(HL), tmp);                         
      nb_cycle = 12;                                     //Number of cycle the instruction took in the CPU
    }else{                                             //else it's a register inc     
      tmp = getReg(reg);                                 //get the data in reg
      tmp_old = tmp;                                     //saving for the Half Carry calculation
      tmp++;                                             
      setReg(reg, tmp);                                  
      nb_cycle = 4;                                      //Number of cycle the instruction took in the CPU
    }
    setHflag(GameboyCpu::isHalfCarry(tmp_old, 1));     //setting correctly the H flag                                      
    setZflag(GameboyCpu::isZero(tmp));                 //same for Zero flag                                    
    setNflag(0);                                       //it's not a substraction
    return nb_cycle;                                        
  }
  /* DEC (Decrement (registers or memory)) */
  else if((op & INCDEC_REG_MASK) == DEC_REG_OP){
    reg = GameboyCpu::regDstField(op);                         //take the id of the reg
    if(reg == M){                                           //if it's a memory 
      tmp = gb_mem->readData(getRegPair(HL));                          //get the data at adress HL
      tmp_old = tmp;                                          //saving for the Half Carry calculation
      tmp--;                                                  
      gb_mem->writeData(getRegPair(HL), tmp);                              
      nb_cycle = 10;                                          //Number of cycle the instruction took in the CPU
    }else{                                                  //else it's a register      
      tmp = getReg(reg);                                      //get the data in reg
      tmp_old = tmp;                                          //saving for the Half Carry calculation
      tmp--;                                                  
      setReg(reg, tmp);                                       
      nb_cycle = 5;                                           //Number of cycle the instruction took in the CPU
    }
    setHflag(GameboyCpu::isHalfCarry(tmp_old, -1));         //set if no borrow                                      
    setZflag(GameboyCpu::isZero(tmp));                      //same for Zero flag                                                                          
    setNflag(1);                                            //it's a substraction
    return nb_cycle;    
  }
  /* CPL (Complement register A) */
  else if(op == CPL_OP){
    tmp = getReg(A);                                        //get accumulator 
    setReg(A, ~tmp);                                        //flip all bits
    setNflag(1);                                            //set N no matter the result
    setHflag(1);                                            //set H no matter the result
    return 4;                                               //takes 4 CPU cycles
  }
  /* DAA (Decimal Adjust register A) */
  /* used for adjusting result of a BCD arithmetic operation to BCD format */
  else if(op == DAA_OP){
    tmp = getReg(A);                                         //get accumulator
    if(getHflag() || (GameboyCpu::lowNibble(tmp) > 9))       //if H flag is set or low 4 bits of acc > 9
      tmp += 6;                                              //add 6 to lsb
    if(getCflag() || (GameboyCpu::highNibble(tmp) > 9)){     //if C flag is set or high 4 bits of acc > 9
      setCflag(GameboyCpu::isCarry(tmp, 0x60));           //set carry accordingly
      tmp += 0x60;                                           //add 6 to msb
    }
    setHflag(0);                                             //always set H flag to 0 
    setZflag(GameboyCpu::isZero(tmp));
    return 4;
  } 
  

  
  
  /*** Data transfert instructions ***/
  /* LD (Load Data 8-bits registers and memory) */
  else if((op & LD_REG_MASK) == LD_REG_OP){
    reg = GameboyCpu::regSrcField(op);          //get the source register id
    nb_cycle = 4;                               //nb of cycle is 4 (if it's operating only on register)
    if (reg == M){                              //if it's a memory source
      tmp = gb_mem->readData(getRegPair(HL));    //read Data from memory address HL
      nb_cycle = 8;                             //no there is a memory operation so it will finally take 8 cylcles
    }
    else                                        //else it's a register source
      tmp = getReg(reg);
    reg = GameboyCpu::regDstField(op);          //get the distination resiter id
    if (reg == M){                              //if it's a memory destination
      gb_mem->writeData(getRegPair(HL), tmp);           //write Data to memory address HL
      nb_cycle = 8;                             //no there is a memory operation so it will finally take 8 cylcles
    }
    else                                        //else it's a register destination
      setReg(reg, tmp);
    
    return nb_cycle;
  }
  /* LD (Load Data A register from/to memory) */
  else if((op & LD_A_M_MASK) == LD_A_M_OP){
    reg  = GameboyCpu::regPairField(op);
    if (reg == AF){                           //if reg pair id is AF (or SP)
      reg = HL;                               //choose HL istead
      op1 = 0xFF;                             //and decrement HL after instruction (special instruction)
    }else if(reg == HL)                       //if it's HL
      op1 =  1;                               //increment HL after instruction (special instruction)
    else 
      op1  = 0;                               //no increment and decrement on other registers
    addr = getRegPair(reg);                   //load the adress
    if(op & BIT3_MASK) {                      //if bit 3 is set it's LD A from memory
      tmp = gb_mem->readData(addr);
      setReg(A, tmp);
    }
    else {                                    //if bit 3 is not set it's LD A to memory
      tmp = getReg(A);
      gb_mem->writeData(addr,tmp);
    }
    setRegPair(HL, addr+op1);                 //increment or decrement (or not)
    return 8;
  }
  /* LDH (Load Data A register from/to memory address 0xFF plus 8-bits immediate) */
  else if((op & LD_A_FF_PLUS_I8_MASK) == LD_A_FF_PLUS_I8_OP){
    addr = gb_mem->readData(++pc_reg);         //get address LSB in next instruction byte
    addr = 0xFF00 | addr;                     //add 0xFF MSB
    if(op & BIT4_MASK){                       //if bit 4 is set, then it's load A from memory
      tmp = gb_mem->readData(addr);
      setReg(A, tmp);
    }else{                                    //else it's load A to memory
      tmp = getReg(A);
      gb_mem->writeData(addr, tmp);       
    }
    return 12;
  }
  /* LD (Load Data A register from/to memory address 0xFF plus C register content) */
  else if((op & LD_A_FF_PLUS_C_MASK) == LD_A_FF_PLUS_C_OP){
    addr = getReg(C);                         //get address LSB in C
    addr = 0xFF00 | addr;                     //add 0xFF MSB
    if(op & BIT4_MASK){                       //if bit 4 is set, then it's load A from memory
      tmp = gb_mem->readData(addr);
      setReg(A, tmp);
    }else{                                    //else it's load A to memory
      tmp = getReg(A);
      gb_mem->writeData(addr, tmp);       
    }
    return 8;
  }
  /* LD (Load Data A register from/to memory immediate addressing) */
  else if((op & LD_A_M_I16_MASK) == LD_A_M_I16_OP){
    addr = gb_mem->readData(++pc_reg);         //get address LSB in next instruction byte
    tmp  = gb_mem->readData(++pc_reg);         //get address MSB in next instruction byte
    addr = tmp| addr;                         //add them together
    if(op & BIT4_MASK){                       //if bit 4 is set, then it's load A from memory
      tmp = gb_mem->readData(addr);
      setReg(A, tmp);
    }else{                                    //else it's load A to memory
      tmp = getReg(A);
      gb_mem->writeData(addr, tmp);       
    }
    return 16;
  }
  
  
  
  
  
  
  /*** Register or memory to accumulator ALU instructions ***/
  else if((op & REG_ALU_MASK) == REG_ALU_OP){
    reg = GameboyCpu::regSrcField(op);                          //get the reg id of the second operand
    op1 = getReg(A);                                            //load first operand being always register A content
    if(reg == M){                                               //if the second operand is from memory
      op2 = gb_mem->readData(getRegPair(HL));                      //load data from adress HL
      nb_cycle = 8;                                               //it will take 8 cycles
    }else{                                                      
      op2 = getReg(reg);                                        //else it's from a register
      nb_cycle = 4;                                               //so it will take only 4 cycles
    }                                                           
    /* ADD (Add register or memory to accumulator */             
    if((op & REG_TO_A_OPERATOR_MASK) == ADD_REG_OP){           
      tmp = op1 + op2;                                          //do the addition
      setReg(A, tmp);                                           //store the result back to the accumulator
      setCflag(GameboyCpu::isCarry(op1,op2));                   //set the flags accordingly
      setHflag(GameboyCpu::isHalfCarry(op1,op2));            
      setZflag(GameboyCpu::isZero(tmp));                                           
      setNflag(0);                                              //it's not a substraction
    }
    /* ADC (Add register or memory with carry to accumulator) */
    else if((op & REG_TO_A_OPERATOR_MASK) == ADC_REG_OP){                                                       
      tmp = op1 + op2 + getCflag();                             //do the addition with carry in
      setReg(A, tmp);                                           //store the result back to the accumulator
      setCflag(GameboyCpu::isCarry(op1,op2,getCflag()));        //set the flags accordingly
      setHflag(GameboyCpu::isHalfCarry(op1,op2,getCflag()));
      setZflag(GameboyCpu::isZero(tmp));
      setNflag(0);                                              //it's not a substraction
    }
    /* SUB (Substract register or memory from accumulator) */
    else if((op & REG_TO_A_OPERATOR_MASK) == SUB_REG_OP){                                                      
      tmp = op1 + (~op2+1);                                     //do the subtraction
      setReg(A, tmp);                                           //store the result back to the accumulator
      setCflag(!GameboyCpu::isCarry(op1,~op2+1));               //set the flags accordingly
      setHflag(!GameboyCpu::isHalfCarry(op1,~op2+1));           //there is a borrow if there is no carry (2s complement addition)
      setZflag(GameboyCpu::isZero(tmp));
      setNflag(1);                                              //it's a substraction
    }
    /* SBC (Substract register or memory with borrow from accumulator) */
    else if((op & REG_TO_A_OPERATOR_MASK) == SBC_REG_OP){            
      op2 +=     getCflag();                                    //add the borrow to op2 (C set if borrow)
      tmp  = op1 + (~op2+1);                                    //do the subtraction
      setReg(A, tmp);                                           //store the result back to the accumulator
      setCflag(!GameboyCpu::isCarry(op1,~op2+1));               //set the flags accordingly
      setHflag(!GameboyCpu::isHalfCarry(op1,~op2+1));           //there is a borrow if there is no carry (2s complement addition)
      setZflag(GameboyCpu::isZero(tmp));
      setNflag(1);                                              //it's a substraction
    }
    /* AND (Logical AND register or memory with accumulator) */
    else if((op & REG_TO_A_OPERATOR_MASK) == AND_REG_OP){
      tmp = op1 & op2;                                          //logical and
      setReg(A, tmp);                                           //store the result back to the accumulator
      setCflag(0);                                              //set the flags accordingly
      setHflag(1);                                              
      setZflag(GameboyCpu::isZero(tmp));
      setNflag(0);                                              //it's not a substraction
    }
    /* XOR (Logical Exclusive-Or register or memory with accumulator) */
    else if((op & REG_TO_A_OPERATOR_MASK) == XOR_REG_OP) {
      tmp = op1 ^ op2;                                          //logical xor
      setReg(A, tmp);                                           //store the result back to the accumulator
      setCflag(0);                                              //set the flags accordingly
      setHflag(0);                                             
      setZflag(GameboyCpu::isZero(tmp));
      setNflag(0);                                              //it's not a substraction
    }
    /* OR (Logical OR register or memory with accumulator) */
    else if((op & REG_TO_A_OPERATOR_MASK) == OR_REG_OP) {
      tmp = op1 | op2;                                          //logical or
      setReg(A, tmp);                                           //store the result back to the accumulator
      setCflag(0);                                              //set the flags accordingly
      setHflag(0);                                              
      setZflag(GameboyCpu::isZero(tmp));
      setNflag(0);                                              //it's not a substraction
    }
    /* CP (Compare register or memory with accumulator) */
    else if((op & REG_TO_A_OPERATOR_MASK) == CP_REG_OP) {
      tmp = op1 + (~op2+1);                                   //do the subtraction but not store it in A
      setCflag(!GameboyCpu::isCarry(op1,~op2+1));             //set the flags accordingly
      setHflag(!GameboyCpu::isHalfCarry(op1,~op2+1));         //set if borrow
      setZflag(GameboyCpu::isZero(tmp));
      setNflag(1);                                            //it does a substraction to compare
    }
    else{
      std::cerr << "Error: instruction " << op << " not defined in REG_TO_A_OPERATOR" << std::endl;
      return -1;
    }
    return nb_cycle;
  }
  
  
  
  /*** Rotate accumulator instructions ***/
  else if((op & ROT_A_MASK) == ROT_A_OP){
    tmp = getReg(A);                            //get the content of the accumulator
    /* RLCA (Rotate left carry accumulator) */    
    if(op == RLCA_OP) {                    
      setCflag(tmp >> 7);                        //put the leftmost bit to carry
      tmp = GameboyCpu::rotateLByte(tmp);        //rotate byte left                      
    }                                           
    /* RRCA (Rotate right carry accumulator) */
    else if(op == RRCA_OP) {                    
      setCflag(tmp & BIT0_MASK);                 //put the rightmost bit to carry
      tmp = GameboyCpu::rotateRByte(tmp);        //rotate byte right
    }
    /* RLA (Rotate accumulator left through carry) */
    else if (op == RLA_OP){                     
      tmp_old = getCflag();                      //save C flag
      tmp = GameboyCpu::rotateLByte(tmp);        //rotate byte left
      setCflag(tmp & BIT0_MASK);                 //set C to rightmost bit value (old leftmost bit)
      tmp = (tmp ^ BIT0_MASK) | tmp_old;         //reset to 0 bit 0 and then set it to old carry value
    }
    /* RRA (Rotate accumulator right through carry) */
    else if (op == RRA_OP){                     
      tmp_old = getCflag();                      //save C flag
      tmp = GameboyCpu::rotateRByte(tmp);        //rotate byte right
      setCflag(tmp >> 7);                        //set C to leftmost bit value (old rightmost bit)
      tmp = (tmp ^ BIT7_MASK) | (tmp_old << 7);  //reset to 0 bit 7 and then set it to old carry value
    }                                                     
    setReg(A, tmp);                              //store value back to A
    setZflag(0);                                 //set flag accordingly
    setNflag(0);                                 //it's not a substraction
    setHflag(0);  
    return 4;
  }
 
  
  /*** Register pair instructions ***/
  /* PUSH (Push data onto the stack) */
  else if((op & REG_PAIR_MASK) == PUSH_OP){
    reg = GameboyCpu::regPairField(op);                      //get reg pair id in op
    tmp = getRegPair(reg, AF_MODE);                          //get the value of the register pair with id 0b11 being register Pair AF
    push(reg);                                               //push the data onto the stack and decrease SP by 2
    return 16;                                               
  }                                                          
  /* POP (Pop data off the stack) */                           
  else if((op & REG_PAIR_MASK) == POP_OP){                   
    reg = GameboyCpu::regPairField(op);                      //get reg pair id in op
    tmp = pop();                                             //pop the data off the stack and increase SP by 2
    setRegPair(reg, tmp, AF_MODE);                           //put the data on register reg with id 0b11 being register Pair AF
    return 12;
  }
  /* ADD (Add register pair to HL) */
  else if((op & REG_PAIR_MASK) == ADD_REG_PAIR_OP){
    reg = GameboyCpu::regPairField(op);                      //get reg pair id in op
    op1 = getRegPair(reg) >> 8;                              //save msb for carries calculation
    op2 = getRegPair(HL) >> 8;                               //save msb for carries calculation
    tmp = getRegPair(reg) + getRegPair(HL);                
    setRegPair(HL, tmp);                                     //save back to HL  
    setCflag(GameboyCpu::isCarry(op1,op2));                  //calculating carry of the 2 msb
    setHflag(GameboyCpu::isHalfCarry(op1,op2));              //calculating half carry of the 2 msb
    setNflag(0);                                             //not a substraction
    return 10;
  }
  /* INC (Increment register pair) */
  else if((op & REG_PAIR_MASK) == INC_REG_PAIR_OP){
    reg = GameboyCpu::regPairField(op);                      //get reg pair id in op
    tmp = getRegPair(reg);                                   //reg=11 is SP and not AF    
    tmp++;                            
    setRegPair(reg, tmp);                                    //save back to reg
    return 8;
  }
  /* DEC (Decrement register pair) */
  else if((op & REG_PAIR_MASK) == DEC_REG_PAIR_OP){
    reg = GameboyCpu::regPairField(op);                      //get reg pair id in op
    tmp = getRegPair(reg);                                   //reg=11 is SP and not AF    
    tmp--;                            
    setRegPair(reg, tmp);                                    //save back to reg
    return 8;
  }
  /* LD (Load SP from HL) */
  else if((op & REG_PAIR_MASK) == LD_SP_OP) {                     
    tmp = getRegPair(HL);                                    //get HL content           
    setRegPair(SP, tmp);                                     //loading it in SP
    return 8;
  }
  
  
  
  /***Immediate instructions***/
  /* LD (Load 8-bits Immediate to register) */
  else if((op & LD_I8_MASK) == LD_I8_OP){            
    reg = GameboyCpu::regDstField(op);                       //load the destination reg id
    tmp = gb_mem->readData(++pc_reg);                         //increment PC then read data at adress pointed by PC
    if(reg == M){                                            //if it's a memory destination
      gb_mem->writeData(getRegPair(HL), tmp);                   //write data back to memory at adress HL
      nb_cycle = 12;                                        
    }else{                                                  
      setReg(reg, tmp);                                      //else write in register
      nb_cycle = 8;                                         
    }                                                       
    return nb_cycle;                                        
  } 
  /* LD (Load 16-bits Immediate to register pair) */
  else if((op & LD_I16_MASK) == LD_I16_OP){            
    reg     = GameboyCpu::regPairField(op);                  //get the reg pair id
    tmp_old = gb_mem->readData(++pc_reg);                     //next byte is immediate lsb
    tmp     = gb_mem->readData(++pc_reg);                     //next byte is immediate msb
    tmp     = (tmp << 8) | tmp_old;                          //put them together 
    setRegPair(reg, tmp);                                               
    return 12;                                        
  } 


  
  /*** Immediate to accumulator ALU instruction ***/                  
  else if((op & IMM_ALU_MASK) == IMM_ALU_OP){              
    op1 = getReg(A);                                         //load accumulator in first operand
    op2 = gb_mem->readData(++pc_reg);                         //increment PC then store data at adress pointed by PC in second operand
    /* ADD (Add immediate to accumulator) */                  
    if(op == ADD_IMM_OP){                                   
      tmp = op1 + op2;                                              
      setZflag(GameboyCpu::isZero(tmp));                     //set flags accordingly
      setCflag(GameboyCpu::isCarry(op1,op2));               
      setHflag(GameboyCpu::isHalfCarry(op1,op2));            
      setNflag(0);                                           //not a substraction
      setReg(A, tmp);                                        //store back the result in accumulator
    }
    /* ADC (Add immediate with carry to accumulator) */
    else if(op == ADC_IMM_OP){
      tmp = op1 + op2 + getCflag();
      setZflag(GameboyCpu::isZero(tmp));                     //set flags accordingly
      setCflag(GameboyCpu::isCarry(op1,op2,getCflag()));
      setHflag(GameboyCpu::isHalfCarry(op1,op2,getCflag())); 
      setNflag(0);                                           //not a substraction
      setReg(A, tmp);                                        //store back the result in accumulator
    }
    /* SUB (Substract immediate from accumulator) */
    else if(op == SUB_IMM_OP){                         
      tmp = op1 + (~op2+1);                                  //substration with 2s complement addition
      setZflag(GameboyCpu::isZero(tmp));                     //set flags accordingly
      setCflag(!GameboyCpu::isCarry(op1,~op2+1));            //there is a burrow if there is no carry (2s complement addition)
      setHflag(!GameboyCpu::isHalfCarry(op1,~op2+1)); 
      setNflag(1);                                           //it's a substraction
      setReg(A, tmp);                                        //store back the result in accumulator
    }
    /* SBC (Substract immediate from accumulator with borrow) */
    else if(op == SBC_IMM_OP){                         
      op2 +=     getCflag();                                 //add the borrow to op2 (C set if borrow)
      tmp = op1 + (~op2+1);                                  //substration with 2s complement addition
      setZflag(GameboyCpu::isZero(tmp));                     //set flags accordingly
      setCflag(!GameboyCpu::isCarry(op1,~op2+1));            //there is a borrow if there is no carry (2s complement addition)
      setHflag(!GameboyCpu::isHalfCarry(op1,~op2+1)); 
      setNflag(1);                                           //it's a substraction
      setReg(A, tmp);                                        //store back the result in accumulator
    }
    /* AND (Logical AND immediate with accumulator) */
    else if(op == AND_IMM_OP){
      tmp = op1 & op2;
      setZflag(GameboyCpu::isZero(tmp));                     //set flags accordingly
      setCflag(0);            
      setHflag(1); 
      setNflag(0);                                           //not a substraction
      setReg(A, tmp);                                        //store back the result in accumulator
    }
    /* XOR (Logical Exclusive-Or immediate with accumulator */
    else if(op == XOR_IMM_OP){
      tmp = op1 ^ op2;
      setZflag(GameboyCpu::isZero(tmp));                     //set flags accordingly
      setCflag(0);            
      setHflag(0); 
      setNflag(0);                                           //not a substraction
      setReg(A, tmp);                                        //store back the result in accumulator
    }
    /* OR (Logical OR immediate with accumulator) */
    else if(op == OR_IMM_OP){
      tmp = op1 | op2;
      setZflag(GameboyCpu::isZero(tmp));                     //set flags accordingly
      setCflag(0);            
      setHflag(0); 
      setNflag(0);                                           //not a substraction  
      setReg(A, tmp);                                        //store back the result in accumulator
    }
    /* CP (Compare immediate with accumulator) */
    else if(op == CP_IMM_OP){
      tmp = op1 + (~op2+1);                                  //do the subtraction but not store it in A
      setCflag(!GameboyCpu::isCarry(op1,~op2+1));            //set the flags accordingly
      setHflag(!GameboyCpu::isHalfCarry(op1,~op2+1));        //set if borrow
      setZflag(GameboyCpu::isZero(tmp));
      setNflag(1);                                           //it does a substraction to compare
    }
    return 8;
  }
  
  
  
  /*** Direct Adressing Instruction ***/
  /* LD (Load accumulator from/to memory immediate adressing) */
  else if((op & LD_A_ADDRI_OP) == LD_A_ADDRI_MASK){
    addr = gb_mem->readData(++pc_reg);                //getting the address LSB byte
    tmp  = gb_mem->readData(++pc_reg);                //getting the address MSB byte
    addr = (tmp << 8) | addr        ;                //put them together 
    if(op & BIT4_MASK){                              //if it's from memory
      tmp = gb_mem->readData(addr);             
      setReg(A, tmp);
    }else{
      tmp = getReg(A);
      gb_mem->writeData(addr, A);
    }
    return 16;
  }
  /* LD (Load SP to memory immediate adressing) */
  else if(op == LD_SP_ADDRI_OP){
    addr = gb_mem->readData(++pc_reg)    ;              //getting the address LSB byte
    tmp  = gb_mem->readData(++pc_reg)    ;              //getting the address MSB byte
    addr = (tmp << 8) | addr            ;              //put them together 
    tmp  = getRegPair(SP)               ;
    gb_mem->writeData(tmp, addr)         ;              //store LSB in addr location (little-endian mode)
    gb_mem->writeData((tmp >> 8), addr+1);              //store MSB in addr+1 location (little-endian mode)
    return 16;
  }
  
  
  
  /*** JUMP instructions ***/    
  /* Jump adress 16-bits unsigned immediate instruction mask */
  else if((op & JUMP_I16_MASK) == JUMP_I16_OP){
    addr    = gb_mem->readData(++pc_reg);                  //getting the address LSB byte
    tmp     = gb_mem->readData(++pc_reg);                  //getting the address MSB byte
    addr    = (tmp << 8) | addr        ;                  //put them together 
    nb_cycle = 12;                                        //12 cycles if no branch is taken, 16 if so
    /* JP (Jump to adress 16-bits immediate) */
    if(op == JP_I16_OP){
      pc_reg = addr-1;                                    //PC will be incremented correctly after current instruction
      nb_cycle = 16;
    }
    /* JP (Jump if Not Carry to adress 16-bits immediate) */
    else if(op == JP_NC_I16_OP){
      if(!getCflag()){
        pc_reg = addr-1;                                  //PC will be incremented correctly after current instruction 
        nb_cycle = 16;
      }
    }
    /* JP (Jump if Carry to adress 16-bits immediate) */
    else if(op == JP_C_I16_OP){
      if(getCflag()){
        pc_reg = addr-1;                                  //PC will be incremented correctly after current instruction
        nb_cycle = 16;
      }
    }
    /* JP (Jump if Not Zero to adress 16-bits immediate) */
    else if(op == JP_NZ_I16_OP){
      if(!getZflag()){
        pc_reg = addr-1;                                  //PC will be incremented correctly after current instruction
        nb_cycle = 16;
      }
    }
    /* JP (Jump if Zero to adress 16-bits immediate) */
    else if(op == JP_Z_I16_OP){
      if(getZflag()){                                    
        pc_reg = addr-1;                                  //PC will be incremented correctly after current instruction
        nb_cycle = 16;
      }
    }
    return nb_cycle;
  }
  /* Jump adress 8-bits signed immediate instruction */
  else if((op & JUMP_I8_MASK) == JUMP_I8_OP){
    tmp     = gb_mem->readData(pc_reg+1);                  //get immediate signed byte 
    tmp     = extendSign(tmp);                            //extend his sign for 16-bits signed addition
    nb_cycle = 8;                                         //8 cycles if no branch is taken, 12 if so
    /* JR (Jump to adress 8-bits signed immediate) */
    if(op == JR_I8_OP){
      pc_reg += tmp-1;                                    //PC will be incremented correctly after current instruction
      nb_cycle = 12;
    }
    /* JR (Jump if Not Carry to adress 8-bits signed immediate) */
    else if(op == JR_NC_I8_OP){
      if(!getCflag()){
        pc_reg += tmp-1;                                  //PC will be incremented correctly after current instruction 
        nb_cycle = 12;
      }
    }
    /* JR (Jump if Carry to adress 8-bits signed immediate) */
    else if(op == JR_C_I8_OP){
      if(getCflag()){
        pc_reg += tmp-1;                                  //PC will be incremented correctly after current instruction
        nb_cycle = 12;
      }
    }
    /* JR (Jump if Not Zero to adress 8-bits signed immediate) */
    else if(op == JR_NZ_I8_OP){
      if(!getZflag()){
        pc_reg += tmp-1;                                  //PC will be incremented correctly after current instruction
        nb_cycle = 12;
      }
    }
    /* JR (Jump if Zero to adress 8-bits signed immediate) */
    else if(op == JR_Z_I8_OP){
      if(getZflag()){                                    
        pc_reg += tmp-1;                                  //PC will be incremented correctly after current instruction
        nb_cycle = 12;
      }
    }
  }
  /* JP (Jump to adress contained by HL) */
  else if(op == JP_HL_OP){
    addr = getRegPair(HL);
    pc_reg = addr-1;
    return 4;
  }
  
  
  
  
  /*** Call instructions ***/
  else if((op & CALL_MASK) == CALL_OP){                         
    addr     = gb_mem->readData(++pc_reg);                   //getting the address LSB byte
    tmp      = gb_mem->readData(++pc_reg);                   //getting the address MSB byte
    addr     = (tmp << 8) | addr        ;                   //put them together
    nb_cycle =  12;
    /* CALL (Call to 16-bits immediate address) */
    if(op == CALL_A16_OP){
      push(pc_reg+1)   ;                                    //push return adress for RET (return) instructions
      pc_reg = addr-1;                                      //PC will be incremented correctly after current instruction
      nb_cycle = 24  ;
    }
    /* CALL (Call if Not Carry to 16-bits immediate address) */
    else if(op == CALL_NC_OP){
      if(!getCflag()){
        push(pc_reg+1)   ;                                  //push return adress for RET (return) instructions
        pc_reg = addr-1;                                    //PC will be incremented correctly after current instruction 
        nb_cycle = 24;
      }
    }
    /* CALL (Call if Carry to 16-bits immediate address) */
    else if(op == CALL_C_OP){
      if(getCflag()){
        push(pc_reg+1)   ;                                  //push return adress for RET (return) instructions
        pc_reg = addr-1;                                    //PC will be incremented correctly after current instruction
        nb_cycle = 24;
      }
    }
    /* CALL (Call if Not Zero to 16-bits immediate address) */
    else if(op == CALL_NZ_OP){
      if(!getZflag()){
        push(pc_reg+1)   ;                                  //push return adress for RET (return) instructions
        pc_reg = addr-1;                                    //PC will be incremented correctly after current instruction
        nb_cycle = 24;
      }
    }
    /* CALL (Call if Zero to 16-bits immediate address) */
    else if(op == CALL_Z_OP){
      if(getZflag()){                                    
        push(pc_reg+1)   ;                                  //push return adress for RET (return) instructions
        pc_reg = addr-1;                                    //PC will be incremented correctly after current instruction
        nb_cycle = 24;
      }
    }
    else{                                                   
      std::cerr << "Error: instruction " << op << " not defined in ISA." << std::endl;    
      return -1;
    }
    return nb_cycle;
  }
  
  
  
  /*** Return instructions ***/
  /* RET  (Return unconditionally) */
  else if(op == RET_UNC_OP){
    addr = pop()   ;                                      //pop adress put by a earlier CALL instruction
    pc_reg = addr-1;                                      //PC will be incremented correctly after current instruction
    return 16      ;
  }
  /* RET  (Return if Not Carry) */
  else if(op == RET_NC_OP){
    if(!getCflag()){
      addr = pop()   ;                                    //pop adress put by a earlier CALL instruction
      pc_reg = addr-1;                                    //PC will be incremented correctly after current instruction 
      return 16      ;
    }else 
      return 8;
  }
  /* RET  (Return if Carry) */
  else if(op == RET_C_OP){
    if(getCflag()){
      addr = pop()   ;                                    //pop adress put by a earlier CALL instruction
      pc_reg = addr-1;                                    //PC will be incremented correctly after current instruction
      return 16      ;
    }else 
      return 8;
  }
  /* RET  (Return if Not Zero) */
  else if(op == RET_NZ_OP){
    if(!getZflag()){
      addr = pop()   ;                                    //pop adress put by a earlier CALL instruction
      pc_reg = addr-1;                                    //PC will be incremented correctly after current instruction
      return 16      ;
    }else 
      return 8;
  }
  /* RET  (Return Zero) */
  else if(op == RET_Z_OP){
    if(getZflag()){                                    
      addr = pop()   ;                                    //pop adress put by a earlier CALL instruction
      pc_reg = addr-1;                                    //PC will be incremented correctly after current instruction
      return 16      ;
    }else 
      return 8;
  }
  /* RETI (Return then enable interrupts) */
  else if(op == RETI_OP){                                  
      addr = pop()      ;                                 //pop adress put by a earlier CALL instruction
      pc_reg = addr-1   ;                                 //PC will be incremented correctly after current instruction
      enableInterrupts();                                 //and then enable interrupts
      return 16         ;
  }
 
  
  
  std::cerr << "Error: instruction " << op << " not defined in ISA." << std::endl;    
  return -1;
}




/* Decode and execute opcode of CB-prefixed instruction */
void GameboyCpu::execCBinstr(uint8_t op){ 
  uint16_t        tmp;                      //temporary working registers
  uint16_t lbit, rbit;                      //leftmost and rightmost bit saving
  uint16_t      carry;                      //carry saving
  uint8_t       bit_n;                      //bit position
  uint8_t         reg;                      //reg id 
  int         nb_bits;                      //number of bits of the operands
  
  
  
  reg = regSrcField(op);                    //get register id
  if(reg == M){                             //if it's memory id M 
    tmp     = getRegPair(HL);               //then change it for HL (special for CB-prefixed instructions)
    nb_bits = 16;                           //16-bits operands
  }else{
    tmp = getReg(reg);                      //else it's a regular 8-bits register
    nb_bits = 8;                            //8-bits operand
  }

  
    
  /*** Rotate, Shift and Swap instructions ***/
  if((op & CB_RSS_MASK) == CB_RSS_OP){
    /* RLC (Rotate left) */
    if((op & CB_OPERATOR_MASK) == RLC_OP){            
      lbit  = (tmp >> (nb_bits-1));                       //saving the msb bit
      tmp = (tmp << 1) | lbit;                          //shift left and put the msb to lsb bit
      setCflag(lbit);                                   //put the old msb bit to Carry flag
    }                                                 
    /* RRC (Rotate right) */                          
    else if((op & CB_OPERATOR_MASK) == RRC_OP){       
      rbit = (tmp & BIT0_MASK);                         //saving the lsb bit
      tmp = (tmp >> 1) | (rbit << (nb_bits-1));           //shift right and put the lsb to msb bit
      setCflag(rbit);                                   //put the old lsb bit to Carry flag
    }                                                 
    /* RL (Rotate left through carry) */              
    else if((op & CB_OPERATOR_MASK) == RL_OP){        
      carry = getCflag();                               //saving carry
      setCflag(tmp >> (nb_bits-1));                       //put msb bit in carry
      tmp = (tmp << 1) | carry;                         //shift left and put old carry in lsb bit*
    }
    /* RR (Rotate right through carry) */
    else if((op & CB_OPERATOR_MASK) == RR_OP){ 
      carry = getCflag();                               //saving carry
      setCflag(tmp & BIT0_MASK);                        //put lsb bit in carry
      tmp = (tmp >> 1) | (carry << (nb_bits-1));          //shift right and put old carry in msb bit
    }
    /* SLA (Shift left arithmetic) */
    else if((op & CB_OPERATOR_MASK) == SLA_OP){
      setCflag(tmp >> (nb_bits-1));                       //put the old msb bit to Carry flag
      tmp <<= 1;                                        //shift left
    }
    /* SRA (Shift right arithmetic) */
    else if((op & CB_OPERATOR_MASK) == SRA_OP){
      lbit = tmp >> (nb_bits-1);                          //saving msb bit for sign extention
      setCflag(tmp & BIT0_MASK);                        //put lsb to carry flag
      tmp = (tmp >> 1) | (lbit << (nb_bits-1));           //shift right and sign extend
    }
    /* SRL (Shift right logic) */
    else if((op & CB_OPERATOR_MASK) == SRL_OP){
      setCflag(tmp & BIT0_MASK);                        //put lsb to carry flag
      tmp >>= 1;                                        //shift right
    }
    /* SWAP (Swap upper and lower nibble) */
    else if((op & CB_OPERATOR_MASK) == SWAP_OP){
      tmp = (lowNibble(tmp) << 4) | highNibble(tmp);    
      setCflag(0);
    }
    setZflag(isZero(tmp));                              //set flags accordingly
    setHflag(0);
    setNflag(0);                                        //no substraction
  }
  
  
  
  /*** Test bit instruction ***/
  /* BIT (Test bit n and change Z flag accordingly) */
  else if((op & CB_BIT_MASK) == BIT_OP){
    bit_n = bitField(op);                               //get the position of the bit to be test
    tmp   = (tmp >> bit_n) | BIT0_MASK;                 //get the bit value
    setZflag(isZero(tmp));                              //set the flags accordingly
    setHflag(1); 
    setNflag(0);                                        //no substraction
  }
  
  
  
  /*** Reset bit instruction ***/
  /* RES (Reset bit n to 0) */
  else if((op & CB_RES_MASK) == RES_OP){
    bit_n = bitField(op);                               //get the position of the bit to be reset
    tmp = tmp & (0xFFFE << bit_n);                      //reset this bit to 0
  }
  
  
  
  /*** Set bit instruction ***/
  /* SET (Reset bit n to 1) */
  else if((op & CB_SET_MASK) == SET_OP){
    bit_n = bitField(op);                               //get the position of the bit to be set
    tmp = tmp | (1 << bit_n);                           //set this bit to 1
  }
  
  
  
  if(reg == M)                                          //if it's memory id M 
    setRegPair(HL, tmp);                                //then change it for HL (special for CB-prefixed instructions) and store result back in it                        
  else
    setReg(reg, tmp);                                  
}




/***Static functions***/
uint8_t GameboyCpu::regDstField (uint8_t op  ){return (op   >>   3) & 0x7;}   //return the field containing the destination reg id
uint8_t GameboyCpu::regSrcField (uint8_t op  ){return (op   &  0x7)      ;}   //return the field containing the source reg id
uint8_t GameboyCpu::regPairField(uint8_t op  ){return (op   >>   4) & 0x3;}   //return the field containing the register pair id
uint8_t GameboyCpu::bitField    (uint8_t op  ){return (op   >>   3) & 0x7;}   //return the field containing the bit position for BIT, RES and SET instructions
bool    GameboyCpu::isZero      (uint8_t val ){return (val  ==   0)      ;}   //return true if value = 0 else false
uint8_t GameboyCpu::lowNibble   (uint8_t byte){return (byte &  0xF)      ;}   //return the 4 bits lsb of byte
uint8_t GameboyCpu::highNibble  (uint8_t byte){return (byte >>   4)      ;}   //return the 4 bits msb of byte

/* return true if a Carry after a 8 bits addition has been generated */
bool GameboyCpu::isCarry (uint8_t op1, uint8_t op2, bool cin){
  return GameboyCpu::isCarryN(op1,op2,cin,8);      //see if a carry is generated at the 8th bit
}
/* return true if a Carry after a 4 bits addition has been generated */
bool GameboyCpu::isHalfCarry (uint8_t op1, uint8_t op2, bool cin){           
  return GameboyCpu::isCarryN(op1,op2,cin,4);      //see if a carry is generated at the 4th bit
}

/* Calculating the Carry at the n-1 bit position */
bool GameboyCpu::isCarryN(uint8_t op1, uint8_t op2, bool cin, uint8_t n){
  uint8_t ci = cin;                                         //current carry (first iteration ci = cin)
  uint8_t op1i, op2i;                                       //current bit i 
  uint8_t ci_next = 0;                                      //ci+1
  
  for(uint8_t i = 0; i < n; i++){                               
    op1i = (op1 >> i) & 1;                                  //isolating the bit i
    op2i = (op2 >> i) & 1;                                  
    ci_next = (op1i & op2i) | (op1i & ci) | (op2i & ci);    //calculating the next carry
    ci = ci_next;                                           //current carry becoming next carry for next iteration
  }
  return (ci == 1);
}


/* rotate 8-bits value right */
uint8_t GameboyCpu::rotateRByte(uint8_t byte){
  uint8_t rbit = byte & BIT0_MASK;              //saving the lsb bit
  return ((byte >> 1) | (rbit << 7));           //shift right and put the lsb to msb 
}

/* rotate 8-bits value left */
uint8_t GameboyCpu::rotateLByte(uint8_t byte){
  uint8_t lbit = (byte >> 7);                   //saving the msb bit
  return ((byte << 1) | lbit);                  //shift right and put the msb to lsb 
}

/* enable all interrupts */
void GameboyCpu::enableInterrupts(){
  gb_mem->writeData(0xFFFF, 0xFF);
}

/* diable all interrupts */
void GameboyCpu::disableInterrupts(){
  gb_mem->writeData(0xFFFF, 0x00);
}

/* extend the sign for signed arithmetic */
uint16_t GameboyCpu::extendSign (uint8_t byte){
  if(byte & BIT7_MASK) return 0xFF00 | byte;   //if signed byte is negative
  else return byte; 
}

















