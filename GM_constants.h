#ifndef _CPU_CONSTANTS_H_
#define _CPU_CONSTANTS_H_


/*** General registers id in Opcode ***/
#define B 0b000 
#define C 0b001 
#define D 0b010
#define E 0b011
#define H 0b100
#define L 0b101
#define M 0b110
#define A 0b111         
#define F 0b1000                       //Doesn't exist in ISA

/*** Registers pair id in Opcode ***/
#define BC    0b00
#define DE    0b01
#define HL    0b10
#define AF    0b11                      //Program Counter
#define SP    0b11                      //Stack Pointer

/*** Flag bits masks ***/             
#define Z_MASK  0x80                    //Zero Flag
#define N_MASK  0x40                    //Subtract Flag
#define H_MASK  0x20                    //Half Carry Flag
#define C_MASK  0x10                    //Carry Flag  

/*** Instructions opcode ***/                             
#define CCF_OP                   0b00111111      //CCF  (Complement Carry Flag)
#define SCF_OP                   0b00110111      //SCF  (Set Carry Flag)
#define INC_REG_OP               0b00000100      //INC  (Increment (registers or memory)) 
#define DEC_REG_OP               0b00000101      //DEC  (Decrement (registers or memory))
#define CPL_OP                   0b00101111      //CPL  (Complement register A)
#define DAA_OP                   0b00100111      //DAA  (Decimal Adjust register A)
#define NOP_OP                   0b00000000      //NOP instruction
#define LD_REG_OP                0b01000000      //LD   (Load Data 8-bits registers and memory)
#define HALT_OP                  0b01110110      //HALT instruction
#define LD_A_M_OP                0b00000010      //LD   (Load Data A register from/to memory (addressed by BC or DE))
#define REG_ALU_OP               0b10000000      //Register or memory to accumulator ALU instructions
#define ADD_REG_OP               0b10000000      //ADD  (Add register or memory to accumulator)
#define ADC_REG_OP               0b10001000      //ADC  (Add register or memory with carry to accumulator)
#define SUB_REG_OP               0b10010000      //SUB  (Substract register or memory from accumulator)
#define SBC_REG_OP               0b10011000      //SBC  (Substract register or memory with borrow from accumulator)
#define AND_REG_OP               0b10100000      //AND  (Logical AND register or memory with accumulator)
#define XOR_REG_OP               0b10101000      //XOR  (Logical Exclusive-Or register or memory with accumulator
#define OR_REG_OP                0b10110000      //OR   (Logical OR register or memory with accumulator)
#define CP_REG_OP                0b10111000      //CP   (Compare register or memory with accumulator)
#define ROT_A_OP                 0b00000111      //Rotate accumulator instructions
#define RLCA_OP                  0b00000111      //RLCA (Rotate left accumulator)
#define RRCA_OP                  0b00001111      //RRCA (Rotate right accumulator)
#define RLA_OP                   0b00010111      //RLA  (Rotate accumulator left through carry)
#define RRA_OP                   0b00011111      //RRA  (Rotate accumulator right through carry)
#define PUSH_OP                  0b11000101      //PUSH (Push data onto the stack)
#define POP_OP                   0b11000001      //POP  (Pop data off the stack)
#define ADD_REG_PAIR_OP          0b00001001      //ADD  (Add register pair to HL) 
#define INC_REG_PAIR_OP          0b00000011      //INC  (Increment register pair)  
#define DEC_REG_PAIR_OP          0b00001011      //DEC  (Decrement register pair)
#define LD_SP_OP                 0b11001001      //LD   (Load SP from HL)
#define LD_I8_OP                 0b00000110      //LD   (Load 8-bits immediate to register)
#define LD_I16_OP                0b00000001      //LD   (Load 16-bits immediate to register pair)
#define IMM_ALU_OP               0b11000110      //Immediate to accumulator ALU instruction
#define ADD_IMM_OP               0b11000110      //ADD  (Add immediate to accumulator)
#define ADC_IMM_OP               0b11001110      //ADC  (Add immediate with carry to accumulator)
#define SUB_IMM_OP               0b11010110      //SUB  (Substract immediate from accumulator)
#define SBC_IMM_OP               0b11011110      //SBC  (Substract immediate with borrow from accumulator)
#define AND_IMM_OP               0b11100110      //AND  (Logical AND immediate with accumulator)
#define XOR_IMM_OP               0b11101110      //XOR  (Logical Exclusive-Or immediate with accumulator
#define OR_IMM_OP                0b11110110      //OR   (Logical OR immediate with accumulator)
#define CP_IMM_OP                0b11111110      //CP   (Compare immediate with accumulator)
#define LD_A_ADDRI_OP            0b11101010      //LD   (Load accumulator from/to memory immediate adressing)
#define LD_SP_ADDRI_OP           0b00001000      //LD   (Load SP to memory immediate adressing)
#define JUMP_I16_OP              0b11000010      //Jump adress 16-bits unsigned immediate instructions
#define JP_NZ_I16_OP             0b11000010      //JP   (Jump if Not Zero to adress 16-bits immediate)
#define JP_NC_I16_OP             0b11010010      //JP   (Jump if Not Carry to adress 16-bits immediate)
#define JP_I16_OP                0b11000011      //JP   (Jump to adress 16-bits immediate)
#define JP_Z_I16_OP              0b11001010      //JP   (Jump if Zero to adress 16-bits immediate)
#define JP_C_I16_OP              0b11011010      //JP   (Jump if Carry to adress 16-bits immediate)
#define JP_HL_OP                 0b11101001      //JP   (Jump to adress contained by HL)
#define JUMP_I8_OP               0b00000000      //Jump current adress plus 8-bits signed immediate instruction
#define JR_NZ_I8_OP              0b00100000      //JR   (Jump if Not Zero to current adress plus 8-bits signed immediate)
#define JR_NC_I8_OP              0b00110000      //JR   (Jump if Not Carry to current adress plus 8-bits signed immediate)
#define JR_I8_OP                 0b00011000      //JR   (Jump to current adress plus 8-bits signed immediate)
#define JR_Z_I8_OP               0b00101000      //JR   (Jump if Zero to current adress plus 8-bits signed immediate)
#define JR_C_I8_OP               0b00111000      //JR   (Jump if Carry to current adress plus 8-bits signed immediate)
#define STOP_OP                  0b00010000      //STOP instruction 
#define CALL_OP                  0b11000100      //Call instructions
#define CALL_NZ_OP               0b11000100      //CALL (Call if Not Zero to 16-bits immediate address)
#define CALL_Z_OP                0b11001100      //CALL (Call if Zero to 16-bits immediate address)
#define CALL_NC_OP               0b11010100      //CALL (Call if Not Carry to 16-bits immediate address)
#define CALL_C_OP                0b11011100      //CALL (Call if Carry to 16-bits immediate address)
#define CALL_A16_OP              0b11001101      //CALL (Call to 16-bits immediate address)
#define RET_NZ_OP                0b11000000      //RET  (Return if Not Zero)
#define RET_Z_OP                 0b11001000      //RET  (Return Zero)
#define RET_NC_OP                0b11010000      //RET  (Return if Not Carry)
#define RET_C_OP                 0b11011000      //RET  (Return if Carry)
#define RET_UNC_OP               0b11001001      //RET  (Return unconditionally)
#define RETI_OP                  0b11011001      //RETI (Return then enable interrupts)
#define EI_OP                    0b11111011      //EI   (Enable Interrupts) instruction
#define DI_OP                    0b11110011      //DI   (Disable Interrupts) instruction
#define RST_OP                   0b11000111      //RST instruction
#define LD_A_FF_PLUS_I8_OP       0b11100000      //LDH  (Load Data A register from/to memory address 0xFF plus 8-bits immediate)
#define LD_A_FF_PLUS_C_OP        0b11100010      //LD   (Load Data A register from/to memory address 0xFF plus C register content)
#define LD_A_M_I16_OP            0b11101010      //LD   (Load Data A register from/to memory immediate addressing)
#define LD_A16_SP_OP             0b10000000      //LD   (Load SP to memory immediate addressing)
#define LD_HL_SP_PLUS_I8_OP      0b11111000      //LD   (Load SP + signed immediate to HL)
#define LD_SP_HL_OP              0b11111001      //LD   (Load SP from HL)
#define ADD_SP_I8_OP             0b11101000      //ADD  (Add signed immediate to SP)


/*** CB-prefixed instructions opcode ***/
#define CB_RSS_OP                0b00000000      //CB-prefixed instruction (Shift, Rotate and SwAP)
#define RLC_OP                   0b00000000      //RLC  (Rotate left)
#define RRC_OP                   0b00001000      //RRC  (Rotate right)
#define RL_OP                    0b00010000      //RL   (Rotate left through carry)
#define RR_OP                    0b00011000      //RR   (Rotate right through carry)
#define SLA_OP                   0b00100000      //SLA  (Shift left arithmetic)
#define SRA_OP                   0b00101000      //SRA  (Shift right arithmetic)
#define SWAP_OP                  0b00110000      //SWAP (Swap upper and lower nibble)
#define SRL_OP                   0b00111000      //SRL  (Shift right logic)
#define BIT_OP                   0b01000000      //BIT  (Test bit n and change Z flag accordingly)
#define RES_OP                   0b10000000      //RES  (Reset bit n to 0)
#define SET_OP                   0b11000000      //SET  (Set bit n to 1)


/*** Instructions mask ***/ 
#define INCDEC_REG_MASK          0b11000111      
#define LD_REG_MASK              0b11000000     
#define LD_A_M_MASK              0b11000111      //LD (Load register from/to memory) mask
#define REG_ALU_MASK             0b11000000      //Register or memory to accumulator ALU instructions mask
#define REG_TO_A_OPERATOR_MASK   0b11111000      //mask for operator field of REG_TO_A instruction (ADD,ADC,SUB, ...)
#define ROT_A_MASK               0b11100111      //Rotate accumulator instructions mask
#define REG_PAIR_MASK            0b11001111      //Register pair instruction mask
#define LD_I8_MASK               0b11000111      //Immediate instructions mask
#define LD_I16_MASK              0b11001111      //LD (Load 8-bits immediate to register)
#define LD_A_ADDRI_MASK          0b11101111      //LD (Load accumulator from/to memory immediate adressing) mask
#define JUMP_I16_MASK            0b11100110      //Jump adress 16-bits unsigned immediate instructions mask
#define JUMP_I8_MASK             0b11000111      //Jump adress 8-bits signed immediate instructions mask 
#define CALL_MASK                0b11100110      //Call instructions mask
#define RST_EXP_FIELD_MASK       0b00111000      //RST exp field mask
#define RST_MASK                 0b11000111      //RST instruction mask
#define LD_A_FF_PLUS_I8_MASK     0b11101111      //LDH (Load Data A register from/to memory address 0xFF plus 8-bits immediate) mask
#define LD_A_FF_PLUS_C_MASK      0b11101111      //LD  (Load Data A register from/to memory address 0xFF plus C register content) mak
#define LD_A_M_I16_MASK          0b11101111      //LD (Load Data A register from/to memory immediate addressing) mask
#define IMM_ALU_MASK             0b11000111      //Immediate to accumulator ALU instruction mask
#define CB_OPERATOR_MASK         0b11111000      //CB-prefixed operator instruction (Shift, Rotate and SwAP) mask
#define CB_RSS_MASK              0b11000000      //CB-prefixed instruction (Shift, Rotate and SwAP) group mask
#define CB_BIT_MASK              0b11000000      //BIT (Test bit n) instruction mask
#define CB_RES_MASK              0b11000000      //RES (Reset bit n to 0) instruction mask
#define CB_SET_MASK              0b11000000      //SET (Set bit n to 1) instruction mask


/*** LCD constants ***/
#define NBLINES 256               //number of lines of the LCD buffer
#define NBCOLS  256               //number of columns of the LCD buffer
#define MAX_SPRITES 40            //maximum number of sprites on screen

/*** I/O Register ***/
#define LCDC_REG    0xFF40        //LCD Control register
#define SCY_REG     0xFF42        //position y of the upper left corner of the real screen (160x144) on the screen buffer (256x256)
#define SCX_REG     0xFF43        //position x of the upper left corner of the real screen (160x144) on the screen buffer (256x256)
#define WY_REG      0xFF4A        //position y of the upper left corner of the Window on the real screen ((SCX,SCY) as origin)
#define WX_REG      0xFF4B        //position x of the upper left corner of the Window on the real screen ((SCX,SCY) as origin)


/*** Adress space ***/
#define OAM_SPACE   0xFE00        //Sprite Attrib Memory (OAM) 0xFE00-FEA0

/*** Other constants ***/
#define MEMORY_SIZE 0x10000       //Size of the adress space (64KB)
#define AF_MODE     true          //used as argument for regPair methods (return AF register if reg id is 11)
#define SP_MODE     false         //used as argument for regPair methods (return SP register if reg id is 11)

/*** Bit selection ***/
#define BIT0_MASK   0b00000001
#define BIT1_MASK   0b00000010
#define BIT2_MASK   0b00000100
#define BIT3_MASK   0b00001000
#define BIT4_MASK   0b00010000
#define BIT5_MASK   0b00100000
#define BIT6_MASK   0b01000000
#define BIT7_MASK   0b10000000


#endif
