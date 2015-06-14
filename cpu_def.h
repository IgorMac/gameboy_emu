#ifndef CPU_DEF_H
#define CPU_DEF_H
typedef unsigned short int16;
typedef unsigned char int8;


/****General registers*****/

#define A 0 
#define F 1 
#define B 2
#define C 3
#define D 4
#define E 5
#define H 6
#define L 7

/***Flag bits***/
#define Z  0x7  //Zero Flag
#define N  0x6  //Subtract Flag
#define HF 0x5  //Half Carry Flag
#define CF 0x4  //Carry Flag  


#endif
