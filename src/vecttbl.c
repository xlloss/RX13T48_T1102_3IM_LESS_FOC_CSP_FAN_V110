/************************************************************************
*
* Device     : RX/RX100/RX13T
*
* File Name  : vecttbl.c
*
* Abstract   : Initialize of Vector Table and MDE register.
*
* History    : 0.40  (2019-02-12)  [Hardware Manual Revision : 0.40]
*
* NOTE       : THIS IS A TYPICAL EXAMPLE.
*
* Copyright (C) 2019 Renesas Electronics Corporation.
*
************************************************************************/

#include "vect.h"

#pragma section C FIXEDVECT

void (*const Fixed_Vectors[])(void) = {
//;0xffffffd0  Exception(Supervisor Instruction)
    Excep_SuperVisorInst,
//;0xffffffd4  Reserved
    Dummy,
//;0xffffffd8  Reserved
    Dummy,
//;0xffffffdc  Exception(Undefined Instruction)
    Excep_UndefinedInst,
//;0xffffffe0  Reserved
    Dummy,
//;0xffffffe4  Reserved
    Excep_FloatingPoint,
//;0xffffffe8  Reserved
    Dummy,
//;0xffffffec  Reserved
    Dummy,
//;0xfffffff0  Reserved
    Dummy,
//;0xfffffff4  Reserved
    Dummy,
//;0xfffffff8  NMI
    NonMaskableInterrupt,
//;0xfffffffc  RESET
//;<<VECTOR DATA START (POWER ON RESET)>>
//;Power On Reset PC
    /*(void*)*/ PowerON_Reset_PC                                                                                                                 
//;<<VECTOR DATA END (POWER ON RESET)>>
};

#pragma address id_code=0xffffffa0 // ID codes (Default)
const unsigned long id_code[4] = {
        0xffffffff,
        0xffffffff,
        0xffffffff,
        0xffffffff,
};

#pragma address _MDEreg=0xffffff80 // MDE register (Single Chip Mode)
#ifdef __BIG
	const unsigned long _MDEreg = 0xfffffff8; // big
#else
	const unsigned long _MDEreg = 0xffffffff; // little
#endif

#pragma address _OFS1reg=0xffffff88 // OFS1 register (HOCO enable)
const unsigned long _OFS1reg = 0xfffffefe;

