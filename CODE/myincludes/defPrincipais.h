#ifndef DEFPRINCIPAIS_H_
#define DEFPRINCIPAIS_H_

#include "espressif/esp_common.h"
// -----------------------------------------------------------------------------
// Basic definitions -----------------------------------------------------------
#define FCPU 80000000							//define a frequencia da CPU - 80 MHz - ALTERAR TB NA LINHA DE BAIXO!
//asm(".equ fcpu, 80000000 \n\t");				//define a frequencia para o uso em c�digo assembly

// -----------------------------------------------------------------------------
// Bit handling macro functions ------------------------------------------------
#ifndef setBit
#define setBit(reg, bit)					((reg) |= (1 << (bit)))
#endif
#ifndef clrBit
#define clrBit(reg, bit)					((reg) &= ~(1 << (bit)))
#endif
#ifndef cplBit
#define cplBit(reg, bit)					((reg) ^= (1 << (bit)))
#endif
#ifndef isBitSet
#define isBitSet(reg, bit)					(((reg) >> (bit)) & 1)
#endif
#ifndef isBitClr
#define isBitClr(reg, bit)					(!(((reg) >> (bit)) & 1))
#endif
#ifndef waitUntilBitIsSet
#define waitUntilBitIsSet(reg, bit)			do{}while(isBitClr((reg), (bit)))
#endif
#ifndef waitUntilBitIsClear
#define waitUntilBitIsClear(reg, bit)		do{}while(isBitSet((reg), (bit)))
#endif
#ifndef noOperation
#define noOperation(cycles)					__builtin_avr_delay_cycles(cycles)
#endif
#ifndef setMask
#define setMask(reg, mask, offset)			((reg) |= ((mask) << (offset)))
#endif
#ifndef clrMask
#define clrMask(reg, mask, offset)			((reg) &= ~((mask) << (offset)))
#endif
#ifndef cplMask
#define cplMask(reg, mask, offset)			((reg) ^= ((mask) << (offset)))
#endif

#define tst_bit(y,bit) 						(y&(1<<bit))	//retorna 0 ou 1 conforme leitura do bit

//From STM32F4xx.h globaldefines.h
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

// -----------------------------------------------------------------------------
// New data types --------------------------------------------------------------
typedef enum bool_t {
	FALSE = 0,
	TRUE = 1
} bool_t;

typedef enum logic_t {
	LOW = 0,
	HIGH = 1,
	OFF = 0,
	ON = 1
} logic_t;

typedef enum direction_t {
	LEFT = 0,
	RIGHT = 1,
	UP = 0,
	DOWN = 1,
	CLOCKWISE = 0,
	COUNTERCLOCKWISE = 1
} direction_t;

//#define DEBUG

#ifdef DEBUG
#define debug(fmt, ...) printf("%s: " fmt "\n", "DEBUG", ## __VA_ARGS__)
#else
#define debug(fmt, ...)
#endif

#endif /* DEFPRINCIPAIS_H_ */
