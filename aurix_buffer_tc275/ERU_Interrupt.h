#ifndef ERU_INTERRUPT_H_
#define ERU_INTERRUPT_H_

#include "Ifx_Types.h"

/*
 * Compatibility wrapper:
 * The original task project used ERU edge interrupts on P33.7 / P15.4.
 * This port replaces that path with the proven GPT12 incremental interface
 * that worked in STM_Interrupt_1_KIT_TC275_LK.
 *
 * Single-wheel encoder input:
 *   T2IN  = P33.7
 *   T2EUD = P33.6
 */

extern volatile sint32 encoderPos;

/* Debug variables visible from ADS Expressions */
extern volatile uint16 dbgT2Raw;
extern volatile sint32 dbgGpt12Delta;
extern volatile uint32 dbgEncoderPollCount;
extern volatile uint32 dbgEncoderEventCount;

void initPeripheralsAndERU(void);
void EncoderService_100us(void);

#endif /* ERU_INTERRUPT_H_ */
