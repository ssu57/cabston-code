#ifndef CONTROL_LOOP_H_
#define CONTROL_LOOP_H_

#include "Ifx_Types.h"

typedef enum
{
    MotorTaskState_Init        = 0,
    MotorTaskState_RunSequence = 1,
    MotorTaskState_HoldFinal   = 2
} MotorTaskState;

void ControlLoop_Init(void);
void ControlLoop(void);

#endif /* CONTROL_LOOP_H_ */
