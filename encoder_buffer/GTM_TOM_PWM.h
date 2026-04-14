#ifndef GTM_TOM_PWM_H_
#define GTM_TOM_PWM_H_

#include "Ifx_Types.h"

void initMotorPwm(void);
void motorForward(void);
void motorBackward(void);
void motorStop(void);
void setDutyAll(uint8 duty_pct);

#endif
