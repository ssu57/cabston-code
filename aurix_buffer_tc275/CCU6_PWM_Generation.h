#ifndef CCU6_PWM_GENERATION_H_
#define CCU6_PWM_GENERATION_H_

#include "Ifx_Types.h"

/*
 * Legacy API name kept on purpose so the original task files do not need to be
 * touched. Internally this implementation uses the same GTM/TOM dual-motor
 * drive path as the verified joystick project.
 */
void initCCU6(void);
void startPWMGeneration(void);
void updatePWMDutyCycle(uint8 dutyPercent);
void initMotorDirectionPins(void);
void stopMotor(void);

#endif /* CCU6_PWM_GENERATION_H_ */
