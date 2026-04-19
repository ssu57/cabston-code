#ifndef BOARD_PINS_H_
#define BOARD_PINS_H_

#include "IfxPort.h"

/*
 * Ported motor-drive mapping from the working STM project.
 *
 * Motor A
 *   PWM A   = P02.1
 *   BRAKE A = P02.7
 *   DIR A   = P10.1
 *
 * Motor B
 *   PWM B   = P10.3
 *   BRAKE B = P02.6
 *   DIR B   = P10.2
 *
 * Single-wheel encoder input (GPT12 T2 incremental interface)
 *   T2IN  = P33.7
 *   T2EUD = P33.6
 */

#define MOTOR_A_BRAKE_PORT   (&MODULE_P02)
#define MOTOR_A_BRAKE_PIN    ((uint8)7)
#define MOTOR_A_DIR_PORT     (&MODULE_P10)
#define MOTOR_A_DIR_PIN      ((uint8)1)

#define MOTOR_B_BRAKE_PORT   (&MODULE_P02)
#define MOTOR_B_BRAKE_PIN    ((uint8)6)
#define MOTOR_B_DIR_PORT     (&MODULE_P10)
#define MOTOR_B_DIR_PIN      ((uint8)2)

/* Scope check pins: moved LOOP_CHECK away from P10.3 (PWM B) */
#define ISR_CHECK_PORT      (&MODULE_P10)
#define ISR_CHECK_PIN       ((uint8)4)
#define LOOP_CHECK_PORT     (&MODULE_P00)
#define LOOP_CHECK_PIN      ((uint8)1)

#endif /* BOARD_PINS_H_ */
