#include "ERU_Interrupt.h"

#include "IfxCpu.h"
#include "IfxPort.h"
#include "IfxSrc.h"
#include "IfxScuEru.h"
#include "IfxScu_PinMap.h"

#define ISR_PRIORITY_SCUERU_INT0   40
#define ISR_PRIORITY_SCUERU_INT1   41

/* 엔코더 핀 직접 정의 */
#define ENCODER_A_PORT   (&MODULE_P33)
#define ENCODER_A_PIN    ((uint8)7)
#define ENCODER_B_PORT   (&MODULE_P15)
#define ENCODER_B_PIN    ((uint8)4)

#define ENCODER_A_REQ    (&IfxScu_REQ8_P33_7_IN)
#define ENCODER_B_REQ    (&IfxScu_REQ0_P15_4_IN)

volatile sint32 encoderPos = 0;
