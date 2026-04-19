#ifndef CAN_SPEED_TX_H_
#define CAN_SPEED_TX_H_

#include "Ifx_Types.h"

#define MOTOR_TASK_CAN_PHASE_ID   0x100U
#define MOTOR_TASK_CAN_BUFFER_ID  0x101U

void CanSpeedTx_Init(void);
void CanSpeedTx_SendPhaseRpm(uint32 rawPhase4Tick100us,
                             uint32 avgPhase4Tick100us,
                             float32 rawRpm,
                             float32 avgRpm);
void CanSpeedTx_SendBufferScan(uint8 activeBufferCount,
                               uint8 bestBufferCount,
                               float32 stddevRpm,
                               float32 p2pRpm,
                               uint8 dutyPercent,
                               uint8 stateCode);

#endif /* CAN_SPEED_TX_H_ */
