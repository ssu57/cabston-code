#include "Can_SpeedTx.h"

#include <math.h>

#include "IfxPort.h"
#include "Multican/Can/IfxMultican_Can.h"

#define MOTOR_TASK_CAN_BAUDRATE       500000U
#define MOTOR_TASK_CAN_NODE_ID        IfxMultican_NodeId_0
#define MOTOR_TASK_CAN_RX_PIN         (&IfxMultican_RXD0B_P20_7_IN)
#define MOTOR_TASK_CAN_TX_PIN         (&IfxMultican_TXD0_P20_8_OUT)
#define MOTOR_TASK_CAN_PHASE_OBJ_ID   0u
#define MOTOR_TASK_CAN_BUFFER_OBJ_ID  1u

#define MOTOR_TASK_CAN_NEN_PORT       (&MODULE_P20)
#define MOTOR_TASK_CAN_NEN_PIN        6u
#define MOTOR_TASK_CAN_NEN_ACTIVE_LOW 1u

static IfxMultican_Can        g_canModule;
static IfxMultican_Can_Node   g_canNode;
static IfxMultican_Can_MsgObj g_txObjPhase;
static IfxMultican_Can_MsgObj g_txObjBuffer;

static void CanSpeedTx_EnableTransceiver(void)
{
    IfxPort_setPinModeOutput(MOTOR_TASK_CAN_NEN_PORT,
                             MOTOR_TASK_CAN_NEN_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);

#if (MOTOR_TASK_CAN_NEN_ACTIVE_LOW != 0)
    IfxPort_setPinLow(MOTOR_TASK_CAN_NEN_PORT, MOTOR_TASK_CAN_NEN_PIN);
#else
    IfxPort_setPinHigh(MOTOR_TASK_CAN_NEN_PORT, MOTOR_TASK_CAN_NEN_PIN);
#endif
}

static uint16 CanSpeedTx_ClampAbsFloatX10(float32 value)
{
    float32 mag = fabsf(value) * 10.0f;

    if (mag > 65535.0f)
    {
        return 65535u;
    }

    return (uint16)mag;
}

static uint16 CanSpeedTx_ClampU32ToU16(uint32 value)
{
    if (value > 65535u)
    {
        return 65535u;
    }

    return (uint16)value;
}

static uint32 CanSpeedTx_PackU32Le(const uint8 *data)
{
    return ((uint32)data[0]) |
           ((uint32)data[1] << 8) |
           ((uint32)data[2] << 16) |
           ((uint32)data[3] << 24);
}

static void CanSpeedTx_SendRaw(IfxMultican_Can_MsgObj *txObj,
                               uint32 id,
                               const uint8 data[8])
{
    IfxMultican_Message msg;
    uint32 dataLow;
    uint32 dataHigh;
    IfxMultican_Status status;

    if ((txObj == 0) || (data == 0))
    {
        return;
    }

    dataLow = CanSpeedTx_PackU32Le(&data[0]);
    dataHigh = CanSpeedTx_PackU32Le(&data[4]);

    IfxMultican_Message_init(&msg,
                             id,
                             dataLow,
                             dataHigh,
                             IfxMultican_DataLengthCode_8);

    /*
     * 메인 루프를 막지 않기 위해 긴 busy retry는 하지 않는다.
     * 이번 주기에 바쁘면 프레임 하나를 건너뛰고 다음 주기에서 다시 보낸다.
     */
    status = IfxMultican_Can_MsgObj_sendMessage(txObj, &msg);
    (void)status;
}

void CanSpeedTx_Init(void)
{
    IfxMultican_Can_Config canConfig;
    IfxMultican_Can_NodeConfig nodeConfig;
    IfxMultican_Can_MsgObjConfig msgConfig;

    CanSpeedTx_EnableTransceiver();

    IfxMultican_Can_initModuleConfig(&canConfig, &MODULE_CAN);
    IfxMultican_Can_initModule(&g_canModule, &canConfig);

    IfxMultican_Can_Node_initConfig(&nodeConfig, &g_canModule);
    nodeConfig.nodeId = MOTOR_TASK_CAN_NODE_ID;
    nodeConfig.baudrate = MOTOR_TASK_CAN_BAUDRATE;
    nodeConfig.rxPin = MOTOR_TASK_CAN_RX_PIN;
    nodeConfig.rxPinMode = IfxPort_InputMode_pullUp;
    nodeConfig.txPin = MOTOR_TASK_CAN_TX_PIN;
    nodeConfig.txPinMode = IfxPort_OutputMode_pushPull;
    IfxMultican_Can_Node_init(&g_canNode, &nodeConfig);

    IfxMultican_Can_MsgObj_initConfig(&msgConfig, &g_canNode);
    msgConfig.msgObjId = MOTOR_TASK_CAN_PHASE_OBJ_ID;
    msgConfig.messageId = MOTOR_TASK_CAN_PHASE_ID;
    msgConfig.acceptanceMask = 0x7FFFFFFFul;
    msgConfig.frame = IfxMultican_Frame_transmit;
    msgConfig.control.messageLen = IfxMultican_DataLengthCode_8;
    msgConfig.control.extendedFrame = FALSE;
    msgConfig.control.matchingId = TRUE;
    IfxMultican_Can_MsgObj_init(&g_txObjPhase, &msgConfig);

    IfxMultican_Can_MsgObj_initConfig(&msgConfig, &g_canNode);
    msgConfig.msgObjId = MOTOR_TASK_CAN_BUFFER_OBJ_ID;
    msgConfig.messageId = MOTOR_TASK_CAN_BUFFER_ID;
    msgConfig.acceptanceMask = 0x7FFFFFFFul;
    msgConfig.frame = IfxMultican_Frame_transmit;
    msgConfig.control.messageLen = IfxMultican_DataLengthCode_8;
    msgConfig.control.extendedFrame = FALSE;
    msgConfig.control.matchingId = TRUE;
    IfxMultican_Can_MsgObj_init(&g_txObjBuffer, &msgConfig);
}

void CanSpeedTx_SendPhaseRpm(uint32 rawPhase4Tick100us,
                             uint32 avgPhase4Tick100us,
                             float32 rawRpm,
                             float32 avgRpm)
{
    uint8 data[8];
    uint16 rawTick;
    uint16 avgTick;
    uint16 rawRpmX10;
    uint16 avgRpmX10;

    rawTick = CanSpeedTx_ClampU32ToU16(rawPhase4Tick100us);
    avgTick = CanSpeedTx_ClampU32ToU16(avgPhase4Tick100us);
    rawRpmX10 = CanSpeedTx_ClampAbsFloatX10(rawRpm);
    avgRpmX10 = CanSpeedTx_ClampAbsFloatX10(avgRpm);

    data[0] = (uint8)(rawTick & 0xFFu);
    data[1] = (uint8)((rawTick >> 8) & 0xFFu);
    data[2] = (uint8)(avgTick & 0xFFu);
    data[3] = (uint8)((avgTick >> 8) & 0xFFu);
    data[4] = (uint8)(rawRpmX10 & 0xFFu);
    data[5] = (uint8)((rawRpmX10 >> 8) & 0xFFu);
    data[6] = (uint8)(avgRpmX10 & 0xFFu);
    data[7] = (uint8)((avgRpmX10 >> 8) & 0xFFu);

    CanSpeedTx_SendRaw(&g_txObjPhase, MOTOR_TASK_CAN_PHASE_ID, data);
}

void CanSpeedTx_SendBufferScan(uint8 activeBufferCount,
                               uint8 bestBufferCount,
                               float32 stddevRpm,
                               float32 p2pRpm,
                               uint8 dutyPercent,
                               uint8 stateCode)
{
    uint8 data[8];
    uint16 stddevRpmX10;
    uint16 p2pRpmX10;

    stddevRpmX10 = CanSpeedTx_ClampAbsFloatX10(stddevRpm);
    p2pRpmX10 = CanSpeedTx_ClampAbsFloatX10(p2pRpm);

    data[0] = activeBufferCount;
    data[1] = bestBufferCount;
    data[2] = (uint8)(stddevRpmX10 & 0xFFu);
    data[3] = (uint8)((stddevRpmX10 >> 8) & 0xFFu);
    data[4] = (uint8)(p2pRpmX10 & 0xFFu);
    data[5] = (uint8)((p2pRpmX10 >> 8) & 0xFFu);
    data[6] = dutyPercent;
    data[7] = stateCode;

    CanSpeedTx_SendRaw(&g_txObjBuffer, MOTOR_TASK_CAN_BUFFER_ID, data);
}
