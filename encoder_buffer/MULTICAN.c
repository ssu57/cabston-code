#include "MULTICAN.h"

#include "IfxPort.h"
#include "IfxMultican.h"
#include "IfxMultican_Can.h"

#define CAN_TX_MESSAGE_ID    0x100u
#define CAN_BAUDRATE         500000u

static IfxMultican_Can        g_multican;
static IfxMultican_Can_Node   g_canNode;
static IfxMultican_Can_MsgObj g_canTxMsgObj;
static uint8                  g_canAlive = 0u;

void initCanTx(void)
{
    IfxMultican_Can_Config       canConfig;
    IfxMultican_Can_NodeConfig   canNodeConfig;
    IfxMultican_Can_MsgObjConfig canMsgObjConfig;

    IfxMultican_Can_initModuleConfig(&canConfig, &MODULE_CAN);
    IfxMultican_Can_initModule(&g_multican, &canConfig);

    IfxMultican_Can_Node_initConfig(&canNodeConfig, &g_multican);
    canNodeConfig.nodeId   = IfxMultican_NodeId_0;
    canNodeConfig.baudrate = CAN_BAUDRATE;

    /* 기존 CAN 성공 핀이 다르면 아래 2줄만 교체 */
    canNodeConfig.rxPin = &IfxMultican_RXD0B_P20_7_IN;
    canNodeConfig.txPin = &IfxMultican_TXD0_P20_8_OUT;

    canNodeConfig.rxPinMode   = IfxPort_InputMode_pullUp;
    canNodeConfig.txPinMode   = IfxPort_OutputMode_pushPull;
    canNodeConfig.pinDriver   = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    canNodeConfig.loopBackMode = FALSE;

    IfxMultican_Can_Node_init(&g_canNode, &canNodeConfig);

    IfxMultican_Can_MsgObj_initConfig(&canMsgObjConfig, &g_canNode);
    canMsgObjConfig.msgObjId  = 0;
    canMsgObjConfig.messageId = CAN_TX_MESSAGE_ID;
    canMsgObjConfig.frame     = IfxMultican_Frame_transmit;

    IfxMultican_Can_MsgObj_init(&g_canTxMsgObj, &canMsgObjConfig);
}

void sendRpmCan(float32 rpm)
{
    IfxMultican_Message txMsg;
    IfxMultican_Status  status;

    sint16 rpm_x10;
    uint16 rpm_u16;
    uint32 dataLow  = 0u;
    uint32 dataHigh = 0u;

    rpm_x10 = (sint16)(rpm * 10.0f);
    rpm_u16 = (uint16)rpm_x10;

    /* Byte0~1 : signed RPM x10, Byte2 : alive */
    dataLow |= ((uint32)(rpm_u16 & 0x00FFu) << 0);
    dataLow |= ((uint32)((rpm_u16 >> 8) & 0x00FFu) << 8);
    dataLow |= ((uint32)g_canAlive << 16);

    IfxMultican_Message_init(&txMsg,
                             CAN_TX_MESSAGE_ID,
                             dataLow,
                             dataHigh,
                             IfxMultican_DataLengthCode_8);

    do
    {
        status = IfxMultican_Can_MsgObj_sendMessage(&g_canTxMsgObj, &txMsg);
    } while (status == IfxMultican_Status_notSentBusy);

    g_canAlive++;
}
