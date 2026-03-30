#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxPort.h"
#include "IfxStm.h"
#include "IfxSrc.h"
#include "IfxMultican.h"
#include "IfxMultican_Can.h"
#include "Bsp.h"

/*===========================================================
 * Global
 *==========================================================*/
IfxCpu_syncEvent g_cpuSyncEvent = 0;

/*===========================================================
 * Scope output pins
 *  P02.1 : 0.1ms interrupt toggle
 *  P02.0 : 5ms loop toggle
 *==========================================================*/
#define CH_ISR_PORT                 (&MODULE_P02)
#define CH_ISR_PIN                  1

#define CH_LOOP_PORT                (&MODULE_P02)
#define CH_LOOP_PIN                 0

#define CAN_STB_PORT                (&MODULE_P20)
#define CAN_STB_PIN                 6

/*===========================================================
 * STM configuration
 *==========================================================*/
#define STM_MODULE                  (&MODULE_STM0)
#define ISR_PRIORITY_STM            40

#define ISR_PERIOD_US               100U      /* 0.1 ms */
#define LOOP_PERIOD_MS              5U        /* 5 ms */
#define ISR_PER_LOOP                ((LOOP_PERIOD_MS * 1000U) / ISR_PERIOD_US)   /* 50 */

/*===========================================================
 * CAN configuration
 *  500 kbps
 *  TX every 5ms
 *==========================================================*/
#define CAN_BAUDRATE                500000U
#define CAN_MESSAGE_ID              0x100U
#define CAN_NODE_ID                 IfxMultican_NodeId_0

/* 필요하면 여기 핀만 바꿔 */
#define CAN_RX_PIN                  (&IfxMultican_RXD0B_P20_7_IN)
#define CAN_TX_PIN                  (&IfxMultican_TXD0_P20_8_OUT)

/*===========================================================
 * Objects
 *==========================================================*/
static IfxStm_CompareConfig g_stmConfig;
static uint32 g_isrTicks;

static IfxMultican_Can        g_canModule;
static IfxMultican_Can_Node   g_canNode;
static IfxMultican_Can_MsgObj g_canTxMsgObj;

/*===========================================================
 * Shared variables
 *==========================================================*/
static volatile uint32 g_isrCount100usTotal = 0U;   /* 누적 0.1ms interrupt count */
static volatile uint16 g_loopCount5msTotal  = 0U;   /* 누적 5ms loop count */
static volatile uint32 g_pending5msEvents   = 0U;   /* ISR이 생성한 5ms event */
static volatile uint8  g_divider100us       = 0U;   /* 0.1ms -> 5ms divider */
static volatile uint8  g_rollingCount       = 0U;   /* 4-bit rolling count */
static volatile uint32 g_canTxErrorCount    = 0U;   /* debug용 */

/*===========================================================
 * ISR declaration
 *==========================================================*/
IFX_INTERRUPT(isrSTM, 0, ISR_PRIORITY_STM);

/*===========================================================
 * Checksum
 * Byte7 = 0xFF - (Byte0~Byte6 합의 하위 8비트)
 *==========================================================*/
static uint8 calcChecksum8(const uint8 *data, uint8 length)
{
    uint32 sum = 0U;
    uint8 i;

    for (i = 0U; i < length; ++i)
    {
        sum += data[i];
    }

    return (uint8)(0xFFU - (sum & 0xFFU));
}

/*===========================================================
 * Scope pins init
 *==========================================================*/
static void initScopePins(void)
{
    IfxPort_setPinMode(CH_ISR_PORT, CH_ISR_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinMode(CH_LOOP_PORT, CH_LOOP_PIN, IfxPort_Mode_outputPushPullGeneral);

    IfxPort_setPinLow(CH_ISR_PORT, CH_ISR_PIN);
    IfxPort_setPinLow(CH_LOOP_PORT, CH_LOOP_PIN);
}

/*===========================================================
 * STM init : 0.1ms periodic interrupt
 *==========================================================*/
static void initSTM(void)
{
    g_isrTicks = IfxStm_getTicksFromMicroseconds(BSP_DEFAULT_TIMER, ISR_PERIOD_US);

    IfxStm_initCompareConfig(&g_stmConfig);
    g_stmConfig.triggerPriority = ISR_PRIORITY_STM;
    g_stmConfig.typeOfService   = IfxSrc_Tos_cpu0;
    g_stmConfig.ticks           = g_isrTicks;

    IfxStm_initCompare(STM_MODULE, &g_stmConfig);
}

/*===========================================================
 * CAN init
 *==========================================================*/
static void initCan(void)
{
    IfxMultican_Can_Config       canConfig;
    IfxMultican_Can_NodeConfig   nodeConfig;
    IfxMultican_Can_MsgObjConfig msgObjConfig;

    IfxPort_setPinMode(CAN_STB_PORT, CAN_STB_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinLow(CAN_STB_PORT, CAN_STB_PIN);

    /* CAN module init */
    IfxMultican_Can_initModuleConfig(&canConfig, &MODULE_CAN);
    IfxMultican_Can_initModule(&g_canModule, &canConfig);

    /* CAN node init */
    IfxMultican_Can_Node_initConfig(&nodeConfig, &g_canModule);
    nodeConfig.nodeId       = CAN_NODE_ID;
    nodeConfig.baudrate     = CAN_BAUDRATE;
    nodeConfig.rxPin        = CAN_RX_PIN;
    nodeConfig.rxPinMode    = IfxPort_InputMode_pullUp;
    nodeConfig.txPin        = CAN_TX_PIN;
    nodeConfig.txPinMode    = IfxPort_OutputMode_pushPull;
    nodeConfig.loopBackMode = FALSE;
    IfxMultican_Can_Node_init(&g_canNode, &nodeConfig);

    /* TX message object init */
    IfxMultican_Can_MsgObj_initConfig(&msgObjConfig, &g_canNode);
    msgObjConfig.msgObjId              = 0;
    msgObjConfig.messageId             = CAN_MESSAGE_ID;
    msgObjConfig.frame                 = IfxMultican_Frame_transmit;
    msgObjConfig.control.messageLen    = IfxMultican_DataLengthCode_8;
    msgObjConfig.control.extendedFrame = FALSE;
    IfxMultican_Can_MsgObj_init(&g_canTxMsgObj, &msgObjConfig);
}

/*===========================================================
 * CAN send
 *
 * Byte0~3 : total 0.1ms interrupt count (uint32)
 * Byte4~5 : total 5ms loop count        (uint16)
 * Byte6   : rolling count (0~15)
 * Byte7   : checksum
 *==========================================================*/
static boolean sendCanLog(uint32 isrCount100us, uint16 loopCount5ms)
{
    uint8 payload[8];
    uint32 dataLow;
    uint32 dataHigh;
    IfxMultican_Message txMsg;
    IfxMultican_Status  status;
    uint32 retry = 1000U;

    payload[0] = (uint8)(isrCount100us >> 0);
    payload[1] = (uint8)(isrCount100us >> 8);
    payload[2] = (uint8)(isrCount100us >> 16);
    payload[3] = (uint8)(isrCount100us >> 24);

    payload[4] = (uint8)(loopCount5ms >> 0);
    payload[5] = (uint8)(loopCount5ms >> 8);

    payload[6] = (uint8)(g_rollingCount & 0x0FU);
    payload[7] = calcChecksum8(payload, 7U);

    dataLow  = ((uint32)payload[3] << 24) |
               ((uint32)payload[2] << 16) |
               ((uint32)payload[1] << 8 ) |
               ((uint32)payload[0] << 0 );

    dataHigh = ((uint32)payload[7] << 24) |
               ((uint32)payload[6] << 16) |
               ((uint32)payload[5] << 8 ) |
               ((uint32)payload[4] << 0 );

    IfxMultican_Message_init(&txMsg,
                             CAN_MESSAGE_ID,
                             dataLow,
                             dataHigh,
                             IfxMultican_DataLengthCode_8);

    do
    {
        status = IfxMultican_Can_MsgObj_sendMessage(&g_canTxMsgObj, &txMsg);
    } while ((status == IfxMultican_Status_notSentBusy) && (--retry > 0U));

    if (status == IfxMultican_Status_ok)
    {
        g_rollingCount = (uint8)((g_rollingCount + 1U) & 0x0FU);
        return TRUE;
    }
    else
    {
        g_canTxErrorCount++;
        return FALSE;
    }
}

/*===========================================================
 * STM ISR : every 0.1ms
 *==========================================================*/
void isrSTM(void)
{
    /* 다음 0.1ms 시점 예약 */
    IfxStm_increaseCompare(STM_MODULE, g_stmConfig.comparator, g_isrTicks);

    /* 오실로스코프용 0.1ms 토글 */
    IfxPort_togglePin(CH_ISR_PORT, CH_ISR_PIN);

    /* 0.1ms count 누적 */
    g_isrCount100usTotal++;

    /* 50번마다 5ms event 발생 */
    g_divider100us++;
    if (g_divider100us >= ISR_PER_LOOP)
    {
        g_divider100us = 0U;
        g_pending5msEvents++;
    }
}

/*===========================================================
 * main
 *==========================================================*/
int core0_main(void)
{
    boolean previousInterruptState;
    boolean do5msTask;
    uint32 snapshotIsrCount;
    uint16 snapshotLoopCount;

    /* global interrupt enable */
    IfxCpu_enableInterrupts();

    /* watchdog disable */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    /* sync */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    /* init */
    initScopePins();
    initCan();
    initSTM();

    while (1)
    {
        do5msTask = FALSE;

        /* ISR에서 생성한 5ms event 하나 가져오기 */
        previousInterruptState = IfxCpu_disableInterrupts();
        if (g_pending5msEvents > 0U)
        {
            g_pending5msEvents--;
            do5msTask = TRUE;
        }
        IfxCpu_restoreInterrupts(previousInterruptState);

        if (do5msTask != FALSE)
        {
            /* 오실로스코프용 5ms 토글 */
            IfxPort_togglePin(CH_LOOP_PORT, CH_LOOP_PIN);

            /* 5ms loop count 증가 */
            g_loopCount5msTotal++;

            /* CAN payload용 snapshot */
            snapshotIsrCount  = g_isrCount100usTotal;
            snapshotLoopCount = g_loopCount5msTotal;

            /* 5ms마다 CAN 송신 */
            (void)sendCanLog(snapshotIsrCount, snapshotLoopCount);
        }
    }
}
