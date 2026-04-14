#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxStm.h"
#include "IfxPort.h"
#include "IfxMultican.h"
#include "IfxMultican_Can.h"
#include "GPT12_Incremental_Encoder.h"

IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0;

void initMotorPwm(void);
void motorForward(void);
void setDutyAll(uint8 duty_pct);

/* =========================
 * Encoder setting
 * =========================
 * PPR = 100, 4x decoding -> 400 counts/rev
 */
#define ENC_COUNTS_PER_REV   400
#define SAMPLE_TIME_MS       10

/* =========================
 * CAN setting
 * ========================= */
#define CAN_TX_MESSAGE_ID    0x100u
#define CAN_BAUDRATE         500000u

/* =========================
 * Watch 확인용 변수
 * ========================= */
volatile sint32  g_raw  = 0;      /* 현재 raw count */
volatile sint32  g_diff = 0;      /* 이전 샘플 대비 count 변화량 */
volatile sint32  g_pos  = 0;      /* 누적 위치 */
volatile sint32  g_dir  = 0;      /* 1: 정방향, -1: 역방향, 0: 정지 */
volatile float32 g_rpm  = 0.0f;   /* 계산된 RPM */

static sint32 g_prevRaw = 0;
static uint32 g_sampleTicks = 0;
static uint32 g_nextTick = 0;

/* =========================
 * CAN 전역 객체
 * ========================= */
static IfxMultican_Can        g_multican;
static IfxMultican_Can_Node   g_canNode;
static IfxMultican_Can_MsgObj g_canTxMsgObj;
static uint8                  g_canAlive = 0u;

/* =========================
 * CAN 초기화
 * ========================= */
static void initCanTx(void)
{
    IfxMultican_Can_Config       canConfig;
    IfxMultican_Can_NodeConfig   canNodeConfig;
    IfxMultican_Can_MsgObjConfig canMsgObjConfig;

    /* CAN module init */
    IfxMultican_Can_initModuleConfig(&canConfig, &MODULE_CAN);
    IfxMultican_Can_initModule(&g_multican, &canConfig);

    /* CAN node init */
    IfxMultican_Can_Node_initConfig(&canNodeConfig, &g_multican);
    canNodeConfig.nodeId   = IfxMultican_NodeId_0;
    canNodeConfig.baudrate = CAN_BAUDRATE;

    /*
     * TC275 Lite에서 자주 쓰는 Node0 핀 예시
     * 기존에 네가 CAN 통신 성공했던 다른 핀 조합이 있으면
     * 아래 2줄만 그 핀으로 바꾸면 됨
     */
    canNodeConfig.rxPin = &IfxMultican_RXD0B_P20_7_IN;
    canNodeConfig.txPin = &IfxMultican_TXD0_P20_8_OUT;

    canNodeConfig.rxPinMode    = IfxPort_InputMode_pullUp;
    canNodeConfig.txPinMode    = IfxPort_OutputMode_pushPull;
    canNodeConfig.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;

    /* CANoe 실제 버스에서 보기 위해 loopback OFF */
    canNodeConfig.loopBackMode = FALSE;

    IfxMultican_Can_Node_init(&g_canNode, &canNodeConfig);

    /* TX message object init */
    IfxMultican_Can_MsgObj_initConfig(&canMsgObjConfig, &g_canNode);
    canMsgObjConfig.msgObjId  = 0;
    canMsgObjConfig.messageId = CAN_TX_MESSAGE_ID;
    canMsgObjConfig.frame     = IfxMultican_Frame_transmit;

    IfxMultican_Can_MsgObj_init(&g_canTxMsgObj, &canMsgObjConfig);
}

/* =========================
 * RPM CAN 송신
 * Byte0~1 : signed RPM x10
 * Byte2   : alive counter
 * ========================= */
static void sendRpmCan(float32 rpm)
{
    IfxMultican_Message txMsg;
    IfxMultican_Status  status;

    sint16 rpm_x10;
    uint16 rpm_u16;
    uint32 dataLow  = 0u;
    uint32 dataHigh = 0u;

    /* 예: 123.4 rpm -> 1234 */
    rpm_x10 = (sint16)(rpm * 10.0f);
    rpm_u16 = (uint16)rpm_x10;

    dataLow |= ((uint32)(rpm_u16 & 0x00FFu) << 0);         /* Byte0 */
    dataLow |= ((uint32)((rpm_u16 >> 8) & 0x00FFu) << 8);  /* Byte1 */
    dataLow |= ((uint32)g_canAlive << 16);                 /* Byte2 */

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

/* =========================
 * 엔코더 속도 계산
 * ========================= */
static void updateEncoderSimple(void)
{
    sint32 raw;
    sint32 diff;
    float32 sampleTimeSec;

    sampleTimeSec = ((float32)SAMPLE_TIME_MS) / 1000.0f;

    /* GPT12 내부 상태 갱신 */
    updateGpt12Encoder();

    /* 현재 raw position 읽기 */
    raw = getGpt12RawPosition();
    g_raw = raw;

    /* 이전 샘플과의 차이 */
    diff = raw - g_prevRaw;

    /* wrap-around 보정
     * raw count가 0 ~ 399 범위를 순환한다고 가정
     */
    if (diff > (ENC_COUNTS_PER_REV / 2))
    {
        diff -= ENC_COUNTS_PER_REV;
    }
    else if (diff < -(ENC_COUNTS_PER_REV / 2))
    {
        diff += ENC_COUNTS_PER_REV;
    }

    g_diff = diff;

    /* 누적 위치 */
    g_pos += diff;

    /* 방향 */
    if (diff > 0)
    {
        g_dir = 1;
    }
    else if (diff < 0)
    {
        g_dir = -1;
    }
    else
    {
        g_dir = 0;
    }

    /* RPM 계산 */
    g_rpm = ((float32)diff * 60.0f) / (((float32)ENC_COUNTS_PER_REV) * sampleTimeSec);

    /* 정지 근처 노이즈 제거용 데드밴드 */
    if ((g_rpm > -2.0f) && (g_rpm < 2.0f))
    {
        g_rpm = 0.0f;
    }

    g_prevRaw = raw;
}

int core0_main(void)
{
    IfxCpu_enableInterrupts();

    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    /* GPT12 엔코더 초기화 */
    initGpt12Encoder();

    /* CAN 초기화 */
    initCanTx();

    /* 시작 raw 값 확보 */
    updateGpt12Encoder();
    g_prevRaw = getGpt12RawPosition();
    g_raw = g_prevRaw;

    /* 10ms 주기 설정          */
    g_sampleTicks = IfxStm_getTicksFromMilliseconds(&MODULE_STM0, SAMPLE_TIME_MS);
    g_nextTick = IfxStm_getLower(&MODULE_STM0) + g_sampleTicks;

    initMotorPwm();
    motorForward();
    setDutyAll(100u);

    while (1)
    {
        if ((sint32)(IfxStm_getLower(&MODULE_STM0) - g_nextTick) >= 0)
        {
            g_nextTick += g_sampleTicks;

            updateEncoderSimple();

            /* g_rpm CAN 송신 */
            sendRpmCan(g_rpm);
        }
    }

    return 1;
}
