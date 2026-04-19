#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxPort.h"
#include "IfxSrc.h"
#include "IfxStm.h"
#include "IfxScuEru.h"
#include "IfxGtm.h"
#include "IfxGtm_Tom.h"
#include "IfxGtm_Tom_Pwm.h"
#include "IfxGtm_Cmu.h"
#include "IfxMultican_Can.h"
#include "GPT12_Timer_Interrupt.h"
#include <math.h>

IFX_ALIGN(4) IfxCpu_syncEvent cpuSyncEvent = 0;

/* =========================================================
 * Interrupt priorities
 * ========================================================= */
#define ISR_PRIORITY_ERU_LEFT_B     40
#define ISR_PRIORITY_ERU_RIGHT_B    41

/* =========================================================
 * CAN
 * ========================================================= */
#define CAN_MESSAGE_ID_SPEED        0x200

/* =========================================================
 * PWM
 * ========================================================= */
#define PWM_PERIOD_TICKS            10000u
#define PWM_RESOLUTION              100u

/* =========================================================
 * Loop
 * GPT12 100us interrupt x 50 = 5ms
 * ========================================================= */
#define MAIN_LOOP_TICK_100US        50u
#define CONTROL_LOOP_PERIOD_MS      5u

/* =========================================================
 * Encoder constants
 * 11 pulse * 4체배 * 30 감속비 = 1320
 * ========================================================= */
#define ENCODER_PPR                 11u
/* Requested encoder pins use ERU only on B channels:
 *   Left  B = P33.7  -> SCU_REQ8 / ERU available
 *   Right B = P15.4  -> SCU_REQ0 / ERU available
 *   Left  A = P33.6  -> no SCU_REQ / ERU not available
 *   Right A = P02.4  -> no SCU_REQ / ERU not available
 * Therefore this version decodes direction on B-edge interrupts only,
 * while sampling A/B levels inside the ISR. Effective edge factor is 2.
 */
#define QUADRATURE_FACTOR           2u
#define GEAR_RATIO                  30u
#define CPR                         (ENCODER_PPR * QUADRATURE_FACTOR * GEAR_RATIO)

/* =========================================================
 * Ring buffer
 * ========================================================= */
#define BUF_SIZE                    16u

/* =========================================================
 * Stop timeout
 * ========================================================= */
#define STOP_TIMEOUT_SEC            0.05f

/* =========================================================
 * Toggle pin
 * ========================================================= */
#define TOGGLE_5MS_PORT             (&MODULE_P00)
#define TOGGLE_5MS_PIN              6

/* =========================================================
 * Encoder pin map
 * Left motor encoder : A=P33.6, B=P33.7
 * Right motor encoder: A=P02.4, B=P15.4
 * ========================================================= */
#define ENC_L_A_PORT                (&MODULE_P33)
#define ENC_L_A_PIN                 6
#define ENC_L_B_PORT                (&MODULE_P33)
#define ENC_L_B_PIN                 7

#define ENC_R_A_PORT                (&MODULE_P02)
#define ENC_R_A_PIN                 4
#define ENC_R_B_PORT                (&MODULE_P15)
#define ENC_R_B_PIN                 4

/* =========================================================
 * ERU request pins (B channel only in this version)
 * ========================================================= */
#define ENC_L_B_REQPIN              (&IfxScu_REQ8_P33_7_IN)
#define ENC_R_B_REQPIN              (&IfxScu_REQ0_P15_4_IN)

/* =========================================================
 * Motor control pin map
 * ========================================================= */
#define MOTOR_A_DIR_PORT            (&MODULE_P10)
#define MOTOR_A_DIR_PIN             1

#define MOTOR_B_DIR_PORT            (&MODULE_P10)
#define MOTOR_B_DIR_PIN             2

#define MOTOR_A_BRK_PORT            (&MODULE_P02)
#define MOTOR_A_BRK_PIN             7

#define MOTOR_B_BRK_PORT            (&MODULE_P02)
#define MOTOR_B_BRK_PIN             6

/* =========================================================
 * CAN standby / enable pin
 * ========================================================= */
#define CAN_STB_PORT                (&MODULE_P20)
#define CAN_STB_PIN                 6

/* =========================================================
 * Wheel / speed
 * ========================================================= */
#define PI_F                        (3.141592654f)
#define WHEEL_DIAMETER_M            (0.06875f)
#define WHEEL_CIRCUMFERENCE_M       (PI_F * WHEEL_DIAMETER_M)

/* =========================================================
 * Step sequence
 * 0 -> 25 -> 50 -> 75 -> 90 -> 75 -> 50 -> 25 -> 0
 * ========================================================= */
#define STAGE_DURATION_MS           5000u
#define STAGE_LOOP_COUNT            (STAGE_DURATION_MS / CONTROL_LOOP_PERIOD_MS)
#define STAGE_COUNT                 9u

/* =========================================================
 * Max speed / rpm
 * 실측 max speed = 1.02 m/s
 * rpm = v * 60 / circumference
 * ========================================================= */
#define MAX_NO_LOAD_SPEED_MPS       (1.02f)
#define MAX_NO_LOAD_RPM             (283.0f)

/* =========================================================
 * PI gains from user's ControlLoop style
 * ========================================================= */
#define SPEED_CTRL_KP               (10.0f)
#define SPEED_CTRL_KI               (0.5f)
#define SPEED_CTRL_INT_LIMIT        (12.0f)
#define SPEED_ERROR_DEADBAND_RPM    (1.0f)

#define START_ASSIST_MIN_DUTY       (30.0f)
#define START_ASSIST_RPM_THRESHOLD  (15.0f)

/* =========================================================
 * GPT12 shared globals
 * ========================================================= */
volatile uint8  loop_cnt = 0;
volatile uint32 count_0_1ms = 0;

/* =========================================================
 * STM frequency
 * ========================================================= */
static float g_stmFreqHz = 0.0f;

/* =========================================================
 * PWM handles
 * ========================================================= */
static IfxGtm_Tom_Pwm_Driver g_pwmA;
static IfxGtm_Tom_Pwm_Driver g_pwmB;
static IfxGtm_Tom_Pwm_Config g_configA;
static IfxGtm_Tom_Pwm_Config g_configB;

/* =========================================================
 * CAN handles
 * ========================================================= */
static IfxMultican_Can        g_myMultican;
static IfxMultican_Can_Node   g_myMulticanNode0;
static IfxMultican_Can_MsgObj g_myMulticanTxMsgObj;
volatile IfxMultican_Status   g_canTxStatus = IfxMultican_Status_ok;

/* =========================================================
 * Debug / measured variables
 * ========================================================= */
volatile uint32 g_leftEdgeCount   = 0u;
volatile uint32 g_rightEdgeCount  = 0u;
volatile uint32 g_leftValidStepCount  = 0u;
volatile uint32 g_rightValidStepCount = 0u;

volatile sint32 g_leftEncoderCount  = 0;
volatile sint32 g_rightEncoderCount = 0;

volatile uint32 g_leftLastDtTick   = 0u;
volatile uint32 g_rightLastDtTick  = 0u;
volatile uint32 g_leftAvgDtTick    = 0u;
volatile uint32 g_rightAvgDtTick   = 0u;

volatile sint8  g_leftDir  = 0;
volatile sint8  g_rightDir = 0;

volatile float  g_leftWheelRpmF    = 0.0f;
volatile float  g_rightWheelRpmF   = 0.0f;
volatile float  g_leftWheelMps     = 0.0f;
volatile float  g_rightWheelMps    = 0.0f;
volatile float  g_vehicleRpm       = 0.0f;
volatile float  g_vehicleMps       = 0.0f;

volatile float  g_targetRpm        = 0.0f;
volatile float  g_targetMps        = 0.0f;
volatile float  g_stddevRpm        = 0.0f;
volatile float  g_p2pRpm           = 0.0f;

volatile float  g_dutyCommand      = 0.0f;
volatile float  g_integralTerm     = 0.0f;

volatile uint8  g_currentDutyPercent  = 0u;
volatile uint8  g_currentCommandPercent = 0u;
volatile uint8  g_currentStageIndex   = 0u;
volatile uint16 g_currentStageLoopCount = 0u;

/* =========================================================
 * Control state
 * ========================================================= */
typedef enum
{
    MotorTaskState_Init = 0,
    MotorTaskState_RunSequence,
    MotorTaskState_HoldFinal
} MotorTaskState;

volatile MotorTaskState g_currentState = MotorTaskState_Init;

/* =========================================================
 * Stats
 * ========================================================= */
typedef struct
{
    double  sumRpm;
    double  sumRpmSq;
    float   minRpm;
    float   maxRpm;
    uint16  count;
} RpmStats;

static RpmStats g_rpmStats;

static const uint8 g_commandPercentSequence[STAGE_COUNT] =
{
    0u, 25u, 50u, 75u, 90u, 75u, 50u, 25u, 0u
};

/* =========================================================
 * Encoder ring buffer structure
 * ========================================================= */
typedef struct
{
    uint32 dtBuf[BUF_SIZE];
    uint64 dtSum;
    uint8  bufIdx;
    uint8  bufCount;

    uint32 prevTick;
    uint32 lastEdgeTick;

    sint8  dir;
    uint8  prevState;

    volatile sint32 *encoderCount;
    volatile uint32 *edgeCount;
    volatile uint32 *validStepCount;
    volatile uint32 *lastDtTick;
    volatile uint32 *avgDtTick;
    volatile sint8  *dirDbg;
} EncoderTimeFilter;

static EncoderTimeFilter g_leftFilter;
static EncoderTimeFilter g_rightFilter;

/* =========================================================
 * Prototypes
 * ========================================================= */
static void DrivePin_Init(void);
static void EncoderPin_Init(void);
static void EncoderEru_Init(void);
static void initGtmPwm(uint16 dutyA, uint16 dutyB);

static void setDutyA(uint8 duty_pct);
static void setDutyB(uint8 duty_pct);
static void motorA_Forward(uint8 duty_pct);
static void motorB_Forward(uint8 duty_pct);
static void motorBoth_Forward(uint8 duty_pct);

static void initMyMultican(void);
static void transmitCanMessage(uint8 *data);

static void initSingleEruInput(const IfxScu_Req_In *reqPin,
                               IfxScuEru_InputChannel inputChannel,
                               IfxScuEru_ExternalInputSelection exis,
                               IfxScuEru_OutputChannel outputChannel,
                               IfxScuEru_InputNodePointer inputNodePointer);

static uint8 readLeftState(void);
static uint8 readRightState(void);
static sint8 decodeDir(uint8 prevState, uint8 currState);
static sint8 decodeDirFromBEdge(uint8 currState);
static void EncoderTimeFilter_Init(EncoderTimeFilter *rb,
                                   volatile sint32 *encoderCount,
                                   volatile uint32 *edgeCount,
                                   volatile uint32 *validStepCount,
                                   volatile uint32 *lastDtTick,
                                   volatile uint32 *avgDtTick,
                                   volatile sint8  *dirDbg);

static void updateTimeRingBuffer(EncoderTimeFilter *rb, uint32 dt);
static void handleEncoderEdge(EncoderTimeFilter *rb, uint8 currState);
static float calculateRpmFromTimeBuffer(EncoderTimeFilter *rb);

static float ControlLoop_RpmToMps(float rpm);
static float ControlLoop_ClampFloat(float value, float lower, float upper);
static uint8 ControlLoop_FloatToDutyPercent(float duty);

static void ControlLoop_ResetStats(void);
static void ControlLoop_ResetController(void);
static void ControlLoop_SetDuty(uint8 dutyPercent);
static void ControlLoop_SetStageTarget(uint8 stageIndex);
static void ControlLoop_UpdateMeasuredValues(void);
static void ControlLoop_AddRpmToStats(float rpm);
static void ControlLoop_UpdateStats(void);
static void ControlLoop_UpdateSpeedController(boolean speedValid);
static void ControlLoop_AdvanceStage(void);
static void ControlLoop_Init(void);
static void ControlLoop_Run(void);

/* =========================================================
 * ISR declarations
 * ========================================================= */
IFX_INTERRUPT(ERU_LeftB_ISR, 0, ISR_PRIORITY_ERU_LEFT_B);
IFX_INTERRUPT(ERU_RightB_ISR, 0, ISR_PRIORITY_ERU_RIGHT_B);

/* =========================================================
 * GPT12 callback
 * ========================================================= */
void App_Task_100us(void)
{
    count_0_1ms++;
    loop_cnt++;
}

/* =========================================================
 * AB state read
 * ========================================================= */
static uint8 readLeftState(void)
{
    return (uint8)(((uint8)IfxPort_getPinState(ENC_L_A_PORT, ENC_L_A_PIN) << 1) |
                   ((uint8)IfxPort_getPinState(ENC_L_B_PORT, ENC_L_B_PIN)));
}

static uint8 readRightState(void)
{
    return (uint8)(((uint8)IfxPort_getPinState(ENC_R_A_PORT, ENC_R_A_PIN) << 1) |
                   ((uint8)IfxPort_getPinState(ENC_R_B_PORT, ENC_R_B_PIN)));
}

static sint8 decodeDir(uint8 prevState, uint8 currState)
{
    uint8 transition = (uint8)((prevState << 2) | currState);

    switch (transition)
    {
        case 0x1:
        case 0x7:
        case 0xE:
        case 0x8:
            return 1;

        case 0x2:
        case 0xB:
        case 0xD:
        case 0x4:
            return -1;

        default:
            return 0;
    }
}

static sint8 decodeDirFromBEdge(uint8 currState)
{
    uint8 a = (uint8)((currState >> 1) & 0x1u);
    uint8 b = (uint8)(currState & 0x1u);

    /* B-edge only quadrature decode
     * Forward  : A != B
     * Reverse  : A == B
     */
    return (a != b) ? 1 : -1;
}

static void EncoderTimeFilter_Init(EncoderTimeFilter *rb,
                                   volatile sint32 *encoderCount,
                                   volatile uint32 *edgeCount,
                                   volatile uint32 *validStepCount,
                                   volatile uint32 *lastDtTick,
                                   volatile uint32 *avgDtTick,
                                   volatile sint8  *dirDbg)
{
    uint8 i;

    for (i = 0u; i < BUF_SIZE; i++)
    {
        rb->dtBuf[i] = 0u;
    }

    rb->dtSum        = 0ull;
    rb->bufIdx       = 0u;
    rb->bufCount     = 0u;
    rb->prevTick     = 0u;
    rb->lastEdgeTick = 0u;
    rb->dir          = 0;
    rb->prevState    = 0u;

    rb->encoderCount   = encoderCount;
    rb->edgeCount      = edgeCount;
    rb->validStepCount = validStepCount;
    rb->lastDtTick     = lastDtTick;
    rb->avgDtTick      = avgDtTick;
    rb->dirDbg         = dirDbg;

    *encoderCount   = 0;
    *edgeCount      = 0u;
    *validStepCount = 0u;
    *lastDtTick     = 0u;
    *avgDtTick      = 0u;
    *dirDbg         = 0;
}

static void updateTimeRingBuffer(EncoderTimeFilter *rb, uint32 dt)
{
    rb->dtSum -= (uint64)rb->dtBuf[rb->bufIdx];
    rb->dtBuf[rb->bufIdx] = dt;
    rb->dtSum += (uint64)dt;

    rb->bufIdx++;
    if (rb->bufIdx >= BUF_SIZE)
    {
        rb->bufIdx = 0u;
    }

    if (rb->bufCount < BUF_SIZE)
    {
        rb->bufCount++;
    }

    if (rb->bufCount > 0u)
    {
        *(rb->avgDtTick) = (uint32)(rb->dtSum / (uint64)rb->bufCount);
    }
    else
    {
        *(rb->avgDtTick) = 0u;
    }
}

static void handleEncoderEdge(EncoderTimeFilter *rb, uint8 currState)
{
    uint32 now = IfxStm_getLower(&MODULE_STM0);
    sint8 dirStep;

    (*(rb->edgeCount))++;

    if (rb->prevTick != 0u)
    {
        uint32 dt = now - rb->prevTick;
        updateTimeRingBuffer(rb, dt);
        *(rb->lastDtTick) = dt;
    }

    dirStep = decodeDirFromBEdge(currState);

    if (dirStep != 0)
    {
        rb->dir = dirStep;
        *(rb->dirDbg) = dirStep;
        *(rb->encoderCount) += dirStep;
        (*(rb->validStepCount))++;
    }

    rb->prevState = currState;
    rb->prevTick = now;
    rb->lastEdgeTick = now;
}

static float calculateRpmFromTimeBuffer(EncoderTimeFilter *rb)
{
    boolean irqState;
    uint64 dtSum;
    uint8  bufCount;
    uint32 lastEdgeTick;
    uint32 now;
    float rpm;

    irqState = IfxCpu_disableInterrupts();
    dtSum        = rb->dtSum;
    bufCount     = rb->bufCount;
    lastEdgeTick = rb->lastEdgeTick;
    IfxCpu_restoreInterrupts(irqState);

    if ((bufCount == 0u) || (dtSum == 0ull))
    {
        return 0.0f;
    }

    now = IfxStm_getLower(&MODULE_STM0);

    if ((lastEdgeTick == 0u) ||
        ((now - lastEdgeTick) > (uint32)(g_stmFreqHz * STOP_TIMEOUT_SEC)))
    {
        return 0.0f;
    }

    rpm = ((float)bufCount * 60.0f * g_stmFreqHz) / ((float)CPR * (float)dtSum);

    if (rb->dir < 0)
    {
        rpm = -rpm;
    }

    return rpm;
}

/* =========================================================
 * PI control utility
 * ========================================================= */
static float ControlLoop_RpmToMps(float rpm)
{
    float rpmMag = fabsf(rpm);
    return (rpmMag * WHEEL_CIRCUMFERENCE_M) / 60.0f;
}

static uint8 ControlLoop_FloatToDutyPercent(float duty)
{
    if (duty <= 0.0f) return 0u;
    if (duty >= 100.0f) return 100u;
    return (uint8)(duty + 0.5f);
}

static float ControlLoop_ClampFloat(float value, float lower, float upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

static void ControlLoop_ResetStats(void)
{
    g_rpmStats.sumRpm = 0.0;
    g_rpmStats.sumRpmSq = 0.0;
    g_rpmStats.minRpm = 999999.0f;
    g_rpmStats.maxRpm = 0.0f;
    g_rpmStats.count = 0u;

    g_stddevRpm = 0.0f;
    g_p2pRpm = 0.0f;
}

static void ControlLoop_ResetController(void)
{
    g_integralTerm = 0.0f;
    g_dutyCommand = 0.0f;
}

static void ControlLoop_SetDuty(uint8 dutyPercent)
{
    g_currentDutyPercent = dutyPercent;
    motorBoth_Forward(dutyPercent);
}

static void ControlLoop_SetStageTarget(uint8 stageIndex)
{
    float targetRpm;
    float targetSpeedMps;
    uint8 commandPercent;

    if (stageIndex >= STAGE_COUNT)
    {
        stageIndex = (uint8)(STAGE_COUNT - 1u);
    }

    commandPercent = g_commandPercentSequence[stageIndex];
    targetRpm = (MAX_NO_LOAD_RPM * (float)commandPercent) / 100.0f;
    targetSpeedMps = ControlLoop_RpmToMps(targetRpm);

    g_currentStageIndex = stageIndex;
    g_currentCommandPercent = commandPercent;
    g_targetRpm = targetRpm;
    g_targetMps = targetSpeedMps;
    g_currentStageLoopCount = 0u;

    ControlLoop_ResetStats();
    ControlLoop_ResetController();

    if (commandPercent == 0u)
    {
        ControlLoop_SetDuty(0u);
    }
    else
    {
        g_dutyCommand = (float)commandPercent;
        ControlLoop_SetDuty(commandPercent);
    }
}

static void ControlLoop_UpdateMeasuredValues(void)
{
    g_leftWheelRpmF  = fabsf(calculateRpmFromTimeBuffer(&g_leftFilter));
    g_rightWheelRpmF = fabsf(calculateRpmFromTimeBuffer(&g_rightFilter));

    g_leftWheelMps   = ControlLoop_RpmToMps(g_leftWheelRpmF);
    g_rightWheelMps  = ControlLoop_RpmToMps(g_rightWheelRpmF);

    g_vehicleRpm     = (g_leftWheelRpmF + g_rightWheelRpmF) * 0.5f;
    g_vehicleMps     = (g_leftWheelMps + g_rightWheelMps) * 0.5f;
}

static void ControlLoop_AddRpmToStats(float rpm)
{
    float rpmMag = fabsf(rpm);

    g_rpmStats.sumRpm += (double)rpmMag;
    g_rpmStats.sumRpmSq += (double)rpmMag * (double)rpmMag;

    if (rpmMag < g_rpmStats.minRpm)
    {
        g_rpmStats.minRpm = rpmMag;
    }

    if (rpmMag > g_rpmStats.maxRpm)
    {
        g_rpmStats.maxRpm = rpmMag;
    }

    g_rpmStats.count++;
}

static void ControlLoop_UpdateStats(void)
{
    double count;
    float meanRpm;
    float variance;

    if (g_rpmStats.count == 0u)
    {
        g_stddevRpm = 0.0f;
        g_p2pRpm = 0.0f;
        return;
    }

    count = (double)g_rpmStats.count;
    meanRpm = (float)(g_rpmStats.sumRpm / count);
    variance = (float)((g_rpmStats.sumRpmSq / count) - ((double)meanRpm * (double)meanRpm));

    if (variance < 0.0f)
    {
        variance = 0.0f;
    }

    g_stddevRpm = sqrtf(variance);
    g_p2pRpm = g_rpmStats.maxRpm - g_rpmStats.minRpm;
}

static void ControlLoop_UpdateSpeedController(boolean speedValid)
{
    float errorRpm;
    float feedforwardDuty;
    float proportionalTerm;
    float integralCandidate;
    float dutyCommand;

    if (g_currentCommandPercent == 0u)
    {
        ControlLoop_ResetController();
        ControlLoop_SetDuty(0u);
        return;
    }

    if (speedValid == FALSE)
    {
        return;
    }

    errorRpm = g_targetRpm - g_vehicleRpm;

    if (fabsf(errorRpm) < SPEED_ERROR_DEADBAND_RPM)
    {
        errorRpm = 0.0f;
    }

    feedforwardDuty = (100.0f * g_targetRpm) / MAX_NO_LOAD_RPM;
    proportionalTerm = SPEED_CTRL_KP * errorRpm;

    integralCandidate = g_integralTerm + (SPEED_CTRL_KI * errorRpm);
    integralCandidate = ControlLoop_ClampFloat(integralCandidate,
                                               -SPEED_CTRL_INT_LIMIT,
                                               SPEED_CTRL_INT_LIMIT);

    dutyCommand = feedforwardDuty + proportionalTerm + integralCandidate;

    if (((dutyCommand > 100.0f) && (errorRpm > 0.0f)) ||
        ((dutyCommand < 0.0f) && (errorRpm < 0.0f)))
    {
        dutyCommand = feedforwardDuty + proportionalTerm + g_integralTerm;
    }
    else
    {
        g_integralTerm = integralCandidate;
    }

    dutyCommand = feedforwardDuty + proportionalTerm + g_integralTerm;

    if ((g_targetRpm > 0.0f) &&
        (g_vehicleRpm < START_ASSIST_RPM_THRESHOLD) &&
        (dutyCommand < START_ASSIST_MIN_DUTY))
    {
        dutyCommand = START_ASSIST_MIN_DUTY;
    }

    dutyCommand = ControlLoop_ClampFloat(dutyCommand, 0.0f, 100.0f);
    g_dutyCommand = dutyCommand;
    ControlLoop_SetDuty(ControlLoop_FloatToDutyPercent(dutyCommand));
}

static void ControlLoop_AdvanceStage(void)
{
    if (g_currentStageIndex < (uint8)(STAGE_COUNT - 1u))
    {
        ControlLoop_SetStageTarget((uint8)(g_currentStageIndex + 1u));
    }
    else
    {
        ControlLoop_SetStageTarget((uint8)(STAGE_COUNT - 1u));
        g_currentState = MotorTaskState_HoldFinal;
    }
}

static void ControlLoop_Init(void)
{
    g_leftWheelRpmF = 0.0f;
    g_rightWheelRpmF = 0.0f;
    g_leftWheelMps = 0.0f;
    g_rightWheelMps = 0.0f;
    g_vehicleRpm = 0.0f;
    g_vehicleMps = 0.0f;

    g_targetRpm = 0.0f;
    g_targetMps = 0.0f;
    g_stddevRpm = 0.0f;
    g_p2pRpm = 0.0f;

    g_dutyCommand = 0.0f;
    g_integralTerm = 0.0f;
    g_currentDutyPercent = 0u;
    g_currentCommandPercent = 0u;
    g_currentStageIndex = 0u;
    g_currentStageLoopCount = 0u;
    g_currentState = MotorTaskState_Init;

    ControlLoop_ResetStats();
    ControlLoop_ResetController();
    ControlLoop_SetDuty(0u);
}

static void ControlLoop_Run(void)
{
    uint8 canData[8];
    boolean speedValid = TRUE;

    ControlLoop_UpdateMeasuredValues();

    switch (g_currentState)
    {
        case MotorTaskState_Init:
            ControlLoop_SetStageTarget(0u);
            g_currentState = MotorTaskState_RunSequence;
            break;

        case MotorTaskState_RunSequence:
            ControlLoop_UpdateSpeedController(speedValid);
            ControlLoop_AddRpmToStats(g_vehicleRpm);
            ControlLoop_UpdateStats();

            g_currentStageLoopCount++;
            if (g_currentStageLoopCount >= STAGE_LOOP_COUNT)
            {
                ControlLoop_AdvanceStage();
            }
            break;

        case MotorTaskState_HoldFinal:
            ControlLoop_UpdateSpeedController(speedValid);
            ControlLoop_AddRpmToStats(g_vehicleRpm);
            ControlLoop_UpdateStats();
            break;

        default:
            g_currentState = MotorTaskState_Init;
            break;
    }

    /* CAN 0x200
       Byte0~1 : Current speed [m/s x1000]
       Byte2~3 : Target speed  [m/s x1000]
       Byte4   : Duty [%]
       Byte5   : Command percent [%]
       Byte6   : State
       Byte7   : Stage index
    */
    {
        uint16 actualMpsX1000 = (uint16)(g_vehicleMps * 1000.0f + 0.5f);
        uint16 targetMpsX1000 = (uint16)(g_targetMps * 1000.0f + 0.5f);

        canData[0] = (uint8)(actualMpsX1000 & 0xFFu);
        canData[1] = (uint8)((actualMpsX1000 >> 8) & 0xFFu);
        canData[2] = (uint8)(targetMpsX1000 & 0xFFu);
        canData[3] = (uint8)((targetMpsX1000 >> 8) & 0xFFu);
        canData[4] = g_currentDutyPercent;
        canData[5] = g_currentCommandPercent;
        canData[6] = (uint8)g_currentState;
        canData[7] = g_currentStageIndex;

        transmitCanMessage(canData);
    }
}

/* =========================================================
 * ERU ISR (B channel only)
 * ========================================================= */
void ERU_LeftB_ISR(void)
{
    IfxScuEru_clearEventFlag(IfxScuEru_InputChannel_4);
    handleEncoderEdge(&g_leftFilter, readLeftState());
}

void ERU_RightB_ISR(void)
{
    IfxScuEru_clearEventFlag(IfxScuEru_InputChannel_0);
    handleEncoderEdge(&g_rightFilter, readRightState());
}

static void initSingleEruInput(const IfxScu_Req_In *reqPin,
                               IfxScuEru_InputChannel inputChannel,
                               IfxScuEru_ExternalInputSelection exis,
                               IfxScuEru_OutputChannel outputChannel,
                               IfxScuEru_InputNodePointer inputNodePointer)
{
    IfxScuEru_initReqPin(reqPin, IfxPort_InputMode_pullUp);
    IfxScuEru_selectExternalInput(inputChannel, exis);
    IfxScuEru_connectTrigger(inputChannel, inputNodePointer);
    IfxScuEru_enableRisingEdgeDetection(inputChannel);
    IfxScuEru_enableFallingEdgeDetection(inputChannel);
    IfxScuEru_enableTriggerPulse(inputChannel);
    IfxScuEru_clearEventFlag(inputChannel);
    IfxScuEru_setInterruptGatingPattern(outputChannel,
                                        IfxScuEru_InterruptGatingPattern_alwaysActive);
}

static void EncoderEru_Init(void)
{
    g_leftFilter.prevState  = readLeftState();
    g_rightFilter.prevState = readRightState();

    /* Requested pins P33.6 and P02.4 do not have SCU_REQ mapping on TC27D.
     * Therefore ERU is attached to B channels only:
     *   Left  encoder interrupt -> P33.7 (REQ8 / input channel 4)
     *   Right encoder interrupt -> P15.4 (REQ0 / input channel 0)
     * Direction is decoded from sampled A/B state inside the B-edge ISR.
     */
    initSingleEruInput(ENC_L_B_REQPIN, IfxScuEru_InputChannel_4, IfxScuEru_ExternalInputSelection_0, IfxScuEru_OutputChannel_1, IfxScuEru_InputNodePointer_1);
    initSingleEruInput(ENC_R_B_REQPIN, IfxScuEru_InputChannel_0, IfxScuEru_ExternalInputSelection_0, IfxScuEru_OutputChannel_0, IfxScuEru_InputNodePointer_0);

    IfxSrc_init(&SRC_SCUERU1, IfxSrc_Tos_cpu0, ISR_PRIORITY_ERU_LEFT_B); IfxSrc_enable(&SRC_SCUERU1);
    IfxSrc_init(&SRC_SCUERU0, IfxSrc_Tos_cpu0, ISR_PRIORITY_ERU_RIGHT_B); IfxSrc_enable(&SRC_SCUERU0);
}

static void initMyMultican(void)
{
    IfxPort_setPinMode(CAN_STB_PORT, CAN_STB_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinLow(CAN_STB_PORT, CAN_STB_PIN);

    {
        IfxMultican_Can_Config canConfig;
        IfxMultican_Can_initModuleConfig(&canConfig, &MODULE_CAN);
        IfxMultican_Can_initModule(&g_myMultican, &canConfig);
    }

    {
        IfxMultican_Can_NodeConfig canNodeConfig;
        IfxMultican_Can_Node_initConfig(&canNodeConfig, &g_myMultican);

        canNodeConfig.nodeId     = IfxMultican_NodeId_0;
        canNodeConfig.baudrate   = 500000;
        canNodeConfig.txPin      = &IfxMultican_TXD0_P20_8_OUT;
        canNodeConfig.rxPin      = &IfxMultican_RXD0B_P20_7_IN;
        canNodeConfig.txPinMode  = IfxPort_OutputMode_pushPull;
        canNodeConfig.rxPinMode  = IfxPort_InputMode_pullUp;
        canNodeConfig.pinDriver  = IfxPort_PadDriver_cmosAutomotiveSpeed1;

        IfxMultican_Can_Node_init(&g_myMulticanNode0, &canNodeConfig);
    }

    {
        IfxMultican_Can_MsgObjConfig canMsgObjConfig;
        IfxMultican_Can_MsgObj_initConfig(&canMsgObjConfig, &g_myMulticanNode0);

        canMsgObjConfig.msgObjId               = 0;
        canMsgObjConfig.messageId              = CAN_MESSAGE_ID_SPEED;
        canMsgObjConfig.acceptanceMask         = 0x7FFFFFFFUL;
        canMsgObjConfig.frame                  = IfxMultican_Frame_transmit;
        canMsgObjConfig.control.messageLen     = IfxMultican_DataLengthCode_8;
        canMsgObjConfig.control.extendedFrame  = FALSE;
        canMsgObjConfig.control.matchingId     = FALSE;

        IfxMultican_Can_MsgObj_init(&g_myMulticanTxMsgObj, &canMsgObjConfig);
    }
}

static void transmitCanMessage(uint8 *data)
{
    IfxMultican_Message msg;
    IfxMultican_Status status;
    uint32 dataLow  = ((uint32)data[3] << 24) | ((uint32)data[2] << 16) | ((uint32)data[1] << 8) | (uint32)data[0];
    uint32 dataHigh = ((uint32)data[7] << 24) | ((uint32)data[6] << 16) | ((uint32)data[5] << 8) | (uint32)data[4];

    IfxMultican_Message_init(&msg,
                             CAN_MESSAGE_ID_SPEED,
                             dataLow,
                             dataHigh,
                             IfxMultican_DataLengthCode_8);

    do
    {
        status = IfxMultican_Can_MsgObj_sendMessage(&g_myMulticanTxMsgObj, &msg);
    } while (status == IfxMultican_Status_notSentBusy);

    g_canTxStatus = status;
}

static void initGtmPwm(uint16 dutyA, uint16 dutyB)
{
    const IfxGtm_Tom_ToutMap *pwmPinA = &IfxGtm_TOM0_9_TOUT1_P02_1_OUT;
    const IfxGtm_Tom_ToutMap *pwmPinB = &IfxGtm_TOM0_3_TOUT105_P10_3_OUT;

    IfxGtm_enable(&MODULE_GTM);
    IfxGtm_Cmu_setGclkFrequency(&MODULE_GTM, 100000000.0f);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_FXCLK);

    IfxGtm_Tom_Pwm_initConfig(&g_configA, &MODULE_GTM);
    g_configA.tom = pwmPinA->tom;
    g_configA.tomChannel = pwmPinA->channel;
    g_configA.clock = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk0;
    g_configA.period = PWM_PERIOD_TICKS;
    g_configA.dutyCycle = dutyA;
    g_configA.pin.outputPin = pwmPinA;
    g_configA.pin.outputMode = IfxPort_OutputMode_pushPull;
    g_configA.pin.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    g_configA.synchronousUpdateEnabled = TRUE;
    IfxGtm_Tom_Pwm_init(&g_pwmA, &g_configA);
    IfxGtm_Tom_Pwm_start(&g_pwmA, TRUE);

    IfxGtm_Tom_Pwm_initConfig(&g_configB, &MODULE_GTM);
    g_configB.tom = pwmPinB->tom;
    g_configB.tomChannel = pwmPinB->channel;
    g_configB.clock = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk0;
    g_configB.period = PWM_PERIOD_TICKS;
    g_configB.dutyCycle = dutyB;
    g_configB.pin.outputPin = pwmPinB;
    g_configB.pin.outputMode = IfxPort_OutputMode_pushPull;
    g_configB.pin.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    g_configB.synchronousUpdateEnabled = TRUE;
    IfxGtm_Tom_Pwm_init(&g_pwmB, &g_configB);
    IfxGtm_Tom_Pwm_start(&g_pwmB, TRUE);
}

static void setDutyA(uint8 duty_pct)
{
    uint32 dutyTicks = (PWM_PERIOD_TICKS * (uint32)duty_pct) / PWM_RESOLUTION;
    IfxGtm_Tom_Ch_setCompareOneShadow(g_pwmA.tom, g_pwmA.tomChannel, dutyTicks);
    IfxGtm_Tom_Tgc_trigger(g_pwmA.tgc[0]);
}

static void setDutyB(uint8 duty_pct)
{
    uint32 dutyTicks = (PWM_PERIOD_TICKS * (uint32)duty_pct) / PWM_RESOLUTION;
    IfxGtm_Tom_Ch_setCompareOneShadow(g_pwmB.tom, g_pwmB.tomChannel, dutyTicks);
    IfxGtm_Tom_Tgc_trigger(g_pwmB.tgc[0]);
}

static void motorA_Forward(uint8 duty_pct)
{
    IfxPort_setPinLow(MOTOR_A_DIR_PORT, MOTOR_A_DIR_PIN);
    setDutyA(duty_pct);
}

static void motorB_Forward(uint8 duty_pct)
{
    IfxPort_setPinHigh(MOTOR_B_DIR_PORT, MOTOR_B_DIR_PIN);
    setDutyB(duty_pct);
}

static void motorBoth_Forward(uint8 duty_pct)
{
    motorA_Forward(duty_pct);
    motorB_Forward(duty_pct);
}

static void DrivePin_Init(void)
{
    IfxPort_setPinMode(MOTOR_A_DIR_PORT, MOTOR_A_DIR_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinMode(MOTOR_B_DIR_PORT, MOTOR_B_DIR_PIN, IfxPort_Mode_outputPushPullGeneral);

    IfxPort_setPinMode(MOTOR_A_BRK_PORT, MOTOR_A_BRK_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinMode(MOTOR_B_BRK_PORT, MOTOR_B_BRK_PIN, IfxPort_Mode_outputPushPullGeneral);

    IfxPort_setPinLow(MOTOR_A_BRK_PORT, MOTOR_A_BRK_PIN);
    IfxPort_setPinLow(MOTOR_B_BRK_PORT, MOTOR_B_BRK_PIN);

    IfxPort_setPinLow(MOTOR_A_DIR_PORT, MOTOR_A_DIR_PIN);
    IfxPort_setPinHigh(MOTOR_B_DIR_PORT, MOTOR_B_DIR_PIN);

    IfxPort_setPinMode(TOGGLE_5MS_PORT, TOGGLE_5MS_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinLow(TOGGLE_5MS_PORT, TOGGLE_5MS_PIN);

    initGtmPwm(0u, 0u);
}

static void EncoderPin_Init(void)
{
    IfxPort_setPinMode(ENC_L_A_PORT, ENC_L_A_PIN, IfxPort_Mode_inputPullUp);
    IfxPort_setPinMode(ENC_L_B_PORT, ENC_L_B_PIN, IfxPort_Mode_inputPullUp);
    IfxPort_setPinMode(ENC_R_A_PORT, ENC_R_A_PIN, IfxPort_Mode_inputPullUp);
    IfxPort_setPinMode(ENC_R_B_PORT, ENC_R_B_PIN, IfxPort_Mode_inputPullUp);

    EncoderTimeFilter_Init(&g_leftFilter, &g_leftEncoderCount, &g_leftEdgeCount, &g_leftValidStepCount, &g_leftLastDtTick, &g_leftAvgDtTick, &g_leftDir);
    EncoderTimeFilter_Init(&g_rightFilter, &g_rightEncoderCount, &g_rightEdgeCount, &g_rightValidStepCount, &g_rightLastDtTick, &g_rightAvgDtTick, &g_rightDir);
}

/* =========================================================
 * Main
 * ========================================================= */
int core0_main(void)
{
    uint32 nextTick100us;

    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    IfxCpu_emitEvent(&cpuSyncEvent);
    IfxCpu_waitEvent(&cpuSyncEvent, 1u);

    g_stmFreqHz = IfxStm_getFrequency(&MODULE_STM0);

    DrivePin_Init();
    EncoderPin_Init();
    EncoderEru_Init();
    initMyMultican();
    initGpt12Timer();
    ControlLoop_Init();

    nextTick100us = count_0_1ms + MAIN_LOOP_TICK_100US;

    while (1)
    {
        do
        {
        } while ((sint32)(count_0_1ms - nextTick100us) < 0);

        IfxPort_togglePin(TOGGLE_5MS_PORT, TOGGLE_5MS_PIN);

        ControlLoop_Run();

        nextTick100us += MAIN_LOOP_TICK_100US;
    }

    return 0;
}
