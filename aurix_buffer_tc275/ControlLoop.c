#include "ControlLoop.h"

#include <math.h>

#include "Board_Pins.h"
#include "CCU6_PWM_Generation.h"
#include "Can_SpeedTx.h"
#include "IfxPort.h"
#include "Speed_Calculate.h"

#define CONTROL_LOOP_PERIOD_MS      5u
#define DUTY_STAGE_DURATION_MS      4000u
#define DUTY_STAGE_LOOP_COUNT       (DUTY_STAGE_DURATION_MS / CONTROL_LOOP_PERIOD_MS)
#define FIXED_BUFFER_COUNT          SPEED_BUFFER_FIXED_COUNT
#define DUTY_STAGE_COUNT            6u

typedef struct
{
    double  sumRpm;
    double  sumRpmSq;
    float32 minRpm;
    float32 maxRpm;
    uint16  count;
} RpmStats;

/*
 * 100% -> 0% -> 25% -> 50% -> 75% -> 100%
 * 각 구간은 4초 유지한다.
 */
static const uint8 g_dutySequence[DUTY_STAGE_COUNT] = {100u, 0u, 25u, 50u, 75u, 100u};

/* CAN 송신 및 구간 통계 계산에 사용하는 현재 값들 */
static volatile float32        g_currentRawRpm             = 0.0f;
static volatile float32        g_currentAvgRpm             = 0.0f;
static volatile uint32         g_currentRawTick100us       = 0u;
static volatile uint32         g_currentAvgTick100us       = 0u;
static volatile float32        g_currentStddevRpm          = 0.0f;
static volatile float32        g_currentP2pRpm             = 0.0f;
static volatile uint8          g_currentDutyPercent        = 0u;
static volatile uint16         g_currentStageIndex         = 0u;
static volatile uint16         g_currentStageLoopCount     = 0u;
static volatile MotorTaskState g_currentState              = MotorTaskState_Init;

static RpmStats g_rpmStats;

static void ControlLoop_ResetStats(void)
{
    g_rpmStats.sumRpm = 0.0;
    g_rpmStats.sumRpmSq = 0.0;
    g_rpmStats.minRpm = 999999.0f;
    g_rpmStats.maxRpm = 0.0f;
    g_rpmStats.count = 0u;

    g_currentStddevRpm = 0.0f;
    g_currentP2pRpm = 0.0f;
}

static void ControlLoop_SetDuty(uint8 dutyPercent)
{
    g_currentDutyPercent = dutyPercent;
    updatePWMDutyCycle(dutyPercent);
}

static void ControlLoop_UpdateRealtimeValues(const SpeedValue *speed)
{
    if ((speed == 0) || (speed->valid == FALSE))
    {
        return;
    }

    /* 역방향 값이 들어와도 그래프는 절대값 기준으로 보기 쉽게 맞춘다. */
    g_currentRawRpm = fabsf(speed->rawRpm);
    g_currentAvgRpm = fabsf(speed->avgRpm);
    g_currentRawTick100us = speed->rawTick100us;
    g_currentAvgTick100us = speed->avgTick100us;
}

static void ControlLoop_AddRpmToStats(float32 rpm)
{
    float32 rpmMag = fabsf(rpm);

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
    float32 meanRpm;
    float32 variance;

    if (g_rpmStats.count == 0u)
    {
        g_currentStddevRpm = 0.0f;
        g_currentP2pRpm = 0.0f;
        return;
    }

    count = (double)g_rpmStats.count;
    meanRpm = (float32)(g_rpmStats.sumRpm / count);
    variance = (float32)((g_rpmStats.sumRpmSq / count) - ((double)meanRpm * (double)meanRpm));

    if (variance < 0.0f)
    {
        variance = 0.0f;
    }

    g_currentStddevRpm = sqrtf(variance);
    g_currentP2pRpm = g_rpmStats.maxRpm - g_rpmStats.minRpm;
}

static void ControlLoop_AdvanceStage(void)
{
    g_currentStageLoopCount = 0u;
    ControlLoop_ResetStats();

    if (g_currentStageIndex < (DUTY_STAGE_COUNT - 1u))
    {
        g_currentStageIndex++;
        ControlLoop_SetDuty(g_dutySequence[g_currentStageIndex]);
    }
    else
    {
        /* 마지막 100% 구간 이후에는 100%를 계속 유지한다. */
        ControlLoop_SetDuty(g_dutySequence[DUTY_STAGE_COUNT - 1u]);
        g_currentState = MotorTaskState_HoldFinal;
    }
}

void ControlLoop_Init(void)
{
    g_currentRawRpm = 0.0f;
    g_currentAvgRpm = 0.0f;
    g_currentRawTick100us = 0u;
    g_currentAvgTick100us = 0u;
    g_currentStddevRpm = 0.0f;
    g_currentP2pRpm = 0.0f;
    g_currentDutyPercent = 0u;
    g_currentStageIndex = 0u;
    g_currentStageLoopCount = 0u;
    g_currentState = MotorTaskState_Init;

    ControlLoop_ResetStats();
    ControlLoop_SetDuty(0u);
}

void ControlLoop(void)
{
    SpeedValue speed;
    boolean speedValid = FALSE;

    if (SpeedCalc_Get(&speed) == TRUE)
    {
        speedValid = TRUE;
        ControlLoop_UpdateRealtimeValues(&speed);
    }

    switch (g_currentState)
    {
        case MotorTaskState_Init:
            g_currentStageIndex = 0u;
            g_currentStageLoopCount = 0u;
            ControlLoop_ResetStats();
            ControlLoop_SetDuty(g_dutySequence[g_currentStageIndex]);
            g_currentState = MotorTaskState_RunSequence;
            break;

        case MotorTaskState_RunSequence:
            if (speedValid == TRUE)
            {
                ControlLoop_AddRpmToStats(g_currentAvgRpm);
                ControlLoop_UpdateStats();
            }

            g_currentStageLoopCount++;
            if (g_currentStageLoopCount >= DUTY_STAGE_LOOP_COUNT)
            {
                ControlLoop_AdvanceStage();
            }
            break;

        case MotorTaskState_HoldFinal:
            if (speedValid == TRUE)
            {
                ControlLoop_AddRpmToStats(g_currentAvgRpm);
                ControlLoop_UpdateStats();
            }
            break;

        default:
            g_currentState = MotorTaskState_Init;
            break;
    }

    /*
     * CAN 송신 값은 사용자가 남기라고 한 항목만 유지한다.
     * ACTIVE/BEST BUFFER는 모두 고정 8로 보낸다.
     */
    CanSpeedTx_SendPhaseRpm(g_currentRawTick100us,
                            g_currentAvgTick100us,
                            g_currentRawRpm,
                            g_currentAvgRpm);

    CanSpeedTx_SendBufferScan((uint8)FIXED_BUFFER_COUNT,
                              (uint8)FIXED_BUFFER_COUNT,
                              g_currentStddevRpm,
                              g_currentP2pRpm,
                              g_currentDutyPercent,
                              (uint8)g_currentState);

    /* 메인 루프 주기 확인용 체크 핀 */
    IfxPort_togglePin(LOOP_CHECK_PORT, LOOP_CHECK_PIN);
}
