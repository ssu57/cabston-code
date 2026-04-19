#include "Speed_Calculate.h"

#include "IfxCpu.h"
#include "ERU_Interrupt.h"
#include "GPT12_Timer_Interrupt.h"

typedef struct
{
    float32 rpm[SPEED_BUFFER_FIXED_COUNT];
    uint32  tick100us[SPEED_BUFFER_FIXED_COUNT];
    uint16  head;
    uint16  size;
} SpeedRingBuffer;

/*
 * 최근 8개 RPM 샘플과 시간차를 보관하는 고정 길이 링버퍼
 * - 새 샘플이 들어오면 가장 오래된 값을 빼고 새 값을 더한다.
 * - 버퍼 크기는 8로 고정한다.
 */
static volatile SpeedRingBuffer g_speedBuffer;

/* 직전 위치/직전 시간은 RPM 샘플 1개를 만들기 위해 반드시 필요하다. */
static volatile sint32  g_prevEncoderPos = 0;
static volatile uint32  g_prevTick100us  = 0u;
static volatile boolean g_hasPrevSample  = FALSE;

/* 8개 평균을 빠르게 계산하기 위한 누적합 */
static double g_rpmSum = 0.0;
static uint32 g_tickSum100us = 0u;

static float32 SpeedCalc_CalcRpm(sint32 deltaCount, uint32 deltaTick100us)
{
    float32 dtSec;

    if (deltaTick100us == 0u)
    {
        return 0.0f;
    }

    dtSec = (float32)deltaTick100us * 0.0001f;
    return (60.0f * (float32)deltaCount) / ((float32)COUNTS_PER_REV * dtSec);
}

static uint16 SpeedCalc_GetNewestIndex(void)
{
    if (g_speedBuffer.head == 0u)
    {
        return (uint16)(SPEED_BUFFER_FIXED_COUNT - 1u);
    }

    return (uint16)(g_speedBuffer.head - 1u);
}

static void SpeedCalc_PushSample(float32 rpm, uint32 deltaTick100us)
{
    uint16 writeIndex = g_speedBuffer.head;

    /*
     * 버퍼가 이미 가득 차 있으면 지금 덮어쓸 자리가 가장 오래된 값이다.
     * 먼저 누적합에서 빼고, 새 값을 넣은 뒤 다시 더한다.
     */
    if (g_speedBuffer.size >= SPEED_BUFFER_FIXED_COUNT)
    {
        g_rpmSum -= (double)g_speedBuffer.rpm[writeIndex];
        g_tickSum100us -= g_speedBuffer.tick100us[writeIndex];
    }
    else
    {
        g_speedBuffer.size++;
    }

    g_speedBuffer.rpm[writeIndex] = rpm;
    g_speedBuffer.tick100us[writeIndex] = deltaTick100us;

    g_rpmSum += (double)rpm;
    g_tickSum100us += deltaTick100us;

    g_speedBuffer.head = (uint16)((writeIndex + 1u) % SPEED_BUFFER_FIXED_COUNT);
}

static boolean SpeedCalc_UpdateSample(void)
{
    boolean intState;
    sint32 encoderPosNow;
    uint32 tickNow;
    sint32 deltaCount;
    uint32 deltaTick100us;
    float32 rpm;

    intState = IfxCpu_disableInterrupts();
    encoderPosNow = encoderPos;
    tickNow = tick100us;
    IfxCpu_restoreInterrupts(intState);

    if (g_hasPrevSample == FALSE)
    {
        g_prevEncoderPos = encoderPosNow;
        g_prevTick100us = tickNow;
        g_hasPrevSample = TRUE;
        return FALSE;
    }

    if (tickNow == g_prevTick100us)
    {
        return FALSE;
    }

    deltaCount = encoderPosNow - g_prevEncoderPos;
    deltaTick100us = tickNow - g_prevTick100us;
    rpm = SpeedCalc_CalcRpm(deltaCount, deltaTick100us);

    g_prevEncoderPos = encoderPosNow;
    g_prevTick100us = tickNow;

    SpeedCalc_PushSample(rpm, deltaTick100us);
    return TRUE;
}

void SpeedCalc_Init(void)
{
    boolean intState = IfxCpu_disableInterrupts();
    uint16 i;

    g_speedBuffer.head = 0u;
    g_speedBuffer.size = 0u;
    for (i = 0u; i < SPEED_BUFFER_FIXED_COUNT; ++i)
    {
        g_speedBuffer.rpm[i] = 0.0f;
        g_speedBuffer.tick100us[i] = 0u;
    }

    g_prevEncoderPos = 0;
    g_prevTick100us = 0u;
    g_hasPrevSample = FALSE;
    g_rpmSum = 0.0;
    g_tickSum100us = 0u;

    IfxCpu_restoreInterrupts(intState);
}

boolean SpeedCalc_Get(SpeedValue *out)
{
    uint16 newestIndex;

    if (out == 0)
    {
        return FALSE;
    }

    (void)SpeedCalc_UpdateSample();

    if (g_speedBuffer.size < SPEED_BUFFER_FIXED_COUNT)
    {
        return FALSE;
    }

    newestIndex = SpeedCalc_GetNewestIndex();

    out->rawRpm = g_speedBuffer.rpm[newestIndex];
    out->avgRpm = (float32)(g_rpmSum / (double)SPEED_BUFFER_FIXED_COUNT);
    out->rawTick100us = g_speedBuffer.tick100us[newestIndex];
    out->avgTick100us = g_tickSum100us / (uint32)SPEED_BUFFER_FIXED_COUNT;
    out->valid = TRUE;

    return TRUE;
}
