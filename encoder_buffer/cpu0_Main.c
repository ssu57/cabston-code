#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxStm.h"

#include "GPT12_Incremental_Encoder.h"
#include "GTM_TOM_PWM.h"
#include "MULTICAN.h"

IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0;

/* =========================
 * Encoder setting
 * =========================
 * PPR = 100, 4x decoding -> 400 counts/rev
 */
#define ENC_COUNTS_PER_REV      400
#define SAMPLE_TIME_MS          10
#define DUTY_TEST_PERCENT       100u

/* =========================
 * Ring buffer / search setting
 * ========================= */
#define RING_BUF_SIZE           64u
#define BUF_STEP                4u
#define MIN_BUF_COUNT           4u
#define MAX_BUF_COUNT           64u

/* 평균 RPM 변화량 기준 */
#define RPM_VARIATION_THRESHOLD 5.0f
#define MIN_EVAL_COUNT          10u

/* 100% duty 인가 후 안정화 */
#define SETTLE_TIME_MS          1000u

/* =========================
 * Watch 변수
 * ========================= */
volatile sint32  g_raw  = 0;      /* 현재 raw count */
volatile sint32  g_diff = 0;      /* 이전 샘플 대비 count 변화량 */
volatile sint32  g_pos  = 0;      /* 누적 위치 */
volatile sint32  g_dir  = 0;      /* 1: 정방향, -1: 역방향, 0: 정지 */
volatile float32 g_rpm  = 0.0f;   /* 최종 선택 buffer 기준 RPM */

volatile uint32  g_bestBufferCount  = 0u;
volatile float32 g_bestAvgVariation = 0.0f;
volatile uint8   g_bufferSearchDone = 0u;

static sint32  g_prevRaw = 0;
static uint32  g_sampleTicks = 0;
static uint32  g_nextTick = 0;
static float32 g_stmFreqHz = 0.0f;

/* =========================
 * 링 버퍼 샘플
 * ========================= */
typedef struct
{
    uint32 tick;   /* 실제 STM clock tick */
    sint32 pos;    /* 누적 encoder position */
} EncoderSample;

static EncoderSample g_ringBuf[RING_BUF_SIZE];

/*
 * head = 다음에 쓸 위치
 * tail = 가장 오래된 데이터 위치
 * used = 현재 사용 중인 샘플 개수
 */
static uint32 g_head = 0u;
static uint32 g_tail = 0u;
static uint32 g_used = 0u;

/* =========================
 * 후보 buffer 평가용
 * ========================= */
#define BUF_CANDIDATE_COUNT   (RING_BUF_SIZE / BUF_STEP)

static float32 g_prevRpmByBuf[BUF_CANDIDATE_COUNT];
static float32 g_sumVariationByBuf[BUF_CANDIDATE_COUNT];
static uint32  g_evalCountByBuf[BUF_CANDIDATE_COUNT];
static uint8   g_prevValidByBuf[BUF_CANDIDATE_COUNT];

static float32 absf32(float32 x)
{
    return (x < 0.0f) ? -x : x;
}

static void resetBufferSearchState(void)
{
    uint32 i;

    g_head = 0u;
    g_tail = 0u;
    g_used = 0u;

    g_bestBufferCount  = 0u;
    g_bestAvgVariation = 0.0f;
    g_bufferSearchDone = 0u;

    for (i = 0u; i < BUF_CANDIDATE_COUNT; i++)
    {
        g_prevRpmByBuf[i]      = 0.0f;
        g_sumVariationByBuf[i] = 0.0f;
        g_evalCountByBuf[i]    = 0u;
        g_prevValidByBuf[i]    = 0u;
    }
}

/* full 이후 초기화하지 않고 계속 덮어쓰기 */
static void pushRingSample(uint32 tick, sint32 pos)
{
    g_ringBuf[g_head].tick = tick;
    g_ringBuf[g_head].pos  = pos;

    g_head++;
    if (g_head >= RING_BUF_SIZE)
    {
        g_head = 0u;
    }

    if (g_used < RING_BUF_SIZE)
    {
        g_used++;
    }
    else
    {
        g_tail++;
        if (g_tail >= RING_BUF_SIZE)
        {
            g_tail = 0u;
        }
    }
}

/* age = 0 이면 newest */
static uint8 getSampleByAge(uint32 age, EncoderSample *sample)
{
    sint32 index;

    if (g_used == 0u)
    {
        return 0u;
    }

    if (age >= g_used)
    {
        return 0u;
    }

    index = (sint32)g_head - 1 - (sint32)age;
    while (index < 0)
    {
        index += (sint32)RING_BUF_SIZE;
    }

    *sample = g_ringBuf[(uint32)index];
    return 1u;
}

/*
 * newest / oldest 의 pos 차이, tick 차이로 RPM 계산
 */
static uint8 calcRpmForBuffer(uint32 bufferCount, float32 *rpm)
{
    EncoderSample newest;
    EncoderSample oldest;
    sint32 deltaCount;
    uint32 deltaTick;
    float32 deltaTime;

    if (bufferCount < 2u)
    {
        return 0u;
    }

    if (g_used < bufferCount)
    {
        return 0u;
    }

    if (getSampleByAge(0u, &newest) == 0u)
    {
        return 0u;
    }

    if (getSampleByAge(bufferCount - 1u, &oldest) == 0u)
    {
        return 0u;
    }

    deltaCount = newest.pos - oldest.pos;
    deltaTick  = newest.tick - oldest.tick;

    if (deltaTick == 0u)
    {
        return 0u;
    }

    deltaTime = ((float32)deltaTick) / g_stmFreqHz;

    *rpm = 60.0f * (float32)deltaCount /
           (((float32)ENC_COUNTS_PER_REV) * deltaTime);

    return 1u;
}

/*
 * 4, 8, 12, ... 식으로 buffer count 증가
 * 평균 RPM 변화량이 threshold 이하가 되는
 * 가장 작은 buffer를 선택
 */
static void evaluateBufferCandidates(void)
{
    uint32 bufCount;
    uint32 idx;
    float32 rpmNow;
    float32 variation;
    float32 avgVariation;

    float32 minVariation = 999999.0f;
    uint32  minVariationBuf = 0u;
    uint8   foundAcceptable = 0u;

    for (bufCount = MIN_BUF_COUNT; bufCount <= MAX_BUF_COUNT; bufCount += BUF_STEP)
    {
        idx = (bufCount / BUF_STEP) - 1u;

        if (calcRpmForBuffer(bufCount, &rpmNow) == 0u)
        {
            continue;
        }

        if (g_prevValidByBuf[idx] != 0u)
        {
            variation = absf32(rpmNow - g_prevRpmByBuf[idx]);
            g_sumVariationByBuf[idx] += variation;
            g_evalCountByBuf[idx]++;
        }

        g_prevRpmByBuf[idx]  = rpmNow;
        g_prevValidByBuf[idx] = 1u;
    }

    for (bufCount = MIN_BUF_COUNT; bufCount <= MAX_BUF_COUNT; bufCount += BUF_STEP)
    {
        idx = (bufCount / BUF_STEP) - 1u;

        if (g_evalCountByBuf[idx] < MIN_EVAL_COUNT)
        {
            continue;
        }

        avgVariation = g_sumVariationByBuf[idx] / (float32)g_evalCountByBuf[idx];

        if (avgVariation < minVariation)
        {
            minVariation = avgVariation;
            minVariationBuf = bufCount;
        }

        if ((foundAcceptable == 0u) && (avgVariation <= RPM_VARIATION_THRESHOLD))
        {
            g_bestBufferCount  = bufCount;
            g_bestAvgVariation = avgVariation;
            g_bufferSearchDone = 1u;
            foundAcceptable = 1u;
        }
    }

    /* threshold 이하가 없으면 현재까지 가장 작은 변화량 buffer 사용 */
    if ((foundAcceptable == 0u) && (minVariationBuf != 0u))
    {
        g_bestBufferCount  = minVariationBuf;
        g_bestAvgVariation = minVariation;
    }

    if (g_bestBufferCount != 0u)
    {
        float32 rpmSelected;

        if (calcRpmForBuffer(g_bestBufferCount, &rpmSelected) != 0u)
        {
            g_rpm = rpmSelected;
        }
    }
}

static void updateEncoderAndRingBuffer(uint32 nowTick)
{
    sint32 raw;
    sint32 diff;

    updateGpt12Encoder();

    raw = getGpt12RawPosition();
    g_raw = raw;

    diff = raw - g_prevRaw;

    /* wrap-around 보정 */
    if (diff > (ENC_COUNTS_PER_REV / 2))
    {
        diff -= ENC_COUNTS_PER_REV;
    }
    else if (diff < -(ENC_COUNTS_PER_REV / 2))
    {
        diff += ENC_COUNTS_PER_REV;
    }

    g_diff = diff;
    g_pos += diff;

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

    g_prevRaw = raw;

    /* sample 번호가 아니라 실제 clock tick 저장 */
    pushRingSample(nowTick, g_pos);

    if (g_bufferSearchDone == 0u)
    {
        evaluateBufferCandidates();
    }
    else
    {
        float32 rpmSelected;
        if (calcRpmForBuffer(g_bestBufferCount, &rpmSelected) != 0u)
        {
            g_rpm = rpmSelected;
        }
    }
}

int core0_main(void)
{
    uint32 settleTicks;
    uint32 settleStart;
    uint32 nowTick;

    IfxCpu_enableInterrupts();

    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    /* PWM / 엔코더 / CAN 초기화 */
    initMotorPwm();
    initGpt12Encoder();
    initCanTx();

    g_stmFreqHz = (float32)IfxStm_getFrequency(&MODULE_STM0);

    /* 듀티 100% 전진 */
    motorForward();
    setDutyAll(DUTY_TEST_PERCENT);

    /* 모터 안정화 */
    settleTicks = IfxStm_getTicksFromMilliseconds(&MODULE_STM0, SETTLE_TIME_MS);
    settleStart = IfxStm_getLower(&MODULE_STM0);

    while ((sint32)(IfxStm_getLower(&MODULE_STM0) - (settleStart + settleTicks)) < 0)
    {
    }

    /* 초기 raw 확보 */
    updateGpt12Encoder();
    g_prevRaw = getGpt12RawPosition();
    g_raw     = g_prevRaw;

    g_pos  = 0;
    g_diff = 0;
    g_dir  = 0;
    g_rpm  = 0.0f;

    resetBufferSearchState();

    /* 10ms 주기 */
    g_sampleTicks = IfxStm_getTicksFromMilliseconds(&MODULE_STM0, SAMPLE_TIME_MS);
    g_nextTick = IfxStm_getLower(&MODULE_STM0) + g_sampleTicks;

    while (1)
    {
        if ((sint32)(IfxStm_getLower(&MODULE_STM0) - g_nextTick) >= 0)
        {
            g_nextTick += g_sampleTicks;
            nowTick = IfxStm_getLower(&MODULE_STM0);

            updateEncoderAndRingBuffer(nowTick);

            /* CAN으로 현재 RPM 송신 */
            sendRpmCan(g_rpm);
        }
    }

    return 1;
}
