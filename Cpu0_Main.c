#include "Ifx_Types.h"          /* uint32, sint32, boolean 등 기본 타입 정의 */
#include "IfxCpu.h"             /* CPU 제어 및 CPU sync event 정의 */
#include "IfxScuWdt.h"          /* CPU/Safety watchdog 제어 함수 */
#include "IfxPort.h"            /* GPIO 핀 설정 및 출력 제어 함수 */
#include "IfxStm.h"             /* STM(System Timer) 관련 함수 */
#include "IfxSrc.h"             /* 인터럽트 서비스 요청(Source) 관련 정의 */
#include "Bsp.h"                /* BSP_DEFAULT_TIMER 등 보드 지원 매크로 */

/* 멀티코어 예제 구조에서 사용하는 CPU 동기화용 전역 변수 */
IfxCpu_syncEvent g_cpuSyncEvent = 0;

/* 채널 1은 P02.1 핀을 사용 */
#define CH1_PORT            &MODULE_P02
#define CH1_PIN             1

/* 채널 2는 P02.0 핀을 사용 */
#define CH2_PORT            &MODULE_P02
#define CH2_PIN             0

/* 사용할 시스템 타이머 모듈은 STM0 */
#define STM_MODULE          &MODULE_STM0

/* STM 인터럽트 우선순위 */
#define ISR_PRIORITY_STM    40

/* STM compare 설정값을 저장할 구조체 */
static IfxStm_CompareConfig g_stmConfig;

/* CH1용 0.1ms 인터럽트 주기 tick 값 */
static uint32 g_isrTicks;

/* CH2용 5ms main loop 주기 tick 값 */
static uint32 g_mainLoopTicks;

/* STM 인터럽트 함수 선언: CPU0에서 우선순위 40으로 실행 */
IFX_INTERRUPT(isrSTM, 0, ISR_PRIORITY_STM);

/* STM 초기화 함수 */
void initSTM(void)
{
    /* 0.1ms = 100us 를 STM tick 값으로 변환 */
    g_isrTicks = IfxStm_getTicksFromMicroseconds(BSP_DEFAULT_TIMER, 100);

    /* 5ms 를 STM tick 값으로 변환 */
    g_mainLoopTicks = IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, 5);

    /* compare 설정 구조체를 기본값으로 초기화 */
    IfxStm_initCompareConfig(&g_stmConfig);

    /* 인터럽트 우선순위를 40으로 설정 */
    g_stmConfig.triggerPriority = ISR_PRIORITY_STM;

    /* 인터럽트를 CPU0에서 서비스하도록 설정 */
    g_stmConfig.typeOfService = IfxSrc_Tos_cpu0;

    /* compare 주기를 0.1ms tick 값으로 설정 */
    g_stmConfig.ticks = g_isrTicks;

    /* STM0 compare 인터럽트 설정 적용 */
    IfxStm_initCompare(STM_MODULE, &g_stmConfig);
}

/* STM 인터럽트 서비스 루틴 */
void isrSTM(void)
{
    /* 다음 0.1ms 시점에 다시 인터럽트가 발생하도록 compare 값을 증가 */
    IfxStm_increaseCompare(STM_MODULE, g_stmConfig.comparator, g_isrTicks);

    /* 채널 1(P02.1)을 0.1ms마다 토글 */
    IfxPort_togglePin(CH1_PORT, CH1_PIN);
}

/* CPU0 메인 함수 */
int core0_main(void)
{
    /* CH2 다음 토글 시점을 저장할 변수 */
    uint32 nextCh2Tick;

    /* 현재 STM tick 값을 저장할 변수 */
    uint32 nowTick;

    /* 전역 인터럽트 허용 */
    IfxCpu_enableInterrupts();

    /* CPU watchdog 비활성화 */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());

    /* Safety watchdog 비활성화 */
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    /* CPU 동기화 이벤트 발생 */
    IfxCpu_emitEvent(&g_cpuSyncEvent);

    /* 다른 CPU와 동기화될 때까지 대기 */
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    /* CH1 핀(P02.1)을 일반 출력(push-pull)으로 설정 */
    IfxPort_setPinMode(CH1_PORT, CH1_PIN, IfxPort_Mode_outputPushPullGeneral);

    /* CH2 핀(P02.0)을 일반 출력(push-pull)으로 설정 */
    IfxPort_setPinMode(CH2_PORT, CH2_PIN, IfxPort_Mode_outputPushPullGeneral);

    /* CH1 초기 출력값을 Low(0)로 설정 */
    IfxPort_setPinLow(CH1_PORT, CH1_PIN);

    /* CH2 초기 출력값을 Low(0)로 설정 */
    IfxPort_setPinLow(CH2_PORT, CH2_PIN);

    /* STM 인터럽트 및 tick 값 초기화 */
    initSTM();

    /* 현재 시간을 읽어서 */
    nowTick = IfxStm_getLower(STM_MODULE);

    /* CH2의 첫 번째 실행 시점을 현재 시간 + 5ms로 설정 */
    nextCh2Tick = nowTick + g_mainLoopTicks;

    /* 무한 루프 */
    while (1)
    {
        /* 다음 5ms 시점이 올 때까지 반복해서 현재 시간을 읽음 */
        do
        {
            /* 현재 STM tick 값을 읽음 */
            nowTick = IfxStm_getLower(STM_MODULE);

        /* 아직 nextCh2Tick 시점이 안 되었으면 계속 반복 */
        } while ((sint32)(nextCh2Tick - nowTick) > 0);

        /* 채널 2(P02.0)를 5ms마다 토글 */
        IfxPort_togglePin(CH2_PORT, CH2_PIN);

        /* 다음 CH2 실행 시점을 다시 5ms 뒤로 예약 */
        nextCh2Tick += g_mainLoopTicks;
    }
}
