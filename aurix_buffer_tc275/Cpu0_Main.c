#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxPort.h"

#include "Board_Pins.h"
#include "CCU6_PWM_Generation.h"
#include "ERU_Interrupt.h"
#include "GPT12_Timer_Interrupt.h"
#include "Speed_Calculate.h"
#include "ControlLoop.h"
#include "Can_SpeedTx.h"


IfxCpu_syncEvent g_cpuSyncEvent = 0;


#define MAIN_LOOP_PERIOD_TICK100US   (50u)

int core0_main(void)
{
    uint32 nextTick100us;

    /* 인터럽트 및 watchdog 해제 */
    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    /* 멀티코어 시작 시점 동기화 */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    /* 디버그용 체크 핀 초기화 */
    IfxPort_setPinModeOutput(ISR_CHECK_PORT,
                             ISR_CHECK_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(LOOP_CHECK_PORT,
                             LOOP_CHECK_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinLow(ISR_CHECK_PORT, ISR_CHECK_PIN);
    IfxPort_setPinLow(LOOP_CHECK_PORT, LOOP_CHECK_PIN);

    /* 기능 초기화 */
    initCCU6();
    initMotorDirectionPins();
    SpeedCalc_Init();
    CanSpeedTx_Init();
    ControlLoop_Init();
    initPeripheralsAndERU();
    initGpt12Timer();
    startPWMGeneration();

    /*
     * 첫 실행 기준 시점 설정
     * - 현재 시점으로부터 정확히 5ms 뒤에 첫 ControlLoop()를 수행한다.
     * - 이후에는 nextTick100us를 5ms씩 누적 증가시켜 주기 경계를 유지한다.
     */
    nextTick100us = tick100us + MAIN_LOOP_PERIOD_TICK100US;

    while (1)
    {
        
         //정해진 5ms tick 경계까지 대기
        do
        {
        } while ((sint32)(tick100us - nextTick100us) < 0);

        /* 5ms 메인 루프 실행 체크용 핀 토글 */
        IfxPort_togglePin(LOOP_CHECK_PORT, LOOP_CHECK_PIN);

        /* 5ms 제어 루프 1회 수행 */
        ControlLoop();

        /* 다음 5ms 실행 시점으로 이동 */
        nextTick100us += MAIN_LOOP_PERIOD_TICK100US;
    }

    return 0;
}
