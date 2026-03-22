#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxPort.h"
#include "IfxStm.h"
#include "IfxSrc.h"
#include "Bsp.h"

IfxCpu_syncEvent g_cpuSyncEvent = 0;

#define CH1_PORT            &MODULE_P02
#define CH1_PIN             1

#define CH2_PORT            &MODULE_P02
#define CH2_PIN             0

#define STM_MODULE          &MODULE_STM0
#define ISR_PRIORITY_STM    40

static IfxStm_CompareConfig g_stmConfig;
static uint32 g_isrTicks;
static uint32 g_mainLoopTicks;

IFX_INTERRUPT(isrSTM, 0, ISR_PRIORITY_STM);

void initSTM(void)
{
    g_isrTicks = IfxStm_getTicksFromMicroseconds(BSP_DEFAULT_TIMER, 100);
    g_mainLoopTicks = IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, 5);

    IfxStm_initCompareConfig(&g_stmConfig);
    g_stmConfig.triggerPriority = ISR_PRIORITY_STM;
    g_stmConfig.typeOfService = IfxSrc_Tos_cpu0;
    g_stmConfig.ticks = g_isrTicks;

    IfxStm_initCompare(STM_MODULE, &g_stmConfig);
}

void isrSTM(void)
{
    IfxStm_increaseCompare(STM_MODULE, g_stmConfig.comparator, g_isrTicks);
    IfxPort_togglePin(CH1_PORT, CH1_PIN);
}

int core0_main(void)
{
    uint32 nextCh2Tick;
    uint32 nowTick;

    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    IfxPort_setPinMode(CH1_PORT, CH1_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinMode(CH2_PORT, CH2_PIN, IfxPort_Mode_outputPushPullGeneral);

    IfxPort_setPinLow(CH1_PORT, CH1_PIN);
    IfxPort_setPinLow(CH2_PORT, CH2_PIN);

    initSTM();

    nowTick = IfxStm_getLower(STM_MODULE);
    nextCh2Tick = nowTick + g_mainLoopTicks;

    while (1)
    {
        do
        {
            nowTick = IfxStm_getLower(STM_MODULE);
        } while ((sint32)(nextCh2Tick - nowTick) > 0);

        IfxPort_togglePin(CH2_PORT, CH2_PIN);
        nextCh2Tick += g_mainLoopTicks;
    }
}
