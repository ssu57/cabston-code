#include "ERU_Interrupt.h"
#include "IfxGpt12.h"
#include "IfxGpt12_PinMap.h"
#include "GPT12_Timer_Interrupt.h"

volatile sint32 encoderPos = 0;

volatile uint16 dbgT2Raw             = 0u;
volatile sint32 dbgGpt12Delta        = 0;
volatile uint32 dbgEncoderPollCount  = 0u;
volatile uint32 dbgEncoderEventCount = 0u;

static uint16 g_prevT2Count = 0u;

static void initGpt12IncrementalT2(void)
{
    IfxGpt12_enableModule(&MODULE_GPT120);

    /* Proven working single-wheel encoder pins from STM project */
    IfxGpt12_initTxInPin(&IfxGpt120_T2INB_P33_7_IN, IfxPort_InputMode_pullUp);
    IfxGpt12_initTxEudInPin(&IfxGpt120_T2EUDB_P33_6_IN, IfxPort_InputMode_pullUp);

    MODULE_GPT120.T2.U = 0u;

    IfxGpt12_T2_setMode(&MODULE_GPT120,
                        IfxGpt12_Mode_incrementalInterfaceEdgeDetection);
    IfxGpt12_T2_setIncrementalInterfaceInputMode(
        &MODULE_GPT120,
        IfxGpt12_IncrementalInterfaceInputMode_bothEdgesTxINOrTxEUD);
    IfxGpt12_T2_setDirectionSource(&MODULE_GPT120,
                                   IfxGpt12_TimerDirectionSource_external);
    IfxGpt12_T2_setTimerDirection(&MODULE_GPT120, IfxGpt12_TimerDirection_down);
    IfxGpt12_T2_setInput(&MODULE_GPT120, IfxGpt12_Input_B);
    IfxGpt12_T2_setEudInput(&MODULE_GPT120, IfxGpt12_EudInput_B);
    IfxGpt12_T2_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);

    encoderPos           = 0;
    g_prevT2Count        = MODULE_GPT120.T2.U;
    dbgT2Raw             = g_prevT2Count;
    dbgGpt12Delta        = 0;
    dbgEncoderPollCount  = 0u;
    dbgEncoderEventCount = 0u;
}

void initPeripheralsAndERU(void)
{
    initGpt12IncrementalT2();
}

void EncoderService_100us(void)
{
    uint16 curr;
    sint32 delta;
    sint32 step;

    curr  = MODULE_GPT120.T2.U;
    delta = (sint16)(curr - g_prevT2Count);
    g_prevT2Count = curr;

    dbgT2Raw            = curr;
    dbgGpt12Delta       = delta;
    dbgEncoderPollCount++;

    if (delta == 0)
    {
        return;
    }

    step = (delta > 0) ? 1 : -1;

    while (delta != 0)
    {
        encoderPos += step;
        dbgEncoderEventCount++;
        delta -= step;
    }
}
