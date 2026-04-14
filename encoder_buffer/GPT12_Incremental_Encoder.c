#include "GPT12_Incremental_Encoder.h"
#include "IfxPort.h"
#include "Ifx_Types.h"
#include "IfxGpt12.h"
#include "IfxGpt12_IncrEnc.h"

#define ISR_PRIORITY_GPT12_TIMER        6
#define ISR_PROVIDER_GPT12_TIMER        IfxSrc_Tos_cpu0

#define UPDATE_FREQ_HZ                  10000
#define SPEED_PULSE_COUNTING_RPM        1200
#define SPEED_MIN_RPM                   (0.0f)
#define SPEED_MAX_RPM                   (2000.0f)

/* 엔코더 핀 그대로 유지 */
#define ENCODER_GPT12_PIN_A             &IfxGpt120_T3INA_P02_6_IN
#define ENCODER_GPT12_PIN_B             &IfxGpt120_T3EUDA_P02_7_IN
#define ENCODER_GPT12_PIN_Z             NULL_PTR

#define ENCODER_OFFSET                  0
#define ENCODER_REVERSED                TRUE
#define ENCODER_RESOLUTION              100
#define ENCODER_UPDATE_PERIOD           (1.0f / ((float32)UPDATE_FREQ_HZ))

#define ENCODER_SPEED_MODE_THRESHOLD    (2.0f * IFX_PI * ((float32)SPEED_PULSE_COUNTING_RPM) / 60.0f)
#define ENCODER_BASE_MIN_SPEED          (((float32)SPEED_MIN_RPM / 60.0f) * 2.0f * IFX_PI)
#define ENCODER_BASE_MAX_SPEED          (((float32)SPEED_MAX_RPM / 60.0f) * 2.0f * IFX_PI)

static IfxGpt12_IncrEnc g_gpt12IncrEnc;

void initGpt12Encoder(void)
{
    IfxGpt12_IncrEnc_Config gpt12Config;

    IfxGpt12_enableModule(&MODULE_GPT120);

    IfxGpt12_setGpt1BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt1BlockPrescaler_8);
    IfxGpt12_setGpt2BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt2BlockPrescaler_4);

    IfxGpt12_IncrEnc_initConfig(&gpt12Config, &MODULE_GPT120);

    gpt12Config.base.offset             = ENCODER_OFFSET;
    gpt12Config.base.reversed           = ENCODER_REVERSED;
    gpt12Config.base.resolution         = ENCODER_RESOLUTION;
    gpt12Config.base.resolutionFactor   = IfxStdIf_Pos_ResolutionFactor_fourFold;

    gpt12Config.base.speedModeThreshold = ENCODER_SPEED_MODE_THRESHOLD;
    gpt12Config.base.minSpeed           = ENCODER_BASE_MIN_SPEED;
    gpt12Config.base.maxSpeed           = ENCODER_BASE_MAX_SPEED;
    gpt12Config.base.speedFilterEnabled = TRUE;
    gpt12Config.base.speedFilerCutOffFrequency = 1000;
    gpt12Config.base.updatePeriod       = ENCODER_UPDATE_PERIOD;

    gpt12Config.zeroIsrPriority         = 0;
    gpt12Config.zeroIsrProvider         = ISR_PROVIDER_GPT12_TIMER;

    gpt12Config.pinA                    = ENCODER_GPT12_PIN_A;
    gpt12Config.pinB                    = ENCODER_GPT12_PIN_B;
    gpt12Config.pinZ                    = ENCODER_GPT12_PIN_Z;
    gpt12Config.pinDriver               = IfxPort_PadDriver_cmosAutomotiveSpeed3;

    IfxGpt12_IncrEnc_init(&g_gpt12IncrEnc, &gpt12Config);

    {
        sint32 resolution = g_gpt12IncrEnc.resolution - 1;
        IfxGpt12_T2_setMode(&MODULE_GPT120, IfxGpt12_Mode_reload);
        IfxGpt12_T2_setTimerValue(&MODULE_GPT120, (uint16)resolution);
        IfxGpt12_T2_setReloadInputMode(&MODULE_GPT120, IfxGpt12_ReloadInputMode_bothEdgesTxOTL);
    }
}

void updateGpt12Encoder(void)
{
    IfxGpt12_IncrEnc_update(&g_gpt12IncrEnc);
}

float32 getGpt12SpeedRadS(void)
{
    return IfxGpt12_IncrEnc_getSpeed(&g_gpt12IncrEnc);
}

float32 getGpt12SpeedRpm(void)
{
    float32 speedRadS = IfxGpt12_IncrEnc_getSpeed(&g_gpt12IncrEnc);
    return (speedRadS * 60.0f) / (2.0f * IFX_PI);
}

sint32 getGpt12RawPosition(void)
{
    return IfxGpt12_IncrEnc_getRawPosition(&g_gpt12IncrEnc);
}

sint32 getGpt12Direction(void)
{
    return (sint32)IfxGpt12_IncrEnc_getDirection(&g_gpt12IncrEnc);
}
