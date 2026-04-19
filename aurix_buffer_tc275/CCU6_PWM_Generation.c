#include "IfxCpu.h"
#include "IfxGtm.h"
#include "IfxGtm_Cmu.h"
#include "IfxGtm_Tom_Pwm.h"
#include "IfxPort.h"
#include "Board_Pins.h"
#include "CCU6_PWM_Generation.h"

#define PWM_PERIOD_TICKS             (5000u)  

#define DIR_STOP                     (0)
#define DIR_FORWARD                  (1)
#define DIR_REVERSE                  (-1)

#define MOTOR_A_FORWARD_DIR_LEVEL    (1u)
#define MOTOR_A_REVERSE_DIR_LEVEL    (0u)
#define MOTOR_B_FORWARD_DIR_LEVEL    (0u)
#define MOTOR_B_REVERSE_DIR_LEVEL    (1u)

#define MOTOR_A_BRAKE_RELEASE_LEVEL  (1u)
#define MOTOR_B_BRAKE_RELEASE_LEVEL  (1u)

static IfxGtm_Tom_Pwm_Driver g_pwmA;
static IfxGtm_Tom_Pwm_Driver g_pwmB;
static IfxGtm_Tom_Pwm_Config g_pwmCfgA;
static IfxGtm_Tom_Pwm_Config g_pwmCfgB;
static uint8                 g_pwmStarted = 0u;

static void setMotorABrakeRelease(void)
{
    if (MOTOR_A_BRAKE_RELEASE_LEVEL != 0u)
    {
        IfxPort_setPinHigh(MOTOR_A_BRAKE_PORT, MOTOR_A_BRAKE_PIN);
    }
    else
    {
        IfxPort_setPinLow(MOTOR_A_BRAKE_PORT, MOTOR_A_BRAKE_PIN);
    }
}

static void setMotorBBrakeRelease(void)
{
    if (MOTOR_B_BRAKE_RELEASE_LEVEL != 0u)
    {
        IfxPort_setPinHigh(MOTOR_B_BRAKE_PORT, MOTOR_B_BRAKE_PIN);
    }
    else
    {
        IfxPort_setPinLow(MOTOR_B_BRAKE_PORT, MOTOR_B_BRAKE_PIN);
    }
}

static void setMotorADirection(sint32 direction)
{
    uint8 level;

    if (direction == DIR_REVERSE)
    {
        level = MOTOR_A_REVERSE_DIR_LEVEL;
    }
    else
    {
        level = MOTOR_A_FORWARD_DIR_LEVEL;
    }

    if (level != 0u)
    {
        IfxPort_setPinHigh(MOTOR_A_DIR_PORT, MOTOR_A_DIR_PIN);
    }
    else
    {
        IfxPort_setPinLow(MOTOR_A_DIR_PORT, MOTOR_A_DIR_PIN);
    }
}

static void setMotorBDirection(sint32 direction)
{
    uint8 level;

    if (direction == DIR_REVERSE)
    {
        level = MOTOR_B_REVERSE_DIR_LEVEL;
    }
    else
    {
        level = MOTOR_B_FORWARD_DIR_LEVEL;
    }

    if (level != 0u)
    {
        IfxPort_setPinHigh(MOTOR_B_DIR_PORT, MOTOR_B_DIR_PIN);
    }
    else
    {
        IfxPort_setPinLow(MOTOR_B_DIR_PORT, MOTOR_B_DIR_PIN);
    }
}

static void setPwmDuty(IfxGtm_Tom_Pwm_Driver *driver, IfxGtm_Tom_Pwm_Config *cfg, uint8 dutyPercent)
{
    if (dutyPercent > 100u)
    {
        dutyPercent = 100u;
    }

    cfg->dutyCycle = (uint16)(((uint32)PWM_PERIOD_TICKS * (uint32)dutyPercent) / 100u);
    (void)IfxGtm_Tom_Pwm_init(driver, cfg);
}

static void initPwmChannel(IfxGtm_Tom_Pwm_Driver *driver,
                           IfxGtm_Tom_Pwm_Config *cfg,
                           IfxGtm_Tom tom,
                           IfxGtm_Tom_Ch tomChannel,
                           IfxGtm_Tom_ToutMap *outputPin)
{
    IfxGtm_Tom_Pwm_initConfig(cfg, &MODULE_GTM);
    cfg->tom                      = tom;
    cfg->tomChannel               = tomChannel;
    cfg->clock                    = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk0;
    cfg->period                   = PWM_PERIOD_TICKS;
    cfg->dutyCycle                = 0u;
    cfg->pin.outputPin            = outputPin;
    cfg->pin.outputMode           = IfxPort_OutputMode_pushPull;
    cfg->pin.padDriver            = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    cfg->synchronousUpdateEnabled = TRUE;
    cfg->signalLevel              = Ifx_ActiveState_high;

    (void)IfxGtm_Tom_Pwm_init(driver, cfg);
    IfxGtm_Tom_Pwm_start(driver, TRUE);
}

static void applyDutyToBoth(uint8 dutyPercent)
{
    boolean intState;

    if (dutyPercent > 100u)
    {
        dutyPercent = 100u;
    }

    setMotorABrakeRelease();
    setMotorBBrakeRelease();
    setMotorADirection(DIR_FORWARD);
    setMotorBDirection(DIR_FORWARD);

    intState = IfxCpu_disableInterrupts();
    setPwmDuty(&g_pwmA, &g_pwmCfgA, dutyPercent);
    setPwmDuty(&g_pwmB, &g_pwmCfgB, dutyPercent);
    IfxCpu_restoreInterrupts(intState);
}

void initMotorDirectionPins(void)
{
    IfxPort_setPinMode(MOTOR_A_BRAKE_PORT, MOTOR_A_BRAKE_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinMode(MOTOR_A_DIR_PORT,   MOTOR_A_DIR_PIN,   IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinMode(MOTOR_B_BRAKE_PORT, MOTOR_B_BRAKE_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinMode(MOTOR_B_DIR_PORT,   MOTOR_B_DIR_PIN,   IfxPort_Mode_outputPushPullGeneral);

    setMotorABrakeRelease();
    setMotorBBrakeRelease();
    setMotorADirection(DIR_FORWARD);
    setMotorBDirection(DIR_FORWARD);
}

void initCCU6(void)
{
    float32 moduleFrequency;

    initMotorDirectionPins();

    IfxGtm_enable(&MODULE_GTM);
    moduleFrequency = IfxGtm_Cmu_getModuleFrequency(&MODULE_GTM);
    IfxGtm_Cmu_setGclkFrequency(&MODULE_GTM, moduleFrequency);
    IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, moduleFrequency);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_FXCLK | IFXGTM_CMU_CLKEN_CLK0);

    initPwmChannel(&g_pwmA, &g_pwmCfgA, IfxGtm_Tom_0, IfxGtm_Tom_Ch_9, &IfxGtm_TOM0_9_TOUT1_P02_1_OUT);
    initPwmChannel(&g_pwmB, &g_pwmCfgB, IfxGtm_Tom_0, IfxGtm_Tom_Ch_3, &IfxGtm_TOM0_3_TOUT105_P10_3_OUT);

    g_pwmStarted = 1u;
    applyDutyToBoth(0u);
}

void startPWMGeneration(void)
{
    if (g_pwmStarted == 0u)
    {
        initCCU6();
    }
}

void updatePWMDutyCycle(uint8 dutyPercent)
{
    if (g_pwmStarted == 0u)
    {
        initCCU6();
    }

    applyDutyToBoth(dutyPercent);
}

void stopMotor(void)
{
    if (g_pwmStarted == 0u)
    {
        return;
    }

    applyDutyToBoth(0u);
}
