#include "GTM_TOM_PWM.h"

#include "IfxPort.h"
#include "IfxGtm.h"
#include "IfxGtm_Cmu.h"
#include "IfxGtm_Tom_Pwm.h"

#define PWM_PERIOD   5000u

/* =========================
 * Motor A
 * PWM  : P02.0 (TOM0_CH8)
 * IN1  : P21.3
 * IN2  : P21.4
 * ========================= */
#define MOTOR_A_IN1_PORT   &MODULE_P21
#define MOTOR_A_IN1_PIN    3
#define MOTOR_A_IN2_PORT   &MODULE_P21
#define MOTOR_A_IN2_PIN    4

/* =========================
 * Motor B
 * PWM  : P02.1 (TOM0_CH9)
 * IN1  : P21.5
 * IN2  : P22.0
 * ========================= */
#define MOTOR_B_IN1_PORT   &MODULE_P21
#define MOTOR_B_IN1_PIN    5
#define MOTOR_B_IN2_PORT   &MODULE_P22
#define MOTOR_B_IN2_PIN    0

/* =========================
 * Motor C
 * PWM  : P10.1 (TOM0_CH1)
 * IN1  : P22.1
 * IN2  : P22.2
 * ========================= */
#define MOTOR_C_IN1_PORT   &MODULE_P22
#define MOTOR_C_IN1_PIN    1
#define MOTOR_C_IN2_PORT   &MODULE_P22
#define MOTOR_C_IN2_PIN    2

/* =========================
 * Motor D
 * PWM  : P10.2 (TOM0_CH2)
 * IN1  : P23.0
 * IN2  : P23.1
 * ========================= */
#define MOTOR_D_IN1_PORT   &MODULE_P23
#define MOTOR_D_IN1_PIN    0
#define MOTOR_D_IN2_PORT   &MODULE_P23
#define MOTOR_D_IN2_PIN    1

static IfxGtm_Tom_Pwm_Driver g_pwmA;
static IfxGtm_Tom_Pwm_Driver g_pwmB;
static IfxGtm_Tom_Pwm_Driver g_pwmC;
static IfxGtm_Tom_Pwm_Driver g_pwmD;

static IfxGtm_Tom_Pwm_Config g_configA;
static IfxGtm_Tom_Pwm_Config g_configB;
static IfxGtm_Tom_Pwm_Config g_configC;
static IfxGtm_Tom_Pwm_Config g_configD;

static void setDutyA(uint8 duty_pct)
{
    if (duty_pct > 100u) duty_pct = 100u;
    g_configA.dutyCycle = (PWM_PERIOD * (uint32)duty_pct) / 100u;
    IfxGtm_Tom_Pwm_init(&g_pwmA, &g_configA);
    IfxGtm_Tom_Pwm_start(&g_pwmA, TRUE);
}

static void setDutyB(uint8 duty_pct)
{
    if (duty_pct > 100u) duty_pct = 100u;
    g_configB.dutyCycle = (PWM_PERIOD * (uint32)duty_pct) / 100u;
    IfxGtm_Tom_Pwm_init(&g_pwmB, &g_configB);
    IfxGtm_Tom_Pwm_start(&g_pwmB, TRUE);
}

static void setDutyC(uint8 duty_pct)
{
    if (duty_pct > 100u) duty_pct = 100u;
    g_configC.dutyCycle = (PWM_PERIOD * (uint32)duty_pct) / 100u;
    IfxGtm_Tom_Pwm_init(&g_pwmC, &g_configC);
    IfxGtm_Tom_Pwm_start(&g_pwmC, TRUE);
}

static void setDutyD(uint8 duty_pct)
{
    if (duty_pct > 100u) duty_pct = 100u;
    g_configD.dutyCycle = (PWM_PERIOD * (uint32)duty_pct) / 100u;
    IfxGtm_Tom_Pwm_init(&g_pwmD, &g_configD);
    IfxGtm_Tom_Pwm_start(&g_pwmD, TRUE);
}

void setDutyAll(uint8 duty_pct)
{
    setDutyA(duty_pct);
    setDutyB(duty_pct);
    setDutyC(duty_pct);
    setDutyD(duty_pct);
}

void motorForward(void)
{
    IfxPort_setPinHigh(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN);
    IfxPort_setPinLow (MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN);

    IfxPort_setPinHigh(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN);
    IfxPort_setPinLow (MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN);

    IfxPort_setPinHigh(MOTOR_C_IN1_PORT, MOTOR_C_IN1_PIN);
    IfxPort_setPinLow (MOTOR_C_IN2_PORT, MOTOR_C_IN2_PIN);

    IfxPort_setPinHigh(MOTOR_D_IN1_PORT, MOTOR_D_IN1_PIN);
    IfxPort_setPinLow (MOTOR_D_IN2_PORT, MOTOR_D_IN2_PIN);
}

void motorBackward(void)
{
    IfxPort_setPinLow (MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN);
    IfxPort_setPinHigh(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN);

    IfxPort_setPinLow (MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN);
    IfxPort_setPinHigh(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN);

    IfxPort_setPinLow (MOTOR_C_IN1_PORT, MOTOR_C_IN1_PIN);
    IfxPort_setPinHigh(MOTOR_C_IN2_PORT, MOTOR_C_IN2_PIN);

    IfxPort_setPinLow (MOTOR_D_IN1_PORT, MOTOR_D_IN1_PIN);
    IfxPort_setPinHigh(MOTOR_D_IN2_PORT, MOTOR_D_IN2_PIN);
}

void motorStop(void)
{
    setDutyAll(0u);
}

void initMotorPwm(void)
{
    /* 방향 핀 초기화 */
    IfxPort_setPinModeOutput(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    IfxPort_setPinModeOutput(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    IfxPort_setPinModeOutput(MOTOR_C_IN1_PORT, MOTOR_C_IN1_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(MOTOR_C_IN2_PORT, MOTOR_C_IN2_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    IfxPort_setPinModeOutput(MOTOR_D_IN1_PORT, MOTOR_D_IN1_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(MOTOR_D_IN2_PORT, MOTOR_D_IN2_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    /* GTM 활성화 */
    IfxGtm_enable(&MODULE_GTM);
    IfxGtm_Cmu_setGclkFrequency(&MODULE_GTM, 100000000.0f);
    IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, 100000000.0f);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0 | IFXGTM_CMU_CLKEN_FXCLK);

    /* PWM A : P02.0 (TOM0_CH8) */
    IfxGtm_Tom_Pwm_initConfig(&g_configA, &MODULE_GTM);
    g_configA.tom                      = IfxGtm_Tom_0;
    g_configA.tomChannel               = IfxGtm_Tom_Ch_8;
    g_configA.period                   = PWM_PERIOD;
    g_configA.dutyCycle                = 0u;
    g_configA.pin.outputPin            = &IfxGtm_TOM0_8_TOUT0_P02_0_OUT;
    g_configA.synchronousUpdateEnabled = TRUE;
    IfxGtm_Tom_Pwm_init(&g_pwmA, &g_configA);
    IfxGtm_Tom_Pwm_start(&g_pwmA, TRUE);

    /* PWM B : P02.1 (TOM0_CH9) */
    IfxGtm_Tom_Pwm_initConfig(&g_configB, &MODULE_GTM);
    g_configB.tom                      = IfxGtm_Tom_0;
    g_configB.tomChannel               = IfxGtm_Tom_Ch_9;
    g_configB.period                   = PWM_PERIOD;
    g_configB.dutyCycle                = 0u;
    g_configB.pin.outputPin            = &IfxGtm_TOM0_9_TOUT1_P02_1_OUT;
    g_configB.synchronousUpdateEnabled = TRUE;
    IfxGtm_Tom_Pwm_init(&g_pwmB, &g_configB);
    IfxGtm_Tom_Pwm_start(&g_pwmB, TRUE);

    /* PWM C : P10.1 (TOM0_CH1) */
    IfxGtm_Tom_Pwm_initConfig(&g_configC, &MODULE_GTM);
    g_configC.tom                      = IfxGtm_Tom_0;
    g_configC.tomChannel               = IfxGtm_Tom_Ch_1;
    g_configC.period                   = PWM_PERIOD;
    g_configC.dutyCycle                = 0u;
    g_configC.pin.outputPin            = &IfxGtm_TOM0_1_TOUT103_P10_1_OUT;
    g_configC.synchronousUpdateEnabled = TRUE;
    IfxGtm_Tom_Pwm_init(&g_pwmC, &g_configC);
    IfxGtm_Tom_Pwm_start(&g_pwmC, TRUE);

    /* PWM D : P10.2 (TOM0_CH2) */
    IfxGtm_Tom_Pwm_initConfig(&g_configD, &MODULE_GTM);
    g_configD.tom                      = IfxGtm_Tom_0;
    g_configD.tomChannel               = IfxGtm_Tom_Ch_2;
    g_configD.period                   = PWM_PERIOD;
    g_configD.dutyCycle                = 0u;
    g_configD.pin.outputPin            = &IfxGtm_TOM0_2_TOUT104_P10_2_OUT;
    g_configD.synchronousUpdateEnabled = TRUE;
    IfxGtm_Tom_Pwm_init(&g_pwmD, &g_configD);
    IfxGtm_Tom_Pwm_start(&g_pwmD, TRUE);
}
