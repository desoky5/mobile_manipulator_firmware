/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Mobile Robot + 3DOF Arm — Ball Pickup
  * Pi sends: N=searching forward, L=rotate left, R=rotate right, S=stop+grab
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
    float output_limit;
} PID_Controller;

typedef struct {
    float theta_1d;
    float theta_2d;
    float theta_3d;
} Angles;

typedef enum {
    ARM_IDLE = 0,
    ARM_SETTLE,
    ARM_GO_HOME,
    ARM_MOVE_TO_BALL,
    ARM_CLOSE_GRIPPER,
    ARM_LIFT_UP,
    ARM_DONE
} ArmState_t;

typedef enum {
    ROBOT_SEARCHING = 0,   /* N — drive forward looking */
    ROBOT_ROTATE_LEFT,     /* L — rotate left to center */
    ROBOT_ROTATE_RIGHT,    /* R — rotate right to center */
    ROBOT_GRABBING,        /* S — stopped, arm working  */
    ROBOT_HOLDING          /* arm done, drive forward holding ball */
} RobotState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PPR          11880.0f
#define SAMPLE_MS    100.0f
#define FF_GAIN      27.0f

#define SERVO_MIN_PULSE  500
#define SERVO_MAX_PULSE  2500

#define FORWARD_RPM   20.0f
#define ROTATE_RPM    15.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define servo1_write(a) __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, angle_to_ccr(a))
#define servo2_write(a) __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, angle_to_ccr(a))
#define servo3_write(a) __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, angle_to_ccr(a))
#define servo4_write(a) __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, angle_to_ccr(a))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* UART */
uint8_t  rx_byte;
volatile uint8_t current_command = 'N';

/* Encoder / motor */
volatile float motorR_rpm = 0.0f;
volatile float motorL_rpm = 0.0f;
volatile float target_rpm_right = 0.0f;
volatile float target_rpm_left  = 0.0f;

PID_Controller pid_left;
PID_Controller pid_right;

/* State machines */
volatile RobotState_t robot_state = ROBOT_SEARCHING;
ArmState_t arm_state = ARM_IDLE;
uint32_t   arm_timer = 0;
Angles     arm_angles;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ── Servo helpers ──────────────────────────────────────────────────────── */
static uint32_t angle_to_ccr(float angle)
{
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    return (uint32_t)(SERVO_MIN_PULSE +
           (angle / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
}

static void TIM5_Reconfigure_For_Servo(void)
{
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
    __HAL_TIM_SET_PRESCALER(&htim5, 83);
    __HAL_TIM_SET_AUTORELOAD(&htim5, 19999);
    HAL_TIM_GenerateEvent(&htim5, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}

static void TIM5_Restore_For_PID(void)
{
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
    __HAL_TIM_SET_PRESCALER(&htim5, 8399);
    __HAL_TIM_SET_AUTORELOAD(&htim5, 99);
    HAL_TIM_GenerateEvent(&htim5, TIM_EVENTSOURCE_UPDATE);
}

static void TIM9_Reconfigure_For_Servo(void)
{
    HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
    __HAL_TIM_SET_PRESCALER(&htim9, 83);
    __HAL_TIM_SET_AUTORELOAD(&htim9, 19999);
    HAL_TIM_GenerateEvent(&htim9, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
}

/* ── PID ────────────────────────────────────────────────────────────────── */
float Compute_PID(PID_Controller *pid, float target, float actual)
{
    float error   = target - actual;
    float p_term  = pid->kp * error;
    float ff_term = target * FF_GAIN;

    pid->integral += error * 0.1f;
    if (pid->integral >  pid->output_limit) pid->integral =  pid->output_limit;
    if (pid->integral < -pid->output_limit) pid->integral = -pid->output_limit;
    float i_term = pid->ki * pid->integral;

    float d_term = pid->kd * ((error - pid->prev_error) / 0.1f);
    pid->prev_error = error;

    float output = ff_term + p_term + i_term + d_term;
    if (output >  pid->output_limit) output =  pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    return output;
}

/* ── Motor drive ────────────────────────────────────────────────────────── */
void Set_Motor_Right(float pwm_val)
{
    if (pwm_val >= 0.0f) {
        HAL_GPIO_WritePin(MOT_R_IN1_GPIO_Port, MOT_R_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOT_R_IN2_GPIO_Port, MOT_R_IN2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOT_R_IN1_GPIO_Port, MOT_R_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOT_R_IN2_GPIO_Port, MOT_R_IN2_Pin, GPIO_PIN_RESET);
        pwm_val = -pwm_val;
    }
    if (pwm_val > 999.0f) pwm_val = 999.0f;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)pwm_val);
}

void Set_Motor_Left(float pwm_val)
{
    if (pwm_val >= 0.0f) {
        HAL_GPIO_WritePin(MOT_L_IN1_GPIO_Port, MOT_L_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOT_L_IN2_GPIO_Port, MOT_L_IN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(MOT_L_IN1_GPIO_Port, MOT_L_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOT_L_IN2_GPIO_Port, MOT_L_IN2_Pin, GPIO_PIN_SET);
        pwm_val = -pwm_val;
    }
    if (pwm_val > 999.0f) pwm_val = 999.0f;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)pwm_val);
}

/* ── Inverse kinematics ─────────────────────────────────────────────────── */
static float constrain_f(float val, float mn, float mx)
{
    if (val < mn) return mn;
    if (val > mx) return mx;
    return val;
}

Angles inverse_kinematics(float xd, float yd, float zd)
{
    const float L1 = 4.349f;
    const float L2 = 16.3104f;
    const float L3 = 20.0f;
    Angles result;

    result.theta_1d = (atan2f(yd, xd) * 180.0f / (float)M_PI) - 25.0f;
    result.theta_1d = constrain_f(result.theta_1d, 0.0f, 180.0f);

    float r = sqrtf(xd * xd + yd * yd);
    float k = sqrtf(r * r + powf((zd - L1), 2.0f));
    float D = (L2 * L2 + L3 * L3 - k * k) / (2.0f * L2 * L3);
    D = constrain_f(D, -1.0f, 1.0f);

    float theta   = acosf(D);
    float theta_3 = 180.0f - (theta * 180.0f / (float)M_PI);
    result.theta_3d = theta_3 + 42.0f;
    if (result.theta_3d > 180.0f)
        result.theta_3d = -result.theta_3d + 360.0f;
    else if (result.theta_3d < 0.0f && result.theta_3d > -180.0f)
        result.theta_3d = -result.theta_3d;

    float beta  = atan2f((zd - L1), r);
    float alpha = atan2f((L3 * sinf(theta)), (k + L3 * cosf(theta)));
    result.theta_2d = (beta + alpha) * 180.0f / (float)M_PI;
    if (result.theta_2d > 180.0f)
        result.theta_2d = -result.theta_2d + 360.0f;
    else if (result.theta_2d < 0.0f && result.theta_2d > -180.0f)
        result.theta_2d = -result.theta_2d;

    return result;
}

/* ── UART debug echo — STM32 echoes received byte back to Pi ────────────── */
static void uart_debug(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

/* USER CODE END 0 */

/* ══════════════════════════════════════════════════════════════════════════
 * MAIN
 * ══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    /* USER CODE BEGIN 1 */
    pid_left.kp  = 5.0f;  pid_left.ki  = 0.5f;  pid_left.kd  = 0.0f;
    pid_left.prev_error = 0.0f; pid_left.integral = 0.0f;
    pid_left.output_limit = 999.0f;

    pid_right.kp = 5.0f;  pid_right.ki = 0.5f;  pid_right.kd = 0.0f;
    pid_right.prev_error = 0.0f; pid_right.integral = 0.0f;
    pid_right.output_limit = 999.0f;
    /* USER CODE END 1 */

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM9_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */

    /* Start motor PWM */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    /* Start encoders */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* Start PID timer interrupt */
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim1);

    /* Start UART receive interrupt */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    /* Arm boot — go to home position */
    TIM5_Reconfigure_For_Servo();
    TIM9_Reconfigure_For_Servo();
    servo4_write(180);   /* gripper open */
    HAL_Delay(300);
    servo1_write(0);
    servo2_write(90);
    servo3_write(100);
    HAL_Delay(2000);
    TIM5_Restore_For_PID();

    uart_debug("STM32 READY\r\n");

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */

        /* ── STEP 1: translate Pi command → robot state ─────────────────── */
        switch (current_command)
        {
            case 'N': case 'n':
                if (robot_state != ROBOT_GRABBING && robot_state != ROBOT_HOLDING)
                    robot_state = ROBOT_SEARCHING;
                break;

            case 'L': case 'l':
                if (robot_state != ROBOT_GRABBING && robot_state != ROBOT_HOLDING)
                    robot_state = ROBOT_ROTATE_LEFT;
                break;

            case 'R': case 'r':
                if (robot_state != ROBOT_GRABBING && robot_state != ROBOT_HOLDING)
                    robot_state = ROBOT_ROTATE_RIGHT;
                break;

            case 'S': case 's':
                if (robot_state != ROBOT_GRABBING && robot_state != ROBOT_HOLDING)
                {
                    robot_state = ROBOT_GRABBING;
                    arm_state   = ARM_SETTLE;
                    arm_timer   = HAL_GetTick();
                    uart_debug("CMD:GRAB\r\n");
                }
                break;

            default:
                break;
        }

        /* ── STEP 2: set motor targets based on robot state ─────────────── */
        switch (robot_state)
        {
            case ROBOT_SEARCHING:
                target_rpm_right = FORWARD_RPM;
                target_rpm_left  = FORWARD_RPM;
                break;

            case ROBOT_ROTATE_LEFT:
                target_rpm_right =  ROTATE_RPM;
                target_rpm_left  = -ROTATE_RPM;
                break;

            case ROBOT_ROTATE_RIGHT:
                target_rpm_right = -ROTATE_RPM;
                target_rpm_left  =  ROTATE_RPM;
                break;

            case ROBOT_GRABBING:
                /* motors fully stopped while arm works */
                target_rpm_right = 0.0f;
                target_rpm_left  = 0.0f;
                break;

            case ROBOT_HOLDING:
                /* ball grabbed — drive forward holding it */
                target_rpm_right = FORWARD_RPM;
                target_rpm_left  = FORWARD_RPM;
                break;
        }

        /* ── STEP 3: arm state machine (non-blocking) ───────────────────── */
        switch (arm_state)
        {
            case ARM_IDLE:
                break;

            case ARM_SETTLE:
                /* wait 500ms for motors to fully stop */
                if (HAL_GetTick() - arm_timer >= 500)
                {
                    TIM5_Reconfigure_For_Servo();
                    TIM9_Reconfigure_For_Servo();
                    arm_angles = inverse_kinematics(0.0f, 23.0f, 7.2f);
                    servo4_write(180);  /* make sure gripper is open */
                    uart_debug("ARM:SETTLE\r\n");
                    arm_state = ARM_GO_HOME;
                    arm_timer = HAL_GetTick();
                }
                break;

            case ARM_GO_HOME:
                /* go to home position first */
                if (HAL_GetTick() - arm_timer >= 500)
                {
                    servo1_write(0);
                    servo2_write(90);
                    servo3_write(100);
                    uart_debug("ARM:HOME\r\n");
                    arm_state = ARM_MOVE_TO_BALL;
                    arm_timer = HAL_GetTick();
                }
                break;

            case ARM_MOVE_TO_BALL:
                /* move arm to IK target position */
                if (HAL_GetTick() - arm_timer >= 2000)
                {
                    servo1_write(arm_angles.theta_1d);
                    servo2_write(arm_angles.theta_2d);
                    servo3_write(arm_angles.theta_3d);
                    uart_debug("ARM:MOVING\r\n");
                    arm_state = ARM_CLOSE_GRIPPER;
                    arm_timer = HAL_GetTick();
                }
                break;

            case ARM_CLOSE_GRIPPER:
                /* close gripper to grab ball */
                if (HAL_GetTick() - arm_timer >= 2000)
                {
                    servo4_write(0);
                    uart_debug("ARM:GRIP\r\n");
                    arm_state = ARM_LIFT_UP;
                    arm_timer = HAL_GetTick();
                }
                break;

            case ARM_LIFT_UP:
                /* lift arm back to carrying position */
                if (HAL_GetTick() - arm_timer >= 1500)
                {
                    servo1_write(0);
                    servo2_write(90);
                    servo3_write(100);
                    uart_debug("ARM:LIFTED\r\n");
                    arm_state = ARM_DONE;
                    arm_timer = HAL_GetTick();
                }
                break;

            case ARM_DONE:
                /* restore TIM5 for PID, robot continues moving */
                if (HAL_GetTick() - arm_timer >= 1000)
                {
                    TIM5_Restore_For_PID();
                    robot_state = ROBOT_HOLDING;
                    arm_state   = ARM_IDLE;
                    uart_debug("ARM:DONE\r\n");
                }
                break;
        }

        HAL_Delay(10);
    }
    /* USER CODE END 3 */
}

/* ══════════════════════════════════════════════════════════════════════════
 * TIMER INTERRUPT — runs every 100ms for PID
 * ══════════════════════════════════════════════════════════════════════════ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        int16_t raw_enc_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        int16_t raw_enc_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        __HAL_TIM_SET_COUNTER(&htim4, 0);

        motorR_rpm = ((float)raw_enc_r / PPR) * (6000.0f / SAMPLE_MS);
        motorL_rpm = ((float)raw_enc_l / PPR) * (6000.0f / SAMPLE_MS);

        float pwm_r = Compute_PID(&pid_right, target_rpm_right, motorR_rpm);
        float pwm_l = Compute_PID(&pid_left,  target_rpm_left,  motorL_rpm);

        Set_Motor_Right(pwm_r);
        Set_Motor_Left(pwm_l);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * UART INTERRUPT — receives one byte at a time from Pi
 * ══════════════════════════════════════════════════════════════════════════ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* ignore whitespace/newlines */
        if (rx_byte != '\n' && rx_byte != '\r' && rx_byte != ' ')
        {
            current_command = rx_byte;

            /* echo back to Pi so you can confirm reception via cat /dev/serial0 */
            HAL_UART_Transmit(&huart1, &rx_byte, 1, 100);
        }
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState  = HAL_UART_STATE_READY;
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState  = HAL_UART_STATE_READY;
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * PERIPHERAL INIT FUNCTIONS — copied exactly from your working CubeMX file
 * ══════════════════════════════════════════════════════════════════════════ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 168;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 8399;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 999;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};
    TIM_OC_InitTypeDef      sConfigOC          = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 83;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 999;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig      = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 0;
    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter    = 0;
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig      = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 65535;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 0;
    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter    = 0;
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM5_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};
    TIM_OC_InitTypeDef      sConfigOC          = {0};

    htim5.Instance               = TIM5;
    htim5.Init.Prescaler         = 8399;
    htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim5.Init.Period            = 99;
    htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) Error_Handler();
    if (HAL_TIM_OC_Init(&htim5) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

    sConfigOC.OCMode = TIM_OCMODE_TIMING;
    if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim5);
}

static void MX_TIM9_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim9.Instance               = TIM9;
    htim9.Init.Prescaler         = 0;
    htim9.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim9.Init.Period            = 65535;
    htim9.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim9) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim9);
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, Built_In_LED_Pin | MOT_R_IN2_Pin | MOT_R_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MOT_L_IN1_Pin | MOT_L_IN2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = Built_In_LED_Pin | MOT_R_IN2_Pin | MOT_R_IN1_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = MOT_L_IN1_Pin | MOT_L_IN2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
