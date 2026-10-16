/* USER CODE BEGIN Header */
/**
 * @file main.c
 * @brief Single finger IMU test, UP_V6_FAST
 */
/* USER CODE END Header */

#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "finger_mpu6050.h"
#include <stdio.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */

#define APP_UART_PERIOD_MS 100u

#define APP_STAGE_COLLECT  5u
#define APP_STAGE_ERROR    6u
#define APP_STAGE_HOLD     7u

/* USER CODE END PD */

/* USER CODE BEGIN PV */

static FingerTracker finger;

/* 单位：毫度，除以1000得到度 */
volatile int32_t debug_tilt_mdeg = 0;
volatile int32_t debug_signed_mdeg = 0;
volatile int32_t debug_flex_mdeg = 0;

/*
 * state：
 * 0：伸直
 * 1：半握拳
 * 2：握拳
 * 255：无可用档位
 */
volatile uint8_t debug_state = 255;
volatile uint8_t debug_valid = 0;

/*
 * stage：
 * 1：初始化
 * 2：初始化失败
 * 3：校准
 * 4：校准失败
 * 5：正常采集
 * 6：更新错误，继续重试
 * 7：暂时保留旧角度，继续重试
 */
volatile uint8_t debug_stage = 0;

volatile uint8_t debug_hal_status = HAL_OK;
volatile uint8_t debug_uart_status = HAL_OK;
volatile uint32_t debug_sample_count = 0;

/* 在调试器中改为1，可以重新初始化并校准 */
volatile uint8_t debug_command = 0;

extern volatile uint8_t debug_update_reason;

/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

static void App_StartSensor(void);
static void App_UpdateSensor(void);
static void App_SendTelemetry(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void App_StartSensor(void)
{
    HAL_StatusTypeDef result;

    debug_valid = 0;
    debug_state = 255;
    debug_sample_count = 0;

    debug_tilt_mdeg = 0;
    debug_signed_mdeg = 0;
    debug_flex_mdeg = 0;
    debug_update_reason = 0;

    /* AD0接GND，地址为0x68 */
    debug_stage = 1;

    result = Finger_Init(&finger, &hi2c1, 0x68);
    debug_hal_status = (uint8_t)result;

    if (result != HAL_OK) {
        debug_stage = 2;
        return;
    }

    finger.direction = 1.0f;

    /*
     * 分档参数：以后主要修改这里。
     *
     * 当前设置：
     * 伸直 -> 半握拳：达到50度
     * 半握拳 -> 握拳：达到105度
     *
     * 握拳 -> 半握拳：降至95度
     * 半握拳 -> 伸直：降至40度
     *

     */
    finger.threshold1_deg = 85.0f;
    finger.threshold2_deg = 160.0f;
    finger.hysteresis_deg = 5.0f;

    /*
     * 手心朝上、食指伸直，并保持静止。
     */
    HAL_Delay(2000);

    debug_stage = 3;

    result = Finger_Calibrate(&finger);
    debug_hal_status = (uint8_t)result;

    if (result != HAL_OK) {
        debug_stage = 4;
        return;
    }

    debug_tilt_mdeg =
        (int32_t)(finger.angle_deg * 1000.0f);

    debug_valid = finger.valid;
    debug_state = finger.state;
    debug_stage = APP_STAGE_COLLECT;
}

static void App_UpdateSensor(void)
{
    HAL_StatusTypeDef result = Finger_Update(&finger);

    if (result == HAL_BUSY && finger.calibrated) {

        /*
         * 未到采样时间：
         * 保持当前显示。
         *
         * 本次样本不可靠：
         * 保留上次角度，显示阶段7，继续重试。
         */
        if (!finger.valid) {
            debug_hal_status = (uint8_t)HAL_BUSY;
            debug_valid = 0;
            debug_state = finger.state;
            debug_stage = APP_STAGE_HOLD;
        }

        return;
    }

    debug_hal_status = (uint8_t)result;

    if (result != HAL_OK || !finger.valid) {
        debug_valid = 0;
        debug_state = 255;
        debug_stage = APP_STAGE_ERROR;
        return;
    }

    const float relative =
        finger.angle_deg - finger.zero_deg;

    debug_tilt_mdeg =
        (int32_t)(finger.angle_deg * 1000.0f);

    debug_signed_mdeg =
        (int32_t)(relative * 1000.0f);

    debug_flex_mdeg =
        (int32_t)(finger.flex_deg * 1000.0f);

    debug_state = finger.state;
    debug_valid = finger.valid;

    debug_sample_count++;

    /* 读取成功后，自动恢复到阶段5 */
    debug_stage = APP_STAGE_COLLECT;
}

static void App_SendTelemetry(void)
{
    static uint32_t last_send_ms = 0;

    uint32_t now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - last_send_ms) <
        APP_UART_PERIOD_MS) {
        return;
        }

    last_send_ms = now_ms;

    long bend_deg =
        (long)((debug_flex_mdeg + 500) / 1000);

    long relative_deg =
        (long)(debug_signed_mdeg / 1000);

    char message[192];

    int length = snprintf(
        message,
        sizeof(message),
        "bend_deg=%ld,rel_deg=%ld,"
        "state=%u,valid=%u,stage=%u,reason=%u,n=%lu\r\n",
        bend_deg,
        relative_deg,
        (unsigned)debug_state,
        (unsigned)debug_valid,
        (unsigned)debug_stage,
        (unsigned)debug_update_reason,
        (unsigned long)debug_sample_count
    );

    if (length > 0 && length < (int)sizeof(message)) {

        debug_uart_status = (uint8_t)HAL_UART_Transmit(
            &huart1,
            (uint8_t *)message,
            (uint16_t)length,
            20u
        );
    }
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */

    App_StartSensor();

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        if (debug_command == 1u) {
            debug_command = 0;
            App_StartSensor();
        }

        /*
         * 阶段5：正常采集
         * 阶段6：错误后继续重试
         * 阶段7：暂时保持旧角度，继续重试
         */
        if (debug_stage == APP_STAGE_COLLECT ||
            debug_stage == APP_STAGE_ERROR ||
            debug_stage == APP_STAGE_HOLD) {

            App_UpdateSensor();
        }

        App_SendTelemetry();
    }
    /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE;

    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2) != HAL_OK) {

        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();

    while (1) {
    }
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}

#endif