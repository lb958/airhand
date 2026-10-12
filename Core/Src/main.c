/* USER CODE BEGIN Header */
/**
 * @file main.c
 * @brief Single finger IMU test, UP_V3 diagnostic output
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

/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define APP_UART_PERIOD_MS 50u
/* USER CODE END PD */

/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* USER CODE BEGIN PV */

static FingerTracker finger;

/* 毫度：除以 1000 得到度 */
volatile int32_t debug_tilt_mdeg = 0;
volatile int32_t debug_signed_mdeg = 0;
volatile int32_t debug_flex_mdeg = 0;

/* 0 伸直、1 中间档、2 弯曲档、255 无效 */
volatile uint8_t debug_state = 255;
volatile uint8_t debug_valid = 0;

/*
 * 1：初始化
 * 2：初始化失败
 * 3：校准
 * 4：校准失败
 * 5：正常采集
 * 6：更新失败
 */
volatile uint8_t debug_stage = 0;

volatile uint8_t debug_hal_status = HAL_OK;
volatile uint8_t debug_uart_status = HAL_OK;
volatile uint32_t debug_sample_count = 0;

/* 改为 1 后，继续运行会重新初始化并校准 */
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

    /* AD0 接 GND，地址 0x68 */
    debug_stage = 1;

    result = Finger_Init(&finger, &hi2c1, 0x68);
    debug_hal_status = (uint8_t)result;

    if (result != HAL_OK) {
        debug_stage = 2;
        return;
    }

    /* 当前向上弯曲得到正角度，因此使用 +1 */
    finger.direction = 1.0f;
    finger.threshold1_deg = 25.0f;
    finger.threshold2_deg = 75.0f;
    finger.hysteresis_deg = 5.0f;

    /*
     * 保持当前芯片朝下的安装方式。
     * 手心朝上，食指伸直，手掌固定且静止。
     */
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
    debug_stage = 5;
}

static void App_UpdateSensor(void)
{
    HAL_StatusTypeDef result = Finger_Update(&finger);

    if (result == HAL_BUSY) {
        return;
    }

    debug_hal_status = (uint8_t)result;

    if (result != HAL_OK) {
        debug_valid = 0;
        debug_state = 255;
        debug_stage = 6;

        /* 保留最后角度，以 valid 判断是否有效 */
        return;
    }

    float relative = finger.angle_deg - finger.zero_deg;

    while (relative > 180.0f) {
        relative -= 360.0f;
    }

    while (relative < -180.0f) {
        relative += 360.0f;
    }

    debug_tilt_mdeg =
        (int32_t)(finger.angle_deg * 1000.0f);

    debug_signed_mdeg =
        (int32_t)(relative * 1000.0f);

    debug_flex_mdeg =
        (int32_t)(finger.flex_deg * 1000.0f);

    debug_state = finger.state;
    debug_valid = finger.valid;

    debug_sample_count++;
}

static void App_SendTelemetry(void)
{
    static uint32_t last_send_ms = 0;
    uint32_t now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - last_send_ms)
        < APP_UART_PERIOD_MS) {
        return;
    }

    last_send_ms = now_ms;

    char message[192];

    int length = snprintf(
        message,
        sizeof(message),
        "FW=UP_V3,n=%lu,dir=%ld,ddir=%ld,"
        "a=%ld,da=%ld,f=%ld,mf=%ld,df=%ld,"
        "state=%u,valid=%u,stage=%u,reason=%u\r\n",

        (unsigned long)debug_driver_count,
        (long)finger.direction,
        (long)debug_driver_direction,

        (long)debug_signed_mdeg,
        (long)debug_driver_relative_mdeg,

        (long)debug_flex_mdeg,
        (long)(finger.flex_deg * 1000.0f),
        (long)debug_driver_flex_mdeg,

        (unsigned)debug_state,
        (unsigned)debug_valid,
        (unsigned)debug_stage,
        (unsigned)debug_update_reason
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
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

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

        if (debug_stage == 5u) {
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

    /* 沿用外部 8MHz 晶振，PLL 倍频至 72MHz */
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

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */

    __disable_irq();

    while (1) {
    }

    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */

    (void)file;
    (void)line;

    /* USER CODE END 6 */
}

#endif