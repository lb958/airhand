// /* USER CODE BEGIN Header */
// /**
//  * @file main.c
//  * @brief Single finger IMU test, UP_V6_FAST
//  */
// /* USER CODE END Header */
// /* Includes ------------------------------------------------------------------*/
// #include "main.h"
// #include "i2c.h"
// #include "usart.h"
// #include "gpio.h"
//
// /* Private includes ----------------------------------------------------------*/
// /* USER CODE BEGIN Includes */
// #include "finger_mpu6050.h"
// #include <stdio.h>
// /* USER CODE END Includes */
//
// /* Private typedef -----------------------------------------------------------*/
// /* USER CODE BEGIN PTD */
//
// /* USER CODE END PTD */
//
// /* Private define ------------------------------------------------------------*/
// /* USER CODE BEGIN PD */
//
// #define APP_UART_PERIOD_MS 100u
//
// #define APP_STAGE_COLLECT  5u
// #define APP_STAGE_ERROR    6u
// #define APP_STAGE_HOLD     7u
//
// /* USER CODE END PD */
//
// /* Private macro -------------------------------------------------------------*/
// /* USER CODE BEGIN PM */
//
// /* USER CODE END PM */
//
// /* Private variables ---------------------------------------------------------*/
//
// /* USER CODE BEGIN PV */
//
// static FingerTracker finger;
//
// /* 单位：毫度，除以1000得到度 */
// volatile int32_t debug_tilt_mdeg = 0;
// volatile int32_t debug_signed_mdeg = 0;
// volatile int32_t debug_flex_mdeg = 0;
//
// /*
//  * state：
//  * 0：伸直
//  * 1：半握拳
//  * 2：握拳
//  * 255：无可用档位
//  */
// volatile uint8_t debug_state = 255;
// volatile uint8_t debug_valid = 0;
//
// /*
//  * stage：
//  * 1：初始化
//  * 2：初始化失败
//  * 3：校准
//  * 4：校准失败
//  * 5：正常采集
//  * 6：更新错误，继续重试
//  * 7：暂时保留旧角度，继续重试
//  */
// volatile uint8_t debug_stage = 0;
//
// volatile uint8_t debug_hal_status = HAL_OK;
// volatile uint8_t debug_uart_status = HAL_OK;
// volatile uint32_t debug_sample_count = 0;
//
// /* 在调试器中改为1，可以重新初始化并校准 */
// volatile uint8_t debug_command = 0;
//
// extern volatile uint8_t debug_update_reason;
//
// /* USER CODE END PV */
//
// /* Private function prototypes -----------------------------------------------*/
// void SystemClock_Config(void);
// /* USER CODE BEGIN PFP */
//
// static void App_StartSensor(void);
// static void App_UpdateSensor(void);
// static void App_SendTelemetry(void);
//
// /* USER CODE END PFP */
//
// /* Private user code ---------------------------------------------------------*/
// /* USER CODE BEGIN 0 */
//
// static void App_StartSensor(void)
// {
//     HAL_StatusTypeDef result;
//
//     debug_valid = 0;
//     debug_state = 255;
//     debug_sample_count = 0;
//
//     debug_tilt_mdeg = 0;
//     debug_signed_mdeg = 0;
//     debug_flex_mdeg = 0;
//     debug_update_reason = 0;
//
//     /* AD0接GND，地址为0x68 */
//     debug_stage = 1;
//
//     result = Finger_Init(&finger, &hi2c1, 0x68);
//     debug_hal_status = (uint8_t)result;
//
//     if (result != HAL_OK) {
//         debug_stage = 2;
//         return;
//     }
//
//     finger.direction = 1.0f;
//
//     /*
//      * 分档参数：以后主要修改这里。
//      *
//      * 当前设置：
//      * 伸直 -> 半握拳：达到50度
//      * 半握拳 -> 握拳：达到105度
//      *
//      * 握拳 -> 半握拳：降至95度
//      * 半握拳 -> 伸直：降至40度
//      *
//
//      */
//     finger.threshold1_deg = 85.0f;
//     finger.threshold2_deg = 160.0f;
//     finger.hysteresis_deg = 5.0f;
//
//     /*
//      * 手心朝上、食指伸直，并保持静止。
//      */
//     HAL_Delay(2000);
//
//     debug_stage = 3;
//
//     result = Finger_Calibrate(&finger);
//     debug_hal_status = (uint8_t)result;
//
//     if (result != HAL_OK) {
//         debug_stage = 4;
//         return;
//     }
//
//     debug_tilt_mdeg =
//         (int32_t)(finger.angle_deg * 1000.0f);
//
//     debug_valid = finger.valid;
//     debug_state = finger.state;
//     debug_stage = APP_STAGE_COLLECT;
// }
//
// static void App_UpdateSensor(void)
// {
//     HAL_StatusTypeDef result = Finger_Update(&finger);
//
//     if (result == HAL_BUSY && finger.calibrated) {
//
//         /*
//          * 未到采样时间：
//          * 保持当前显示。
//          *
//          * 本次样本不可靠：
//          * 保留上次角度，显示阶段7，继续重试。
//          */
//         if (!finger.valid) {
//             debug_hal_status = (uint8_t)HAL_BUSY;
//             debug_valid = 0;
//             debug_state = finger.state;
//             debug_stage = APP_STAGE_HOLD;
//         }
//
//         return;
//     }
//
//     debug_hal_status = (uint8_t)result;
//
//     if (result != HAL_OK || !finger.valid) {
//         debug_valid = 0;
//         debug_state = 255;
//         debug_stage = APP_STAGE_ERROR;
//         return;
//     }
//
//     const float relative =
//         finger.angle_deg - finger.zero_deg;
//
//     debug_tilt_mdeg =
//         (int32_t)(finger.angle_deg * 1000.0f);
//
//     debug_signed_mdeg =
//         (int32_t)(relative * 1000.0f);
//
//     debug_flex_mdeg =
//         (int32_t)(finger.flex_deg * 1000.0f);
//
//     debug_state = finger.state;
//     debug_valid = finger.valid;
//
//     debug_sample_count++;
//
//     /* 读取成功后，自动恢复到阶段5 */
//     debug_stage = APP_STAGE_COLLECT;
// }
//
// static void App_SendTelemetry(void)
// {
//     static uint32_t last_send_ms = 0;
//
//     uint32_t now_ms = HAL_GetTick();
//
//     if ((uint32_t)(now_ms - last_send_ms) <
//         APP_UART_PERIOD_MS) {
//         return;
//         }
//
//     last_send_ms = now_ms;
//
//     long bend_deg =
//         (long)((debug_flex_mdeg + 500) / 1000);
//
//     long relative_deg =
//         (long)(debug_signed_mdeg / 1000);
//
//     char message[192];
//
//     int length = snprintf(
//         message,
//         sizeof(message),
//         "bend_deg=%ld,rel_deg=%ld,"
//         "state=%u,valid=%u,stage=%u,reason=%u,n=%lu\r\n",
//         bend_deg,
//         relative_deg,
//         (unsigned)debug_state,
//         (unsigned)debug_valid,
//         (unsigned)debug_stage,
//         (unsigned)debug_update_reason,
//         (unsigned long)debug_sample_count
//     );
//
//     if (length > 0 && length < (int)sizeof(message)) {
//
//         debug_uart_status = (uint8_t)HAL_UART_Transmit(
//             &huart1,
//             (uint8_t *)message,
//             (uint16_t)length,
//             20u
//         );
//     }
// }
//
// /* USER CODE END 0 */
//
// /**
//   * @brief  The application entry point.
//   * @retval int
//   */
// int main(void)
// {
//
//   /* USER CODE BEGIN 1 */
//
//   /* USER CODE END 1 */
//
//   /* MCU Configuration--------------------------------------------------------*/
//
//   /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//   HAL_Init();
//
//   /* USER CODE BEGIN Init */
//
//   /* USER CODE END Init */
//
//   /* Configure the system clock */
//   SystemClock_Config();
//
//   /* USER CODE BEGIN SysInit */
//
//   /* USER CODE END SysInit */
//
//   /* Initialize all configured peripherals */
//   MX_GPIO_Init();
//   MX_I2C1_Init();
//   MX_USART1_UART_Init();
//   MX_I2C2_Init();
//   /* USER CODE BEGIN 2 */
//
//     App_StartSensor();
//
//   /* USER CODE END 2 */
//
//   /* Infinite loop */
//   /* USER CODE BEGIN WHILE */
//     while (1) {
//     /* USER CODE END WHILE */
//
//     /* USER CODE BEGIN 3 */
//
//         if (debug_command == 1u) {
//             debug_command = 0;
//             App_StartSensor();
//         }
//
//         /*
//          * 阶段5：正常采集
//          * 阶段6：错误后继续重试
//          * 阶段7：暂时保持旧角度，继续重试
//          */
//         if (debug_stage == APP_STAGE_COLLECT ||
//             debug_stage == APP_STAGE_ERROR ||
//             debug_stage == APP_STAGE_HOLD) {
//
//             App_UpdateSensor();
//         }
//
//         App_SendTelemetry();
//     }
//   /* USER CODE END 3 */
// }
//
// /**
//   * @brief System Clock Configuration
//   * @retval None
//   */
// void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//   RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//   RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }
//
//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }
//
// /* USER CODE BEGIN 4 */
//
// /* USER CODE END 4 */
//
// /**
//   * @brief  This function is executed in case of error occurrence.
//   * @retval None
//   */
// void Error_Handler(void)
// {
//   /* USER CODE BEGIN Error_Handler_Debug */
//   /* User can add his own implementation to report the HAL error return state */
//   __disable_irq();
//   while (1)
//   {
//   }
//   /* USER CODE END Error_Handler_Debug */
// }
// #ifdef USE_FULL_ASSERT
// /**
//   * @brief  Reports the name of the source file and the source line number
//   *         where the assert_param error has occurred.
//   * @param  file: pointer to the source file name
//   * @param  line: assert_param error line source number
//   * @retval None
//   */
// void assert_failed(uint8_t *file, uint32_t line)
// {
//   /* USER CODE BEGIN 6 */
//   /* User can add his own implementation to report the file name and line number,
//      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//   /* USER CODE END 6 */
// }
// #endif /* USE_FULL_ASSERT */







/* USER CODE BEGIN Header */
/*
 * 单板双指采集：
 * 中指：I2C1，PB6=SCL，PB7=SDA
 * 无名指：I2C2，PB10=SCL，PB11=SDA
 * 两个传感器AD0均接GND，地址均为0x68。
 *
 * 依赖原工程的 finger_mpu6050.c/.h。
 * 此版本暂不包含两块STM32之间的四指数据汇总。
 */
/* USER CODE END Header */

#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "finger_mpu6050.h"
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */

typedef struct {
    int32_t tilt_mdeg;
    int32_t signed_mdeg;
    int32_t flex_mdeg;

    uint8_t state;
    uint8_t valid;
    uint8_t stage;
    uint8_t hal_status;
    uint8_t reason;

    uint32_t sample_count;
} AppFingerDebug;

/* USER CODE END PTD */

/* USER CODE BEGIN PD */

#define APP_FINGER_COUNT       2u
#define APP_UART_PERIOD_MS     200u

#define APP_STAGE_COLLECT      5u
#define APP_STAGE_ERROR        6u
#define APP_STAGE_HOLD         7u

/* 保留你原代码实际使用的参数 */
#define APP_THRESHOLD1_DEG     85.0f
#define APP_THRESHOLD2_DEG     160.0f
#define APP_HYSTERESIS_DEG     5.0f

/* USER CODE END PD */

/* USER CODE BEGIN PV */

/* 两个独立的传感器对象 */
static FingerTracker fingers[APP_FINGER_COUNT];

/* 第0个接I2C1，第1个接I2C2 */
static I2C_HandleTypeDef *const finger_buses[APP_FINGER_COUNT] = {
    &hi2c1,
    &hi2c2
};

/* 串口输出中的手指名称 */
static const char *const finger_names[APP_FINGER_COUNT] = {
    "middle",
    "ring"
};

/* 只有完成初始化和校准，才开始读取 */
static uint8_t finger_ready[APP_FINGER_COUNT];

/*
 * CLion中观察：
 *
 * debug_fingers[0]：中指
 * debug_fingers[1]：无名指
 *
 * 角度单位为毫度，除以1000得到度。
 *
 * state：
 * 0：伸直
 * 1：半握拳
 * 2：握拳
 * 255：无有效档位
 *
 * stage：
 * 1：初始化
 * 2：初始化失败
 * 3：校准中
 * 4：校准失败
 * 5：正常采集
 * 6：更新错误，继续重试
 * 7：保留上次角度，继续重试
 */
volatile AppFingerDebug debug_fingers[APP_FINGER_COUNT];

volatile uint8_t debug_uart_status = HAL_OK;

/* 调试器中设置为1，重新初始化并校准两个手指 */
volatile uint8_t debug_command = 0;

/*
 * 原驱动中的诊断变量。
 * 每次操作完成后立即保存到对应手指的reason中。
 */
extern volatile uint8_t debug_update_reason;

/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

static void App_StartSensors(void);
static void App_UpdateSensor(uint32_t index);
static void App_SendTelemetry(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void App_StartSensors(void)
{
    uint32_t index;
    uint8_t any_initialized = 0u;

    /* 先分别初始化两个传感器 */
    for (index = 0u; index < APP_FINGER_COUNT; ++index) {
        volatile AppFingerDebug *d = &debug_fingers[index];
        FingerTracker *f = &fingers[index];

        HAL_StatusTypeDef result;

        finger_ready[index] = 0u;

        d->tilt_mdeg = 0;
        d->signed_mdeg = 0;
        d->flex_mdeg = 0;

        d->state = 255u;
        d->valid = 0u;
        d->sample_count = 0u;
        d->reason = 0u;
        d->stage = 1u;

        /*
         * 两个AD0均接GND。
         * 因为使用不同的I2C总线，所以地址都可以是0x68。
         */
        debug_update_reason = 0u;

        result = Finger_Init(
            f,
            finger_buses[index],
            0x68
        );

        d->reason = debug_update_reason;
        d->hal_status = (uint8_t)result;

        if (result != HAL_OK) {
            d->stage = 2u;

            /* 一个失败，继续处理另一个 */
            continue;
        }

        f->direction = 1.0f;

        f->threshold1_deg = APP_THRESHOLD1_DEG;
        f->threshold2_deg = APP_THRESHOLD2_DEG;
        f->hysteresis_deg = APP_HYSTERESIS_DEG;

        d->stage = 3u;
        any_initialized = 1u;
    }

    if (!any_initialized) {
        return;
    }

    /*
     * 手心朝上，中指和无名指都伸直。
     * 保持静止，直到两个传感器的校准都结束。
     *
     * 先统一等待2秒，再依次校准。
     */
    HAL_Delay(2000u);

    for (index = 0u; index < APP_FINGER_COUNT; ++index) {
        volatile AppFingerDebug *d = &debug_fingers[index];
        FingerTracker *f = &fingers[index];

        HAL_StatusTypeDef result;

        /* 跳过初始化失败的传感器 */
        if (d->stage != 3u) {
            continue;
        }

        debug_update_reason = 0u;

        result = Finger_Calibrate(f);

        d->reason = debug_update_reason;
        d->hal_status = (uint8_t)result;

        if (result != HAL_OK) {
            d->stage = 4u;
            continue;
        }

        d->tilt_mdeg =
            (int32_t)(f->angle_deg * 1000.0f);

        d->valid = f->valid;
        d->state = f->state;
        d->stage = APP_STAGE_COLLECT;

        finger_ready[index] = 1u;
    }
}

static void App_UpdateSensor(uint32_t index)
{
    FingerTracker *f;
    volatile AppFingerDebug *d;

    HAL_StatusTypeDef result;
    uint8_t reason;
    float relative;

    if (index >= APP_FINGER_COUNT ||
        !finger_ready[index]) {
        return;
    }

    f = &fingers[index];
    d = &debug_fingers[index];

    result = Finger_Update(f);

    /* 立即保存，避免被另一个传感器的操作覆盖 */
    reason = debug_update_reason;

    if (result == HAL_BUSY && f->calibrated) {
        /*
         * 未到采样时间时保持当前显示。
         * 如果本次样本不可靠，保留旧角度并标记阶段7。
         */
        if (!f->valid) {
            d->hal_status = (uint8_t)HAL_BUSY;
            d->reason = reason;

            d->valid = 0u;
            d->state = f->state;
            d->stage = APP_STAGE_HOLD;
        }

        return;
    }

    d->hal_status = (uint8_t)result;
    d->reason = reason;

    if (result != HAL_OK || !f->valid) {
        d->valid = 0u;
        d->state = 255u;
        d->stage = APP_STAGE_ERROR;

        return;
    }

    relative = f->angle_deg - f->zero_deg;

    d->tilt_mdeg =
        (int32_t)(f->angle_deg * 1000.0f);

    d->signed_mdeg =
        (int32_t)(relative * 1000.0f);

    d->flex_mdeg =
        (int32_t)(f->flex_deg * 1000.0f);

    d->state = f->state;
    d->valid = f->valid;

    d->sample_count++;
    d->stage = APP_STAGE_COLLECT;
}

static void App_SendTelemetry(void)
{
    static uint32_t last_send_ms = 0u;

    uint32_t now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - last_send_ms) <
        APP_UART_PERIOD_MS) {
        return;
        }

    last_send_ms = now_ms;

    /* 单位：毫度 */
    int32_t middle_mdeg = debug_fingers[0].flex_mdeg;
    int32_t ring_mdeg = debug_fingers[1].flex_mdeg;

    char message[96];

    int length = snprintf(
        message,
        sizeof(message),
        "middle_deg=%ld.%03ld,ring_deg=%ld.%03ld\r\n",
        (long)(middle_mdeg / 1000),
        (long)(middle_mdeg % 1000),
        (long)(ring_mdeg / 1000),
        (long)(ring_mdeg % 1000)
    );

    if (length > 0 && length < (int)sizeof(message)) {
        debug_uart_status = (uint8_t)HAL_UART_Transmit(
            &huart1,
            (uint8_t *)message,
            (uint16_t)length,
            20u
        );
    } else {
        debug_uart_status = (uint8_t)HAL_ERROR;
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
    MX_I2C2_Init();

    /* USER CODE BEGIN 2 */

    App_StartSensors();

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */

    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        uint32_t index;

        if (debug_command == 1u) {
            debug_command = 0u;
            App_StartSensors();
        }

        for (index = 0u;
             index < APP_FINGER_COUNT;
             ++index) {
            App_UpdateSensor(index);
        }

        App_SendTelemetry();
    }

    /* USER CODE END 3 */
}

/* 保留你原来的时钟配置 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE;

    RCC_OscInitStruct.HSEState =
        RCC_HSE_ON;

    RCC_OscInitStruct.HSEPredivValue =
        RCC_HSE_PREDIV_DIV1;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSE;

    RCC_OscInitStruct.PLL.PLLMUL =
        RCC_PLL_MUL9;

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

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2
        ) != HAL_OK) {
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