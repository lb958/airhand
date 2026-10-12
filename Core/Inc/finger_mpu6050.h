#ifndef FINGER_MPU6050_H
#define FINGER_MPU6050_H

#include "stm32f1xx_hal.h"

/* 固定手掌、缓慢绕 X 轴弯曲的单指倾角测试 */
typedef struct {
    I2C_HandleTypeDef *bus;
    uint16_t address;

    float angle_deg;
    float zero_deg;
    float gyro_bias_dps;
    float flex_deg;
    float direction;

    float threshold1_deg;
    float threshold2_deg;
    float hysteresis_deg;

    uint32_t last_ms;
    uint32_t candidate_ms;

    uint8_t state;
    uint8_t candidate;
    uint8_t calibrated;
    uint8_t valid;
} FingerTracker;

HAL_StatusTypeDef Finger_Init(
    FingerTracker *f,
    I2C_HandleTypeDef *bus,
    uint8_t address7
);

HAL_StatusTypeDef Finger_Calibrate(FingerTracker *f);
HAL_StatusTypeDef Finger_Update(FingerTracker *f);

/* 驱动内部成功更新记录 */
extern volatile uint32_t debug_driver_count;
extern volatile int32_t debug_driver_direction;
extern volatile int32_t debug_driver_relative_mdeg;
extern volatile int32_t debug_driver_flex_mdeg;

#endif