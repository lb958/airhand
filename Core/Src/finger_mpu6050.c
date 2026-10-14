#include "finger_mpu6050.h"
#include <math.h>
#include <string.h>

/* 校准数据：校准结束后不再更新 */
volatile float debug_cal_ax = 0.0f;
volatile float debug_cal_ay = 0.0f;
volatile float debug_cal_az = 0.0f;
volatile float debug_cal_gx = 0.0f;
volatile float debug_cal_norm2 = 0.0f;

volatile uint32_t debug_cal_samples = 0;
volatile float debug_cal_gyro_span = -1.0f;

/* 实时加速度，单位 g */
volatile float debug_run_ax = 0.0f;
volatile float debug_run_ay = 0.0f;
volatile float debug_run_az = 0.0f;

/*
 * 更新错误原因：
 * 0：正常
 * 1：采样间隔超过 50ms
 * 2：YZ 加速度投影过小
 * 3：X 轴角速度接近量程上限
 * 4：方向或分档参数错误
 * 5：I2C 读取失败
 */
volatile uint8_t debug_update_reason = 0;

/* 配置读回 */
volatile uint8_t debug_accel_config = 0xFF;
volatile uint8_t debug_gyro_config = 0xFF;
volatile uint8_t debug_config_read_status = HAL_ERROR;

/* 驱动内部成功更新快照 */
volatile uint32_t debug_driver_count = 0;
volatile int32_t debug_driver_direction = 0;
volatile int32_t debug_driver_relative_mdeg = 0;
volatile int32_t debug_driver_flex_mdeg = 0;

#define DEG_PER_RAD       57.2957795131f
#define SAMPLE_MS         10u
#define CONFIRM_MS        50u
#define I2C_TIMEOUT_MS    20u
#define CAL_SAMPLES       200u

/* 沿用当前模块的调试范围 */
#define ACC_MIN_G         0.70f
#define ACC_MAX_G         1.30f

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
} Sample;

static float wrap180(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }

    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

static int16_t be16(const uint8_t *p)
{
    int32_t value = ((int32_t)p[0] << 8) | p[1];

    if (value >= 32768) {
        value -= 65536;
    }

    return (int16_t)value;
}

static HAL_StatusTypeDef write_reg(
    FingerTracker *f,
    uint8_t reg,
    uint8_t value)
{
    return HAL_I2C_Mem_Write(
        f->bus,
        f->address,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        I2C_TIMEOUT_MS
    );
}

static HAL_StatusTypeDef read_sample(
    FingerTracker *f,
    Sample *s)
{
    uint8_t b[14];

    HAL_StatusTypeDef result = HAL_I2C_Mem_Read(
        f->bus,
        f->address,
        0x3B,
        I2C_MEMADD_SIZE_8BIT,
        b,
        sizeof b,
        I2C_TIMEOUT_MS
    );

    if (result != HAL_OK) {
        return result;
    }

    /* 加速度计 ±2g */
    s->ax = be16(b)     / 16384.0f;
    s->ay = be16(b + 2) / 16384.0f;
    s->az = be16(b + 4) / 16384.0f;

    /* 陀螺仪 ±250°/s */
    s->gx = be16(b + 8) / 131.0f;

    debug_run_ax = s->ax;
    debug_run_ay = s->ay;
    debug_run_az = s->az;

    return HAL_OK;
}

HAL_StatusTypeDef Finger_Init(
    FingerTracker *f,
    I2C_HandleTypeDef *bus,
    uint8_t address7)
{
    uint8_t id = 0;
    uint8_t config = 0;
    HAL_StatusTypeDef result;

    if (f == NULL || bus == NULL ||
        (address7 != 0x68 && address7 != 0x69)) {
        return HAL_ERROR;
    }

    memset(f, 0, sizeof *f);

    debug_driver_count = 0;
    debug_driver_direction = 0;
    debug_driver_relative_mdeg = 0;
    debug_driver_flex_mdeg = 0;

    debug_update_reason = 0;
    debug_cal_samples = 0;
    debug_cal_gyro_span = -1.0f;

    debug_run_ax = 0.0f;
    debug_run_ay = 0.0f;
    debug_run_az = 0.0f;

    debug_accel_config = 0xFF;
    debug_gyro_config = 0xFF;
    debug_config_read_status = HAL_ERROR;

    f->bus = bus;
    f->address = (uint16_t)address7 << 1;

    f->direction = 1.0f;
    f->threshold1_deg = 25.0f;
    f->threshold2_deg = 75.0f;
    f->hysteresis_deg = 5.0f;

    HAL_Delay(100);

    /* WHO_AM_I */
    result = HAL_I2C_Mem_Read(
        f->bus,
        f->address,
        0x75,
        I2C_MEMADD_SIZE_8BIT,
        &id,
        1,
        I2C_TIMEOUT_MS
    );

    if (result != HAL_OK) {
        return result;
    }

    /* 接受 MPU6050 / MPU6500 的设备标识 */
    if (id != 0x68 && id != 0x70) {
        return HAL_ERROR;
    }

    /* 复位 */
    result = write_reg(f, 0x6B, 0x80);

    if (result != HAL_OK) {
        return result;
    }

    HAL_Delay(100);

    const uint8_t settings[][2] = {
        {0x6B, 0x01}, /* 唤醒，选择时钟 */
        {0x6C, 0x00}, /* 启用各测量轴 */
        {0x1A, 0x06}, /* 低通滤波 */
        {0x19, 0x09}, /* 采样分频 */
        {0x1B, 0x00}, /* 陀螺仪 ±250°/s */
        {0x1C, 0x00}  /* 加速度计 ±2g */
    };

    for (unsigned i = 0;
         i < sizeof settings / sizeof settings[0];
         ++i) {
        result = write_reg(
            f,
            settings[i][0],
            settings[i][1]
        );

        if (result != HAL_OK) {
            return result;
        }
    }

    /* MPU6500 加速度计滤波配置 */
    if (id == 0x70) {
        result = write_reg(f, 0x1D, 0x06);

        if (result != HAL_OK) {
            return result;
        }
    }

    /* 读回加速度计配置 */
    result = HAL_I2C_Mem_Read(
        f->bus,
        f->address,
        0x1C,
        I2C_MEMADD_SIZE_8BIT,
        &config,
        1,
        I2C_TIMEOUT_MS
    );

    debug_config_read_status = (uint8_t)result;

    if (result != HAL_OK) {
        return result;
    }

    debug_accel_config = config;

    /* 读回陀螺仪配置 */
    result = HAL_I2C_Mem_Read(
        f->bus,
        f->address,
        0x1B,
        I2C_MEMADD_SIZE_8BIT,
        &config,
        1,
        I2C_TIMEOUT_MS
    );

    debug_config_read_status = (uint8_t)result;

    if (result != HAL_OK) {
        return result;
    }

    debug_gyro_config = config;

    HAL_Delay(1000);

    return HAL_OK;
}

HAL_StatusTypeDef Finger_Calibrate(FingerTracker *f)
{
    Sample s;
    Sample first = {0};

    float sum_gx = 0.0f;
    float sum_ay = 0.0f;
    float sum_az = 0.0f;

    float min_gx = 1000.0f;
    float max_gx = -1000.0f;

    debug_cal_samples = 0;
    debug_cal_gyro_span = -1.0f;
    debug_update_reason = 0;

    if (f == NULL || f->bus == NULL) {
        return HAL_ERROR;
    }

    f->valid = 0;
    f->calibrated = 0;

    /* 食指伸直、手掌固定，整个校准过程保持静止 */
    for (unsigned i = 0; i < CAL_SAMPLES; ++i) {
        HAL_StatusTypeDef result = read_sample(f, &s);

        if (result != HAL_OK) {
            return result;
        }

        if (i == 0) {
            first = s;
        }

        const float norm2 =
            s.ax * s.ax +
            s.ay * s.ay +
            s.az * s.az;

        debug_cal_ax = s.ax;
        debug_cal_ay = s.ay;
        debug_cal_az = s.az;
        debug_cal_gx = s.gx;
        debug_cal_norm2 = norm2;
        debug_cal_samples = i + 1;

        if (norm2 < ACC_MIN_G * ACC_MIN_G ||
            norm2 > ACC_MAX_G * ACC_MAX_G ||
            s.ay * s.ay + s.az * s.az < 0.30f * 0.30f ||
            fabsf(s.gx) > 244.0f ||
            fabsf(s.ax - first.ax) > 0.10f ||
            fabsf(s.ay - first.ay) > 0.10f ||
            fabsf(s.az - first.az) > 0.10f) {
            return HAL_ERROR;
        }

        if (s.gx < min_gx) {
            min_gx = s.gx;
        }

        if (s.gx > max_gx) {
            max_gx = s.gx;
        }

        sum_gx += s.gx;
        sum_ay += s.ay;
        sum_az += s.az;

        HAL_Delay(SAMPLE_MS);
    }

    debug_cal_gyro_span = max_gx - min_gx;

    if (debug_cal_gyro_span > 8.0f) {
        return HAL_ERROR;
    }

    /* 静止陀螺仪零偏 */
    f->gyro_bias_dps = sum_gx / (float)CAL_SAMPLES;

    /* 绕 X 轴的角度，支持零点接近 ±180° */
    f->zero_deg = atan2f(sum_ay, sum_az) * DEG_PER_RAD;
    f->angle_deg = f->zero_deg;

    f->flex_deg = 0.0f;
    f->state = 0;
    f->candidate = 0;

    f->last_ms = HAL_GetTick();
    f->candidate_ms = f->last_ms;

    debug_driver_count = 0;
    debug_driver_direction = (int32_t)f->direction;
    debug_driver_relative_mdeg = 0;
    debug_driver_flex_mdeg = 0;

    f->valid = 1;
    f->calibrated = 1;

    return HAL_OK;
}

static void classify(FingerTracker *f, uint32_t now)
{
    const float a = f->flex_deg;
    const float h = f->hysteresis_deg;
    const float t1 = f->threshold1_deg;
    const float t2 = f->threshold2_deg;

    uint8_t next = f->state;

    switch (f->state) {
        case 0:
            if (a >= t2 + h) {
                next = 2;
            } else if (a >= t1 + h) {
                next = 1;
            }
            break;

        case 1:
            if (a >= t2 + h) {
                next = 2;
            } else if (a <= t1 - h) {
                next = 0;
            }
            break;

        case 2:
            if (a <= t1 - h) {
                next = 0;
            } else if (a <= t2 - h) {
                next = 1;
            }
            break;

        default:
            next = 0;
            break;
    }

    if (next != f->candidate) {
        f->candidate = next;
        f->candidate_ms = now;
    }

    /* 新档位连续保持至少 50ms */
    if (next != f->state &&
        (uint32_t)(now - f->candidate_ms) >= CONFIRM_MS) {
        f->state = next;
    }
}

HAL_StatusTypeDef Finger_Update(FingerTracker *f)
{
    if (debug_update_reason != 0) {
        f->valid = 0;

        /* 侧翻时保留原角度、档位和校准状态 */
        if (debug_update_reason == 2) {
            f->last_ms = HAL_GetTick();
            f->candidate = f->state;
            f->candidate_ms = f->last_ms;

            return HAL_BUSY;
        }

        /* 其他错误仍按原逻辑处理 */
        f->calibrated = 0;
        return HAL_ERROR;
    }

    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - f->last_ms) < SAMPLE_MS) {
        return HAL_BUSY;
    }

    Sample s;
    HAL_StatusTypeDef result = read_sample(f, &s);

    if (result != HAL_OK) {
        debug_update_reason = 5;
        f->valid = 0;
        f->calibrated = 0;
        return result;
    }

    now = HAL_GetTick();

    const uint32_t elapsed =
        (uint32_t)(now - f->last_ms);

    debug_update_reason = 0;

    if (elapsed > 50u) {
        debug_update_reason = 1;
    } else if (
        s.ay * s.ay + s.az * s.az < 0.25f * 0.25f) {
        debug_update_reason = 2;
    } else if (fabsf(s.gx) > 244.0f) {
        debug_update_reason = 3;
    } else if (
        (f->direction != 1.0f && f->direction != -1.0f) ||
        f->hysteresis_deg < 0.0f ||
        f->threshold1_deg <= f->hysteresis_deg ||
        f->threshold2_deg - f->threshold1_deg
            <= 2.0f * f->hysteresis_deg ||
        f->threshold2_deg + f->hysteresis_deg >= 180.0f) {
        debug_update_reason = 4;
    }

    if (debug_update_reason != 0) {
        f->valid = 0;
        f->calibrated = 0;
        return HAL_ERROR;
    }

    f->last_ms = now;
    const float dt = elapsed * 0.001f;

    /*
     * 连续角度积分：
     * 累计角度不再折回 ±180°。
     */
    float predicted = f->angle_deg +
        (s.gx - f->gyro_bias_dps) * dt;

    const float norm2 =
        s.ax * s.ax +
        s.ay * s.ay +
        s.az * s.az;

    if (norm2 >= ACC_MIN_G * ACC_MIN_G &&
        norm2 <= ACC_MAX_G * ACC_MAX_G) {
        const float acc_angle =
            atan2f(s.ay, s.az) * DEG_PER_RAD;

        const float alpha = 0.5f / (0.5f + dt);

        /*
         * 只把修正误差折回 ±180°。
         * 累计角度本身保持连续。
         */
        predicted += (1.0f - alpha) *
            wrap180(acc_angle - predicted);
    }

    f->angle_deg = predicted;

    /*
     * 相对伸直姿态的连续角度。
     * 允许超过 180°，不再使用 wrap180。
     */
    const float relative =
        f->angle_deg - f->zero_deg;

    float flex = f->direction * relative;

    if (flex < 0.0f) {
        flex = 0.0f;
    }

    /*
     * 超过 180°仍按最大弯曲判断。
     * 例如 relative=200°，flex=180°。
     */
    if (flex > 180.0f) {
        flex = 180.0f;
    }

    f->flex_deg = flex;

    debug_driver_direction = (int32_t)f->direction;

    debug_driver_relative_mdeg =
        (int32_t)(relative * 1000.0f);

    debug_driver_flex_mdeg =
        (int32_t)(f->flex_deg * 1000.0f);

    debug_driver_count++;

    classify(f, now);

    f->valid = 1;

    return HAL_OK;
}