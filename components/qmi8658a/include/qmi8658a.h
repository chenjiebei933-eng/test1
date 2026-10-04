/* Copyright (c) 2022, 2026 lewis he. SPDX-License-Identifier: MIT */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 驱动管理的设备句柄，由初始化创建、反初始化释放。 */
typedef struct qmi8658a_device *qmi8658a_handle_t;

typedef struct {
    /* 传感器 X/Y/Z 三轴加速度，单位 g。 */
    float ax_g;
    float ay_g;
    float az_g;
    /* 传感器 X/Y/Z 三轴角速度，单位 °/s。 */
    float gx_dps;
    float gy_dps;
    float gz_dps;
} qmi8658a_sample_t;

/**
 * 依次探测 0x6A、0x6B 并配置六轴采样，失败时 *out_handle 为 NULL。
 * 总线归调用方管理，驱动仅管理设备句柄；总线必须使用同步事务。
 * 量程为 ±4 g、±512 °/s，输出数据率约 112.1 Hz，使用 SyncSample 锁存。
 * 接口不支持并发调用，请由单个任务调用或在外部串行化访问。
 */
esp_err_t qmi8658a_init(i2c_master_bus_handle_t bus,
                      qmi8658a_handle_t *out_handle);

/** 读取同一采样时刻的完整六轴数据，失败时保持输出参数不变。 */
esp_err_t qmi8658a_read(qmi8658a_handle_t handle,
                      qmi8658a_sample_t *out_sample);

/** 移除设备并释放句柄；即使传感器关闭失败，也会执行清理。 */
esp_err_t qmi8658a_deinit(qmi8658a_handle_t handle);

#ifdef __cplusplus
}
#endif
