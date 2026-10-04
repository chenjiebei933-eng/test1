/*
 * Copyright (c) 2022 lewis he
 * Copyright (c) 2026 lewis he
 * SPDX-License-Identifier: MIT
 *
 * QMI8658A register configuration and CTRL9/locking logic adapted from:
 * https://github.com/lewisxhe/SensorLib/tree/477fc682e9ed30ced39774f3b7e93a1504968884
 * src/sensor/imu/qmi8658/SensorQMI8658.cpp and SensorQMI8658_Reg.hpp.
 * See LICENSE and README.md for the upstream license and port details.
 */

#include "qmi8658a.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 芯片寄存器地址、状态位及各阶段的超时时间。 */
enum {
    REG_WHO_AM_I = 0x00,
    REG_REVISION = 0x01,
    REG_CTRL1 = 0x02,
    REG_CTRL2 = 0x03,
    REG_CTRL3 = 0x04,
    REG_CTRL5 = 0x06,
    REG_CTRL7 = 0x08,
    REG_CTRL8 = 0x09,
    REG_CTRL9 = 0x0A,
    REG_CAL1_L = 0x0B,
    REG_FIFO_CTRL = 0x14,
    REG_STATUS_INT = 0x2D,
    REG_AX_L = 0x35, /* 六轴数据起始地址：加速度 X 轴低字节。 */
    REG_GZ_H = 0x40, /* 六轴数据末尾：读取后自动释放样本锁存。 */
    REG_RESET_RESULT = 0x4D,
    REG_RESET = 0x60,
    CHIP_ID = 0x05,
    RESET_COMMAND = 0xB0,
    CMD_AHB_CLOCK_GATING = 0x12,
    STATUS_AVAILABLE = 0x01,    /* 有新样本可读。 */
    STATUS_LOCKED = 0x02,       /* 样本已锁存，读取期间不会被更新。 */
    STATUS_COMMAND_DONE = 0x80, /* CTRL9 命令已执行完成。 */
    TRANSFER_TIMEOUT_MS = 20,
    COMMAND_TIMEOUT_MS = 100,
    SAMPLE_TIMEOUT_MS = 100,
    RESET_TIMEOUT_MS = 500,
    RESET_DELAY_MS = 15,
    DATA_LOCK_DELAY_US = 12,
};

struct qmi8658a_device {
    i2c_master_dev_handle_t i2c; /* 本驱动管理的 I2C 设备句柄。 */
    uint8_t address;            /* 实际探测到的 7 位设备地址。 */
};

static const char *TAG = "qmi8658a";

static void delay_ms(uint32_t milliseconds)
{
    /* 毫秒向上换算为调度节拍；最小实际等待时间由调用处另行校验。 */
    TickType_t ticks = (milliseconds + portTICK_PERIOD_MS - 1) /
                       portTICK_PERIOD_MS;
    vTaskDelay(ticks > 0 ? ticks : 1);
}

/* 先发送寄存器地址，再连续读取数据；通信错误直接交给调用者。 */
static esp_err_t read_register(qmi8658a_handle_t handle, uint8_t reg,
                               uint8_t *data, size_t length, int timeout_ms)
{
    return i2c_master_transmit_receive(handle->i2c, &reg, 1, data, length,
                                      timeout_ms);
}

static esp_err_t write_register(qmi8658a_handle_t handle, uint8_t reg,
                                uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(handle->i2c, data, sizeof(data),
                               TRANSFER_TIMEOUT_MS);
}

/* 等待指定状态位置位或清零，使用绝对截止时间避免无限等待。 */
static esp_err_t wait_status(qmi8658a_handle_t handle, uint8_t reg,
                             uint8_t mask, bool set, int64_t deadline_us,
                             uint8_t *out_status)
{
    while (true) {
        int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0) {
            return ESP_ERR_TIMEOUT;
        }
        int timeout_ms = (int)((remaining_us + 999) / 1000);
        if (timeout_ms > TRANSFER_TIMEOUT_MS) {
            timeout_ms = TRANSFER_TIMEOUT_MS;
        }
        uint8_t status;
        esp_err_t err = read_register(handle, reg, &status, 1, timeout_ms);
        if (err != ESP_OK) {
            return err;
        }
        /* 超时后才返回的应答也不能判定为成功。 */
        if (esp_timer_get_time() >= deadline_us) {
            return ESP_ERR_TIMEOUT;
        }
        if (((status & mask) != 0) == set) {
            if (out_status != NULL) {
                *out_status = status;
            }
            return ESP_OK;
        }
        delay_ms(1);
    }
}

/* CTRL9 握手：发送命令、等待完成、写入确认、等待完成位清零。 */
static esp_err_t send_command(qmi8658a_handle_t handle, uint8_t command)
{
    esp_err_t err = write_register(handle, REG_CTRL9, command);
    if (err != ESP_OK) {
        return err;
    }
    const int64_t deadline = esp_timer_get_time() + COMMAND_TIMEOUT_MS * 1000LL;
    err = wait_status(handle, REG_STATUS_INT, STATUS_COMMAND_DONE, true,
                      deadline, NULL);
    if (err != ESP_OK) {
        return err;
    }
    err = write_register(handle, REG_CTRL9, 0x00);
    if (err != ESP_OK) {
        return err;
    }
    return wait_status(handle, REG_STATUS_INT, STATUS_COMMAND_DONE, false,
                       deadline, NULL);
}

/* 复位芯片，并配置量程、采样率和同步锁存读取模式。 */
static esp_err_t configure_sensor(qmi8658a_handle_t handle)
{
    esp_err_t err = write_register(handle, REG_RESET, RESET_COMMAND);
    if (err != ESP_OK) {
        return err;
    }
    const int64_t reset_start = esp_timer_get_time();
    const int64_t reset_deadline = reset_start + RESET_TIMEOUT_MS * 1000LL;
    const int64_t reset_ready_at = reset_start + RESET_DELAY_MS * 1000LL;
    delay_ms(RESET_DELAY_MS);
    /* 调度延时可能提前返回，确保复位后实际等待至少 15ms。 */
    while (esp_timer_get_time() < reset_ready_at) {
        delay_ms(1);
    }
    err = wait_status(handle, REG_RESET_RESULT, 0x80, true, reset_deadline, NULL);
    if (err != ESP_OK) {
        return err;
    }

    /* 地址自动递增、小端读取；两个中断引脚均保持高阻。 */
    const uint8_t configuration[][2] = {
        {REG_CTRL1, 0x40},
        {REG_CTRL7, 0x00}, /* 配置期间关闭加速度计和陀螺仪。 */
        {REG_CTRL8, 0x80}, /* 通过 STATUSINT 的 bit7 检查命令完成。 */
        {REG_FIFO_CTRL, 0x00}, /* 关闭 FIFO，直接读取数据寄存器。 */
        {REG_CTRL2, 0x16}, /* 加速度量程 ±4g，采样率档位 6。 */
        {REG_CTRL3, 0x56}, /* 角速度量程 ±512°/s，采样率档位 6。 */
        {REG_CTRL5, 0x00}, /* 关闭可选低通滤波器。 */
        {REG_CAL1_L, 0x01},
    };
    for (size_t i = 0; i < sizeof(configuration) / sizeof(configuration[0]); ++i) {
        err = write_register(handle, configuration[i][0], configuration[i][1]);
        if (err != ESP_OK) {
            return err;
        }
    }
    /* I2C 同步锁存读取要求先关闭芯片内部 AHB 时钟门控。 */
    err = send_command(handle, CMD_AHB_CLOCK_GATING);
    if (err != ESP_OK) {
        return err;
    }
    /* 同时开启六轴传感器和 SyncSample 锁存模式。 */
    err = write_register(handle, REG_CTRL7, 0x83);
    if (err != ESP_OK) {
        return err;
    }

    /* 回读使能寄存器，确认配置已生效。 */
    uint8_t enabled;
    err = read_register(handle, REG_CTRL7, &enabled, 1, TRANSFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    return enabled == 0x83 ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t qmi8658a_init(i2c_master_bus_handle_t bus,
                      qmi8658a_handle_t *out_handle)
{
    if (out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_handle = NULL;
    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    qmi8658a_handle_t handle = calloc(1, sizeof(*handle));
    if (handle == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* 本板 SA0 接高电平，优先探测 0x6A，再尝试备用地址。 */
    const uint8_t addresses[] = {0x6A, 0x6B};
    esp_err_t err = ESP_ERR_NOT_FOUND;
    esp_err_t last_probe_error = ESP_ERR_NOT_FOUND;
    for (size_t i = 0; i < sizeof(addresses); ++i) {
        err = i2c_master_probe(bus, addresses[i], TRANSFER_TIMEOUT_MS);
        if (err != ESP_OK) {
            if (err != ESP_ERR_NOT_FOUND) {
                last_probe_error = err;
            }
            continue;
        }
        const i2c_device_config_t config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addresses[i],
            .scl_speed_hz = 400000,
        };
        err = i2c_master_bus_add_device(bus, &config, &handle->i2c);
        if (err != ESP_OK) {
            free(handle);
            return err;
        }
        /* 地址应答后仍需核对芯片 ID，避免误认同总线的其他设备。 */
        uint8_t id;
        err = read_register(handle, REG_WHO_AM_I, &id, 1, TRANSFER_TIMEOUT_MS);
        if (err == ESP_OK && id == CHIP_ID) {
            handle->address = addresses[i];
            break;
        }
        if (err != ESP_OK) {
            last_probe_error = err;
        }
        esp_err_t remove_err = i2c_master_bus_rm_device(handle->i2c);
        handle->i2c = NULL;
        if (remove_err != ESP_OK) {
            free(handle);
            return remove_err;
        }
    }
    if (handle->i2c == NULL) {
        free(handle);
        return last_probe_error;
    }

    err = configure_sensor(handle);
    uint8_t revision = 0;
    if (err == ESP_OK) {
        err = read_register(handle, REG_REVISION, &revision, 1, TRANSFER_TIMEOUT_MS);
    }
    if (err != ESP_OK) {
        /* 初始化失败时清理已分配资源，并保留原始错误。 */
        esp_err_t cleanup_err = qmi8658a_deinit(handle);
        if (cleanup_err != ESP_OK) {
            ESP_LOGW(TAG, "Initialization cleanup: %s", esp_err_to_name(cleanup_err));
        }
        return err;
    }
    ESP_LOGI(TAG, "Address 0x%02X, revision 0x%02X, +/-4 g, +/-512 deg/s, "
                  "112.1 Hz, SyncSample", handle->address, revision);
    *out_handle = handle;
    return ESP_OK;
}

static int16_t decode_le16(const uint8_t *bytes)
{
    uint16_t value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    /* 显式还原有符号 16 位数值，正确处理负加速度和负角速度。 */
    int32_t signed_value = value >= 0x8000 ? (int32_t)value - 0x10000 : value;
    return (int16_t)signed_value;
}

/* 读取失败后尝试读末字节解锁，避免下一次采样一直被锁住。 */
static void unlock_after_error(qmi8658a_handle_t handle)
{
    uint8_t ignored;
    esp_err_t err = read_register(handle, REG_GZ_H, &ignored, 1,
                                  TRANSFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not release sample lock: %s", esp_err_to_name(err));
    }
}

esp_err_t qmi8658a_read(qmi8658a_handle_t handle,
                      qmi8658a_sample_t *out_sample)
{
    if (handle == NULL || out_sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t status;
    const int64_t deadline = esp_timer_get_time() + SAMPLE_TIMEOUT_MS * 1000LL;
    esp_err_t err = wait_status(handle, REG_STATUS_INT, STATUS_AVAILABLE,
                                true, deadline, &status);
    if (err != ESP_OK) {
        /* 状态读取虽失败，芯片可能已触发锁存，仍需尝试解锁。 */
        unlock_after_error(handle);
        return err;
    }
    if ((status & STATUS_LOCKED) == 0) {
        /* 样本可用但锁存尚未完成，等待该采样率对应的锁存时间。 */
        esp_rom_delay_us(DATA_LOCK_DELAY_US);
    }

    uint8_t data[12];
    err = read_register(handle, REG_AX_L, data, sizeof(data), TRANSFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        unlock_after_error(handle);
        return err;
    }
    /* 一次读完六轴，末字节 GZ_H 自动解锁；按量程换算为 g 和 °/s。 */
    const qmi8658a_sample_t sample = {
        .ax_g = decode_le16(&data[0]) / 8192.0f,
        .ay_g = decode_le16(&data[2]) / 8192.0f,
        .az_g = decode_le16(&data[4]) / 8192.0f,
        .gx_dps = decode_le16(&data[6]) / 64.0f,
        .gy_dps = decode_le16(&data[8]) / 64.0f,
        .gz_dps = decode_le16(&data[10]) / 64.0f,
    };
    /* 仅在整组读取成功后更新输出，失败时保留调用者原有数据。 */
    *out_sample = sample;
    return ESP_OK;
}

esp_err_t qmi8658a_deinit(qmi8658a_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    /* 尝试关闭传感器；即使关停失败，也继续移除设备并释放句柄。 */
    esp_err_t shutdown_err = write_register(handle, REG_CTRL7, 0x00);
    esp_err_t remove_err = i2c_master_bus_rm_device(handle->i2c);
    free(handle);
    return remove_err != ESP_OK ? remove_err : shutdown_err;
}
