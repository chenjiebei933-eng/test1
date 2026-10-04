#include <inttypes.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "qmi8658a.h"

#define BOARD_POWER_HOLD_GPIO GPIO_NUM_6  /* MTCK：拉高 MCU_EN，保持板上电源。 */
#define BOARD_IMU_INT1_GPIO   GPIO_NUM_4  /* MTMS：连接 QMI8658A 的 INT1。 */
#define BOARD_I2C_SDA_GPIO    GPIO_NUM_0  /* I2C 数据线。 */
#define BOARD_I2C_SCL_GPIO    GPIO_NUM_1  /* I2C 时钟线。 */
#define OUTPUT_PERIOD_MS     300         /* 串口输出间隔，单位 ms。 */
#define RECOVERY_PERIOD_MS   1000        /* 通信失败后的重试间隔，单位 ms。 */

static const char *TAG = "imu_demo";

static void board_init(void)
{
    /* 先预置高电平，再使能 MCU_EN 输出，保持供电。 */
    ESP_ERROR_CHECK(gpio_set_level(BOARD_POWER_HOLD_GPIO, 1));
    const gpio_config_t power_hold_config = {
        .pin_bit_mask = 1ULL << BOARD_POWER_HOLD_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    /* 将 MTCK 从复位时的复用功能切换为 GPIO 输出。 */
    ESP_ERROR_CHECK(gpio_config(&power_hold_config));

    /* INT1 配置为输入；本示例轮询采样，不启用中断。 */
    const gpio_config_t int1_config = {
        .pin_bit_mask = 1ULL << BOARD_IMU_INT1_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&int1_config));
}

void app_main(void)
{
    /* 先保持板上电源；关闭标准输出缓冲，及时发送串口数据。 */
    board_init();
    setvbuf(stdout, NULL, _IONBF, 0);

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BOARD_I2C_SDA_GPIO,
        .scl_io_num = BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        /* SDA、SCL 已有外部 3.3 V 上拉，无需内部上拉。 */
        .flags.enable_internal_pullup = false,
    };
    i2c_master_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));
    ESP_LOGI(TAG, "SDA=GPIO0 SCL=GPIO1 I2C=400kHz INT1=GPIO4 (polling)");
    ESP_LOGI(TAG, "UART0 TX=GPIO20 RX=GPIO19 115200 baud; output every %d ms",
             OUTPUT_PERIOD_MS);

    qmi8658a_handle_t imu = NULL;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        if (imu == NULL) {
            /* 自动识别并配置传感器；初始化失败后等待 1 秒重试。 */
            const esp_err_t err = qmi8658a_init(bus, &imu);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "IMU initialization failed: %s; retry in 1 s",
                         esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(RECOVERY_PERIOD_MS));
                continue;
            }
            /* 初始化或恢复成功后，重新建立固定周期计时基准。 */
            last_wake = xTaskGetTickCount();
        }

        /* 按固定 300 ms 周期读取，避免累积本次处理耗时。 */
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(OUTPUT_PERIOD_MS));
        qmi8658a_sample_t sample;
        const esp_err_t err = qmi8658a_read(imu, &sample);
        if (err != ESP_OK) {
            /* 读取失败则释放句柄并重试初始化，不输出无效样本。 */
            ESP_LOGE(TAG, "IMU read failed: %s; retry initialization in 1 s",
                     esp_err_to_name(err));
            const esp_err_t cleanup_err = qmi8658a_deinit(imu);
            imu = NULL;
            if (cleanup_err != ESP_OK) {
                ESP_LOGW(TAG, "IMU cleanup: %s", esp_err_to_name(cleanup_err));
            }
            vTaskDelay(pdMS_TO_TICKS(RECOVERY_PERIOD_MS));
            continue;
        }

        /* 启动时间用 ms 表示；加速度单位 g，角速度单位 °/s，每行一组。 */
        const int64_t timestamp_ms = esp_timer_get_time() / 1000;
        printf("t=%" PRId64 "ms Acc[g]=(%.3f,%.3f,%.3f) "
               "Gyro[dps]=(%.3f,%.3f,%.3f)\n",
               timestamp_ms, sample.ax_g, sample.ay_g, sample.az_g,
               sample.gx_dps, sample.gy_dps, sample.gz_dps);
    }
}
