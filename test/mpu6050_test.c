/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "mpu6050.h"
#include "esp_system.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 26      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "mpu6050 test";
i2c_bus_handle_t i2c_bus = NULL;
static mpu6050_handle_t mpu6050 = NULL;

/**
 * @brief i2c master initialization
 */
static void i2c_bus_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = IIC_SDA_NUM,
        .scl_io_num = IIC_SCL_NUM,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {0},
        .clk_flags = 0,
    };
    conf.master.clk_speed = 400000,
    i2c_bus = i2c_bus_create(IIC_MASTER_NUM, &conf);
}

/**
 * @brief i2c master initialization
 */
static void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;
    mpu6050 = mpu6050_create(i2c_bus, MPU6050_I2C_ADDRESS);
    ESP_ERROR_CHECK(mpu6050_init(mpu6050));
    ESP_ERROR_CHECK(mpu6050_enable_motiondetection(mpu6050, 1, 20));
}

void app_main(void)
{
    i2c_bus_init();
    i2c_sensor_mpu6050_init();
    while (1)
    {
        bool active = false;
        mpu6050_getMotionInterruptStatus(mpu6050, &active);
        if (active)
        {
            ESP_LOGI(TAG, "mpu6050 int");
        }
        vTaskDelay(1000);
    }
}