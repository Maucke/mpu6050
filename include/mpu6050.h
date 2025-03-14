/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief MPU6050 driver
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "i2c_bus.h"
#include "driver/gpio.h"

#define MPU6050_I2C_ADDRESS 0x68u   /*!< I2C address with AD0 pin low */
#define MPU6050_I2C_ADDRESS_1 0x69u /*!< I2C address with AD0 pin high */
#define MPU6050_WHO_AM_I_VAL 0x68u

#define MPU6050_SELF_TEST_X \
    0x0D ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Y \
    0x0E ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Z \
    0x0F ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_A \
    0x10                         ///< Self test factory calibrated values register
#define MPU6050_SMPLRT_DIV 0x19  ///< sample rate divisor register
#define MPU6050_CONFIG 0x1A      ///< General configuration register
#define MPU6050_GYRO_CONFIG 0x1B ///< Gyro specfic configuration register
#define MPU6050_ACCEL_CONFIG \
    0x1C                               ///< Accelerometer specific configration register
#define MPU6050_INT_PIN_CONFIG 0x37    ///< Interrupt pin configuration register
#define MPU6050_INT_ENABLE 0x38        ///< Interrupt enable configuration register
#define MPU6050_INT_STATUS 0x3A        ///< Interrupt status register
#define MPU6050_WHO_AM_I 0x75          ///< Divice ID register
#define MPU6050_SIGNAL_PATH_RESET 0x68 ///< Signal path reset register
#define MPU6050_USER_CTRL 0x6A         ///< FIFO and I2C Master control register
#define MPU6050_PWR_MGMT_1 0x6B        ///< Primary power/sleep control register
#define MPU6050_PWR_MGMT_2 0x6C        ///< Secondary power/sleep control register
#define MPU6050_TEMP_H 0x41            ///< Temperature data high byte register
#define MPU6050_TEMP_L 0x42            ///< Temperature data low byte register
#define MPU6050_GYRO_OUT 0x43u
#define MPU6050_ACCEL_OUT 0x3B ///< base address for sensor data reads
#define MPU6050_MOT_THR 0x1F   ///< Motion detection threshold bits [7:0]
#define MPU6050_MOT_DUR \
    0x20 ///< Duration counter threshold for motion int. 1 kHz rate, LSB = 1 ms

    typedef enum
    {
        ACCE_FS_2G = 0,  /*!< Accelerometer full scale range is +/- 2g */
        ACCE_FS_4G = 1,  /*!< Accelerometer full scale range is +/- 4g */
        ACCE_FS_8G = 2,  /*!< Accelerometer full scale range is +/- 8g */
        ACCE_FS_16G = 3, /*!< Accelerometer full scale range is +/- 16g */
    } mpu6050_acce_fs_t;

    typedef enum
    {
        GYRO_FS_250DPS = 0,  /*!< Gyroscope full scale range is +/- 250 degree per sencond */
        GYRO_FS_500DPS = 1,  /*!< Gyroscope full scale range is +/- 500 degree per sencond */
        GYRO_FS_1000DPS = 2, /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
        GYRO_FS_2000DPS = 3, /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
    } mpu6050_gyro_fs_t;

    typedef enum
    {
        INTERRUPT_PIN_ACTIVE_HIGH = 0, /*!< The mpu6050 sets its INT pin HIGH on interrupt */
        INTERRUPT_PIN_ACTIVE_LOW = 1   /*!< The mpu6050 sets its INT pin LOW on interrupt */
    } mpu6050_int_pin_active_level_t;

    typedef enum
    {
        INTERRUPT_PIN_PUSH_PULL = 0, /*!< The mpu6050 configures its INT pin as push-pull */
        INTERRUPT_PIN_OPEN_DRAIN = 1 /*!< The mpu6050 configures its INT pin as open drain*/
    } mpu6050_int_pin_mode_t;

    typedef enum
    {
        INTERRUPT_LATCH_50US = 0,         /*!< The mpu6050 produces a 50 microsecond pulse on interrupt */
        INTERRUPT_LATCH_UNTIL_CLEARED = 1 /*!< The mpu6050 latches its INT pin to its active level, until interrupt is cleared */
    } mpu6050_int_latch_t;

    typedef enum
    {
        INTERRUPT_CLEAR_ON_ANY_READ = 0,   /*!< INT_STATUS register bits are cleared on any register read */
        INTERRUPT_CLEAR_ON_STATUS_READ = 1 /*!< INT_STATUS register bits are cleared only by reading INT_STATUS value*/
    } mpu6050_int_clear_t;

    typedef enum
    {
        MPU6050_BAND_260_HZ, ///< Docs imply this disables the filter
        MPU6050_BAND_184_HZ, ///< 184 Hz
        MPU6050_BAND_94_HZ,  ///< 94 Hz
        MPU6050_BAND_44_HZ,  ///< 44 Hz
        MPU6050_BAND_21_HZ,  ///< 21 Hz
        MPU6050_BAND_10_HZ,  ///< 10 Hz
        MPU6050_BAND_5_HZ,   ///< 5 Hz
    } mpu6050_bandwidth_t;

    /**
     * @brief Accelerometer high pass filter options
     *
     * Allowed values for `setHighPassFilter`.
     */
    typedef enum
    {
        MPU6050_HIGHPASS_DISABLE,
        MPU6050_HIGHPASS_5_HZ,
        MPU6050_HIGHPASS_2_5_HZ,
        MPU6050_HIGHPASS_1_25_HZ,
        MPU6050_HIGHPASS_0_63_HZ,
        MPU6050_HIGHPASS_UNUSED,
        MPU6050_HIGHPASS_HOLD,
    } mpu6050_highpass_t;

    /**
     * @brief Accelerometer range options
     *
     * Allowed values for `setAccelerometerRange`.
     */
    typedef enum
    {
        MPU6050_RANGE_2_G = 0b00,  ///< +/- 2g (default value)
        MPU6050_RANGE_4_G = 0b01,  ///< +/- 4g
        MPU6050_RANGE_8_G = 0b10,  ///< +/- 8g
        MPU6050_RANGE_16_G = 0b11, ///< +/- 16g
    } mpu6050_accel_range_t;

    /**
     * @brief Gyroscope range options
     *
     * Allowed values for `setGyroRange`.
     */
    typedef enum
    {
        MPU6050_RANGE_250_DEG,  ///< +/- 250 deg/s (default value)
        MPU6050_RANGE_500_DEG,  ///< +/- 500 deg/s
        MPU6050_RANGE_1000_DEG, ///< +/- 1000 deg/s
        MPU6050_RANGE_2000_DEG, ///< +/- 2000 deg/s
    } mpu6050_gyro_range_t;

    typedef struct
    {
        gpio_num_t interrupt_pin;                     /*!< GPIO connected to mpu6050 INT pin       */
        mpu6050_int_pin_active_level_t active_level;  /*!< Active level of mpu6050 INT pin         */
        mpu6050_int_pin_mode_t pin_mode;              /*!< Push-pull or open drain mode for INT pin*/
        mpu6050_int_latch_t interrupt_latch;          /*!< The interrupt pulse behavior of INT pin */
        mpu6050_int_clear_t interrupt_clear_behavior; /*!< Interrupt status clear behavior         */
    } mpu6050_int_config_t;

    extern const uint8_t MPU6050_DATA_RDY_INT_BIT;      /*!< DATA READY interrupt bit               */
    extern const uint8_t MPU6050_I2C_MASTER_INT_BIT;    /*!< I2C MASTER interrupt bit               */
    extern const uint8_t MPU6050_FIFO_OVERFLOW_INT_BIT; /*!< FIFO Overflow interrupt bit            */
    extern const uint8_t MPU6050_MOT_DETECT_INT_BIT;    /*!< MOTION DETECTION interrupt bit         */
    extern const uint8_t MPU6050_ALL_INTERRUPTS;        /*!< All interrupts supported by mpu6050    */

    typedef struct
    {
        int16_t raw_acce_x;
        int16_t raw_acce_y;
        int16_t raw_acce_z;
    } mpu6050_raw_acce_value_t;

    typedef struct
    {
        int16_t raw_gyro_x;
        int16_t raw_gyro_y;
        int16_t raw_gyro_z;
    } mpu6050_raw_gyro_value_t;

    typedef struct
    {
        float acce_x;
        float acce_y;
        float acce_z;
    } mpu6050_acce_value_t;

    typedef struct
    {
        float gyro_x;
        float gyro_y;
        float gyro_z;
    } mpu6050_gyro_value_t;

    typedef struct
    {
        float temp;
    } mpu6050_temp_value_t;

    typedef struct
    {
        float roll;
        float pitch;
    } complimentary_angle_t;

    typedef void *mpu6050_handle_t;

    typedef gpio_isr_t mpu6050_isr_t;

    /**
     * @brief Create and init sensor object and return a sensor handle
     *
     * @param port I2C port number
     * @param dev_addr I2C device address of sensor
     *
     * @return
     *     - NULL Fail
     *     - Others Success
     */
    mpu6050_handle_t mpu6050_create(i2c_bus_handle_t bus, uint8_t dev_addr);

    /**
     * @brief Delete and release a sensor object
     *
     * @param sensor object handle of mpu6050
     */
    void mpu6050_delete(mpu6050_handle_t sensor);

    /**
     * @brief Get device identification of MPU6050
     *
     * @param sensor object handle of mpu6050
     * @param deviceid a pointer of device ID
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_deviceid(mpu6050_handle_t sensor, uint8_t *const deviceid);

    /**
     * @brief Wake up MPU6050
     *
     * @param sensor object handle of mpu6050
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_wake_up(mpu6050_handle_t sensor);

    /**
     * @brief Enter sleep mode
     *
     * @param sensor object handle of mpu6050
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_sleep(mpu6050_handle_t sensor);

    /**
     * @brief Set accelerometer and gyroscope full scale range
     *
     * @param sensor object handle of mpu6050
     * @param acce_fs accelerometer full scale range
     * @param gyro_fs gyroscope full scale range
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_config(mpu6050_handle_t sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs);

    /**
     * @brief Get accelerometer sensitivity
     *
     * @param sensor object handle of mpu6050
     * @param acce_sensitivity accelerometer sensitivity
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t sensor, float *const acce_sensitivity);

    /**
     * @brief Get gyroscope sensitivity
     *
     * @param sensor object handle of mpu6050
     * @param gyro_sensitivity gyroscope sensitivity
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t sensor, float *const gyro_sensitivity);

    /**
     * @brief Configure INT pin behavior and setup target GPIO.
     *
     * @param sensor object handle of mpu6050
     * @param interrupt_configuration mpu6050 INT pin configuration parameters
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or incorrect
     *      - ESP_FAIL Failed to configure INT pin on mpu6050
     */
    esp_err_t mpu6050_config_interrupts(mpu6050_handle_t sensor, const mpu6050_int_config_t *const interrupt_configuration);

    /**
     * @brief Register an Interrupt Service Routine to handle mpu6050 interrupts.
     *
     * @param sensor object handle of mpu6050
     * @param isr function to handle interrupts produced by mpu6050
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
     *      - ESP_FAIL Failed to register ISR
     */
    esp_err_t mpu6050_register_isr(mpu6050_handle_t sensor, const mpu6050_isr_t isr);

    /**
     * @brief Enable specific interrupts from mpu6050
     *
     * @param sensor object handle of mpu6050
     * @param interrupt_sources bit mask with interrupt sources to enable
     *
     * This function does not disable interrupts not set in interrupt_sources. To disable
     * specific mpu6050 interrupts, use mpu6050_disable_interrupts().
     *
     * To enable all mpu6050 interrupts, pass MPU6050_ALL_INTERRUPTS as the argument
     * for interrupt_sources.
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
     *      - ESP_FAIL Failed to enable interrupt sources on mpu6050
     */
    esp_err_t mpu6050_enable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources);

    /**
     * @brief Disable specific interrupts from mpu6050
     *
     * @param sensor object handle of mpu6050
     * @param interrupt_sources bit mask with interrupt sources to disable
     *
     * This function does not enable interrupts not set in interrupt_sources. To enable
     * specific mpu6050 interrupts, use mpu6050_enable_interrupts().
     *
     * To disable all mpu6050 interrupts, pass MPU6050_ALL_INTERRUPTS as the
     * argument for interrupt_sources.
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
     *      - ESP_FAIL Failed to enable interrupt sources on mpu6050
     */
    esp_err_t mpu6050_disable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources);

    /**
     * @brief Get the interrupt status of mpu6050
     *
     * @param sensor object handle of mpu6050
     * @param out_intr_status[out] bit mask that is assigned a value representing the interrupts triggered by the mpu6050
     *
     * This function can be used by the mpu6050 ISR to determine the source of
     * the mpu6050 interrupt that it is handling.
     *
     * After this function returns, the bits set in out_intr_status are
     * the sources of the latest interrupt triggered by the mpu6050. For example,
     * if MPU6050_DATA_RDY_INT_BIT is set in out_intr_status, the last interrupt
     * from the mpu6050 was a DATA READY interrupt.
     *
     * The behavior of the INT_STATUS register of the mpu6050 may change depending on
     * the value of mpu6050_int_clear_t used on interrupt configuration.
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
     *      - ESP_FAIL Failed to retrieve interrupt status from mpu6050
     */
    esp_err_t mpu6050_get_interrupt_status(mpu6050_handle_t sensor, uint8_t *const out_intr_status);

    /**
     * @brief Determine if the last mpu6050 interrupt was due to data ready.
     *
     * @param interrupt_status mpu6050 interrupt status, obtained by invoking mpu6050_get_interrupt_status()
     *
     * @return
     *      - 0: The interrupt was not produced due to data ready
     *      - Any other positive integer: Interrupt was a DATA_READY interrupt
     */
    extern uint8_t mpu6050_is_data_ready_interrupt(uint8_t interrupt_status);

    /**
     * @brief Determine if the last mpu6050 interrupt was an I2C master interrupt.
     *
     * @param interrupt_status mpu6050 interrupt status, obtained by invoking mpu6050_get_interrupt_status()
     *
     * @return
     *      - 0: The interrupt is not an I2C master interrupt
     *      - Any other positive integer: Interrupt was an I2C master interrupt
     */
    extern uint8_t mpu6050_is_i2c_master_interrupt(uint8_t interrupt_status);

    /**
     * @brief Determine if the last mpu6050 interrupt was triggered by a fifo overflow.
     *
     * @param interrupt_status mpu6050 interrupt status, obtained by invoking mpu6050_get_interrupt_status()
     *
     * @return
     *      - 0: The interrupt is not a fifo overflow interrupt
     *      - Any other positive integer: Interrupt was triggered by a fifo overflow
     */
    extern uint8_t mpu6050_is_fifo_overflow_interrupt(uint8_t interrupt_status);

    /**
     * @brief Read raw accelerometer measurements
     *
     * @param sensor object handle of mpu6050
     * @param raw_acce_value raw accelerometer measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_raw_acce(mpu6050_handle_t sensor, mpu6050_raw_acce_value_t *const raw_acce_value);

    /**
     * @brief Read raw gyroscope measurements
     *
     * @param sensor object handle of mpu6050
     * @param raw_gyro_value raw gyroscope measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_raw_gyro(mpu6050_handle_t sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value);

    /**
     * @brief Read accelerometer measurements
     *
     * @param sensor object handle of mpu6050
     * @param acce_value accelerometer measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_acce(mpu6050_handle_t sensor, mpu6050_acce_value_t *const acce_value);

    /**
     * @brief Read gyro values
     *
     * @param sensor object handle of mpu6050
     * @param gyro_value gyroscope measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_gyro(mpu6050_handle_t sensor, mpu6050_gyro_value_t *const gyro_value);

    /**
     * @brief Read temperature values
     *
     * @param sensor object handle of mpu6050
     * @param temp_value temperature measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_temp(mpu6050_handle_t sensor, mpu6050_temp_value_t *const temp_value);

    /**
     * @brief Use complimentory filter to calculate roll and pitch
     *
     * @param sensor object handle of mpu6050
     * @param acce_value accelerometer measurements
     * @param gyro_value gyroscope measurements
     * @param complimentary_angle complimentary angle
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_complimentory_filter(mpu6050_handle_t sensor, const mpu6050_acce_value_t *const acce_value,
                                           const mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle);

    /**
     * @brief Enable motion detection on MPU6050.
     *
     * Configures the MPU6050 to detect motion based on a specified threshold and duration.
     * When motion acceleration exceeds the threshold for the set duration, an interrupt is triggered.
     *
     * @param sensor Handle to the MPU6050 sensor instance.
     * @param threshold Motion detection sensitivity (0 - 255). Higher value means less sensitive.
     * @param duration Motion duration (0 - 255) required to trigger the interrupt.
     *
     * @return
     *     - ESP_OK: Success
     *     - ESP_FAIL: Failure
     */
    esp_err_t mpu6050_enable_motiondetection(mpu6050_handle_t sensor, uint8_t threshold, uint8_t duration);

    /**
     * @brief Initialize the MPU6050 sensor.
     *
     * Sets up the basic configuration for the MPU6050 sensor.
     *
     * @param sensor Handle to the MPU6050 sensor instance.
     * @return
     *     - ESP_OK: Success
     *     - ESP_FAIL: Failure
     */
    esp_err_t mpu6050_init(mpu6050_handle_t sensor);

    /**
     * @brief Reset the MPU6050 sensor.
     *
     * Resets the MPU6050 sensor to its default configuration.
     *
     * @param sensor Handle to the MPU6050 sensor instance.
     * @return
     *     - ESP_OK: Success
     *     - ESP_FAIL: Failure
     */
    esp_err_t mpu6050_reset(mpu6050_handle_t sensor);

    /**
     * @brief Set the sample rate divisor of the MPU6050 sensor.
     *
     * Configures the sample rate divisor to adjust the data output rate.
     *
     * @param sensor Handle to the MPU6050 sensor instance.
     * @param divisor Sample rate divisor value.
     * @return
     *     - ESP_OK: Success
     *     - ESP_FAIL: Failure
     */
    esp_err_t mpu6050_setSampleRateDivisor(mpu6050_handle_t sensor, uint8_t divisor);

    /**
     * @brief Set the filter bandwidth of the MPU6050 sensor.
     *
     * Configures the filter bandwidth to control the noise in the sensor data.
     *
     * @param sensor Handle to the MPU6050 sensor instance.
     * @param bandwidth Filter bandwidth setting.
     * @return
     *     - ESP_OK: Success
     *     - ESP_FAIL: Failure
     */
    esp_err_t mpu6050_setFilterBandwidth(mpu6050_handle_t sensor, mpu6050_bandwidth_t bandwidth);

    /**
     * @brief Set the gyroscope range of the MPU6050 sensor.
     *
     * Configures the full - scale range of the gyroscope.
     *
     * @param sensor Handle to the MPU6050 sensor instance.
     * @param new_range New gyroscope range setting.
     * @return
     *     - ESP_OK: Success
     *     - ESP_FAIL: Failure
     */
    esp_err_t mpu6050_setGyroRange(mpu6050_handle_t sensor, mpu6050_gyro_range_t new_range);

    /**
     * @brief Set the accelerometer range of the MPU6050 sensor.
     *
     * Configures the full - scale range of the accelerometer.
     *
     * @param sensor Handle to the MPU6050 sensor instance.
     * @param new_range New accelerometer range setting.
     * @return
     *     - ESP_OK: Success
     *     - ESP_FAIL: Failure
     */
    esp_err_t mpu6050_setAccelerometerRange(mpu6050_handle_t sensor, mpu6050_accel_range_t new_range);

    /**
     * @brief Set the high - pass filter of the MPU6050 sensor.
     *
     * Configures the high - pass filter bandwidth.
     *
     * @param sensor Handle to the MPU6050 sensor instance.
     * @param bandwidth High - pass filter bandwidth setting.
     * @return
     *     - ESP_OK: Success
     *     - ESP_FAIL: Failure
     */
    esp_err_t mpu6050_setHighPassFilter(mpu6050_handle_t sensor, mpu6050_highpass_t bandwidth);

    /**
     * @brief Set the motion detection threshold of the MPU6050 sensor.
     *
     * Configures the threshold for motion detection.
     *
     * @param sensor Handle to the MPU6050 sensor instance.
     * @param thr Motion detection threshold value.
     * @return
     *     - ESP_OK: Success
     *     - ESP_FAIL: Failure
     */
    esp_err_t mpu6050_setMotionDetectionThreshold(mpu6050_handle_t sensor, uint8_t thr);

    /**
     * @brief Set the motion detection duration of the MPU6050 sensor.
     *
     * Configures the duration required for motion to trigger an interrupt.
     *
     * @param sensor Handle to the MPU6050 sensor instance.
     * @param dur Motion detection duration value.
     * @return
     *     - ESP_OK: Success
     *     - ESP_FAIL: Failure
     */
    esp_err_t mpu6050_setMotionDetectionDuration(mpu6050_handle_t sensor, uint8_t dur);

    /**
     * @brief Set the interrupt pin latch of the MPU6050 sensor.
     *
     * Configures whether the interrupt pin is latched or not.
     *
     * @param sensor Handle to the MPU6050 sensor instance.
     * @param held True to enable latch, false to disable.
     * @return
     *     - ESP_OK: Success
     *     - ESP_FAIL: Failure
     */
    esp_err_t mpu6050_setInterruptPinLatch(mpu6050_handle_t sensor, bool held);

    /**
     * @brief Set the interrupt pin polarity of the MPU6050 sensor.
     *
     * Configures the polarity of the interrupt pin (active low or high).
     *
     * @param sensor Handle to the MPU6050 sensor instance.
     * @param active_low True for active low, false for active high.
     * @return
     *     - ESP_OK: Success
     *     - ESP_FAIL: Failure
     */
    esp_err_t mpu6050_setInterruptPinPolarity(mpu6050_handle_t sensor, bool active_low);

    /**
     * @brief Enable or disable the motion interrupt of the MPU6050 sensor.
     *
     * Turns on or off the motion - based interrupt.
     *
     * @param sensor Handle to the MPU6050 sensor instance.
     * @param active True to enable, false to disable.
     * @return
     *     - ESP_OK: Success
     *     - ESP_FAIL: Failure
     */
    esp_err_t mpu6050_setMotionInterrupt(mpu6050_handle_t sensor, bool active);

    /**
     * @brief Get the status of the motion interrupt of the MPU6050 sensor.
     *
     * Checks whether the motion interrupt is active or not.
     *
     * @param sensor Handle to the MPU6050 sensor instance.
     * @param active Pointer to a boolean variable to store the interrupt status.
     * @return
     *     - ESP_OK: Success
     *     - ESP_FAIL: Failure
     */
    esp_err_t mpu6050_getMotionInterruptStatus(mpu6050_handle_t sensor, bool *active);

#ifdef __cplusplus
}
#endif
