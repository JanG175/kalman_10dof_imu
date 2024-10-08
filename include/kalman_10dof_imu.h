/**
 * @file kalman_10dof_imu.h
 * @author JanG175
 * @brief 10 DOF IMU sensor made from sensor fusion of MPU6050 accelerometer and gyroscope, HMC5883L magnetometer,
 * TF-LC02 TOF distance sensor and BMP280 pressure sensor
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include <math.h>
#include "dsp_platform.h"
#include "esp_dsp.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mpu6050.h"
#include "esp_hmc5883l.h"
#include "esp_bmp280.h"
#include "esp_tf-lc02.h"

// #define TEST_PERFORMANCE 1 // uncomment to test loop performance
// #define COMPLEMENTARY_FILTER 1 // uncomment to use complementary filter instead of kalman filter

#define DT               4 // integration step in ms

// euler angles kalman filter
#define STD_DEV_V_E      0.0001f // process noise
#define STD_DEV_W_E      0.005f // sensor noise

// height kalman filter
#define STD_DEV_V_H      0.001f // process noise
#define STD_DEV_W_H      0.01f // sensor noise

// complementary filter
#define ALPHA            0.98f // complementary filter coefficient

typedef struct
{
    i2c_port_num_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t i2c_freq;

    uart_port_t uart_port;
    gpio_num_t uart_tx_pin;
    gpio_num_t uart_rx_pin;
} imu_conf_t;

typedef struct
{
    float x;
    float y;
    float z;
} acce_raw_t;

typedef struct
{
    float x;
    float y;
    float z;
} gyro_raw_t;

typedef struct
{
    float x;
    float y;
    float z;
} mag_raw_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} euler_angle_t;

typedef struct
{
    float acce_roll;
    float acce_pitch;
    float mag_yaw;

    float gyro_roll;
    float gyro_pitch;
    float gyro_yaw;

    float gyro_raw_z;

    float raw_height;
    float height;
} kalman_data_t;


void imu_init(imu_conf_t imu_conf);

void imu_get_kalman_data(kalman_data_t* kalman_data);

#ifdef __cplusplus
}
#endif