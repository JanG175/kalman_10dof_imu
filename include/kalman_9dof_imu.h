#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_qmc5883l.h"
#include "esp_matrix.h"
#include "esp_log.h"
#include "esp_timer.h"

#define DT             2   // integration step in ms

#define STD_DEV_V      0.8 // process noise
#define STD_DEV_W      0.9 // sensor noise


typedef struct
{
    i2c_port_t i2c_num;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t i2c_freq;
} imu_i2c_conf_t;

typedef struct
{
    float x;
    float y;
    float z;
} magnetometer_raw_t;

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
} kalman_euler_angle_t;


void imu_init(imu_i2c_conf_t mpu_conf);

void imu_get_data(mpu6050_acce_value_t* acce, mpu6050_gyro_value_t* gyro, magnetometer_raw_t* mag);

void imu_get_euler_angle(kalman_euler_angle_t* euler_angle);