/**
 * @file kalman_10dof_imu.c
 * @author JanG175
 * @brief 10 DOF IMU sensor made from sensor fusion of MPU6050 accelerometer and gyroscope, HMC5883L magnetometer,
 * TF-LC02 TOF distance sensor and BMP280 pressure sensor
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "kalman_10dof_imu.h"

static const char* TAG = "kalman_10_dof_imu";

static mpu6050_handle_t mpu;
static hmc5883l_conf_t hmc;
static bmp280_conf_t bmp;
static tflc02_conf_t tflc;

static SemaphoreHandle_t mutex = NULL;
static kalman_data_t static_kalman_data;

i2c_master_bus_handle_t bus_handle; // I2C bus handle for all components


/**
 * @brief calculate Z acceleration from accelerometer data
 * 
 * @param acce_data accelerometer data
 * @param kalman_data kalman data
*/
static float calculate_z_accel(mpu6050_acce_value_t* acce_data, kalman_data_t* kalman_data)
{
    float roll = kalman_data->gyro_roll * M_PI / 180.0f;
    float pitch = -kalman_data->gyro_pitch * M_PI / 180.0f; // minus because of CCW angles

    float a_z = acce_data->acce_z * cosf(pitch) * cosf(roll) - acce_data->acce_x * sinf(pitch) + 
                    acce_data->acce_y * cosf(pitch) * sinf(roll);

    a_z = (a_z - 1.0f) * 9.81f;

    return a_z;
}


/**
 * @brief calculate euler angle from accelerometer data
 * 
 * @param acce_data accelerometer data
 * @param mag_data magnetometer data
 * @param euler_angle euler angle
*/
static void calculate_euler_angle_from_accel(mpu6050_acce_value_t* acce_data, magnetometer_raw_t* mag_data,
                                                euler_angle_t* euler_angle)
{
    euler_angle->roll = atan2f(acce_data->acce_y, acce_data->acce_z);
    euler_angle->pitch = atan2f(acce_data->acce_x, acce_data->acce_z);

    float mag_x = mag_data->x * cosf(euler_angle->pitch) +
                    mag_data->z * cosf(euler_angle->roll) * sinf(euler_angle->pitch) +
                    mag_data->y * sinf(euler_angle->roll) * sinf(euler_angle->pitch);

    float mag_y = mag_data->y * cosf(euler_angle->roll) - mag_data->z * sinf(euler_angle->roll);

    euler_angle->yaw = -atan2f(mag_y, mag_x); // minus because of CCW angles

    euler_angle->roll = euler_angle->roll * 180.0f / M_PI;
    euler_angle->pitch = euler_angle->pitch * 180.0f / M_PI;
    euler_angle->yaw = euler_angle->yaw * 180.0f / M_PI;
}


/**
 * @brief get data from sensors
 * 
 * @param acce acce data
 * @param gyro gyro data
 * @param mag magnetometer data
 * @param height height data
 * @param V_h process noise matrix
 * @param W_h sensor noise matrix
*/
static void imu_get_data(mpu6050_acce_value_t* acce, mpu6050_gyro_value_t* gyro, magnetometer_raw_t* mag, float* height,
                    dspm::Mat& V_h, dspm::Mat& W_h)
{
    ESP_ERROR_CHECK(mpu6050_get_acce(mpu, acce));
    ESP_ERROR_CHECK(mpu6050_get_gyro(mpu, gyro));

    // CCW angles
    acce->acce_x = -acce->acce_x;

    hmc5883l_read_magnetometer(hmc, &(mag->x), &(mag->y), &(mag->z));

    uint16_t tof_height = (uint16_t)(*height * 1000.0f);
    esp_err_t err = tflc02_measure_distance(tflc, &tof_height);

    if (err != ESP_OK) // if TOF sensor fails, use BMP280 sensor
    {
        bmp280_read_height(bmp, height);

        V_h(0, 0) = powf(STD_DEV_V_H_B, 2.0f);
        V_h(1 ,1) = powf(STD_DEV_V_H_B, 2.0f);
        W_h(0, 0) = powf(STD_DEV_W_H_B, 2.0f);
    }
    else // use TOF sensor
    {
        *height = (float)tof_height / 1000.0f; // convert to meters

        V_h(0, 0) = powf(STD_DEV_V_H_T, 2.0f);
        V_h(1, 1) = powf(STD_DEV_V_H_T, 2.0f);
        W_h(0, 0) = powf(STD_DEV_W_H_T, 2.0f);
    }
}


/**
 * @brief kalman filter task
 * 
 * @param pvParameters task parameters
*/
static IRAM_ATTR void kalman_data_read(void* pvParameters)
{
    float dt = (float)DT / 1000.0f; // convert to seconds

    // init measurement
    mpu6050_acce_value_t acce_data;
    mpu6050_gyro_value_t gyro_data;
    magnetometer_raw_t mag_data;
    float h_data;

    // height sensor noise
    dspm::Mat V_h(2, 2);
    V_h(0, 0) = powf(STD_DEV_V_H_T, 2.0f);
    V_h(0, 1) = 0.0f;
    V_h(1, 0) = 0.0f;
    V_h(1, 1) = powf(STD_DEV_V_H_T, 2.0f);

    dspm::Mat W_h(1, 1);
    W_h(0, 0) = powf(STD_DEV_W_H_T, 2.0f);

    imu_get_data(&acce_data, &gyro_data, &mag_data, &h_data, V_h, W_h);

    kalman_data_t task_kalman_data;
    euler_angle_t acce_euler_angle;
    calculate_euler_angle_from_accel(&acce_data, &mag_data, &acce_euler_angle);

    task_kalman_data.gyro_roll = acce_euler_angle.roll;
    task_kalman_data.gyro_pitch = acce_euler_angle.pitch;
    task_kalman_data.gyro_yaw = acce_euler_angle.yaw;

    // zero roll, pitch and yaw with accelerometer
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    {
        static_kalman_data.gyro_roll = acce_euler_angle.roll;
        static_kalman_data.gyro_pitch = acce_euler_angle.pitch;
        static_kalman_data.gyro_yaw = acce_euler_angle.yaw;

        xSemaphoreGive(mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take mutex to init task");
    }

    /*

    state space model:

    Xpri[k] = A * Xpri[k] + B * U[k]
       Y[k] = C * Xpri[k] + D * U[k]

    euler angles:

    --        --   --     --   --      --   --  --            
    | phi[k+1] |   | 1 -dt |   | phi[k] |   | dt |   --        --
    |          | = |       | * |        | + |    | * | Wgyro[k] |
    |  g[k+1]  |   | 0   1 |   |  g[k]  |   |  0 |   --        --
    --        --   --     --   --      --   --  --            

                     --      --
           --   --   | phi[k] |
    Y[k] = | 1 0 | * |        |
           --   --   |  g[k]  |
                     --      --

    height:

    --    --   --    --   --    --   --        --
    | h[k] |   | 1 dt |   | h[k] |   | 0.5*dt^2 |   --     --
    |      | = |      | * |      | + |          | * | Az[k] |
    | v[k] |   | 0  1 |   | v[k] |   |     0    |   --     --
    --    --   --    --   --    --   --        --

                     --    --
           --   --   | h[k] |
    Y[k] = | 1 0 | * |      |
           --   --   | v[k] |
                     --    --

    */

    // state space model for euler angles
    dspm::Mat A_e(2, 2);
    A_e(0, 0) = 1.0f;
    A_e(0, 1) = -dt;
    A_e(1, 0) = 0.0f;
    A_e(1, 1) = 1.0f;

    dspm::Mat B_e(2, 1);
    B_e(0, 0) = dt;
    B_e(1, 0) = 0.0f;

    dspm::Mat C_e(1, 2);
    C_e(0, 0) = 1.0f;
    C_e(0, 1) = 0.0f;

    // noise
    dspm::Mat V_e(2, 2);
    V_e(0, 0) = powf(STD_DEV_V_E, 2.0f);
    V_e(0, 1) = 0.0f;
    V_e(1, 0) = 0.0f;
    V_e(1, 1) = powf(STD_DEV_V_E, 2.0f);

    dspm::Mat W_e(1, 1);
    W_e(0, 0) = powf(STD_DEV_W_E, 2.0f);

    // initial states
    dspm::Mat Xpri_e(2, 3);
    Xpri_e(0, 0) = 0.0f;
    Xpri_e(0, 1) = 0.0f;
    Xpri_e(0, 2) = 0.0f;
    Xpri_e(1, 0) = 0.0f;
    Xpri_e(1, 1) = 0.0f;
    Xpri_e(1, 2) = 0.0f;

    dspm::Mat Ppri_e(2, 2);
    Ppri_e(0, 0) = 1.0f;
    Ppri_e(0, 1) = 0.0f;
    Ppri_e(1, 0) = 0.0f;
    Ppri_e(1, 1) = 1.0f;

    dspm::Mat Xpost_e(2, 3);
    Xpost_e(0, 0) = acce_euler_angle.roll;
    Xpost_e(0, 1) = acce_euler_angle.pitch;
    Xpost_e(0, 2) = acce_euler_angle.yaw;
    Xpost_e(1, 0) = 0.0f;
    Xpost_e(1, 1) = 0.0f;
    Xpost_e(1, 2) = 0.0f;

    dspm::Mat Ppost_e(2, 2);
    Ppost_e(0, 0) = 1.0f;
    Ppost_e(0, 1) = 0.0f;
    Ppost_e(1, 0) = 0.0f;
    Ppost_e(1, 1) = 1.0f;

    dspm::Mat U_e(1, 3);
    U_e(0, 0) = 0.0f;
    U_e(0, 1) = 0.0f;
    U_e(0, 2) = 0.0f;

    dspm::Mat Y_e(1, 3);
    Y_e(0, 0) = 0.0f;
    Y_e(0, 1) = 0.0f;
    Y_e(0, 2) = 0.0f;

    dspm::Mat E_e(1, 3);
    dspm::Mat S_e(1, 1);
    dspm::Mat K_e(2, 1);

    // state space model for height
    dspm::Mat A_h(2, 2);
    A_h(0, 0) = 1.0f;
    A_h(0, 1) = dt;
    A_h(1, 0) = 0.0f;
    A_h(1, 1) = 1.0f;

    dspm::Mat B_h(2, 1);
    B_h(0, 0) = 0.5f * powf(dt, 2.0f);
    B_h(1, 0) = dt;

    dspm::Mat C_h(1, 2);
    C_h(0, 0) = 1.0f;
    C_h(0, 1) = 0.0f;

    // initial states
    dspm::Mat Xpri_h(2, 1);
    Xpri_h(0, 0) = 0.0f;
    Xpri_h(1, 0) = 0.0f;

    dspm::Mat Ppri_h(2, 2);
    Ppri_h(0, 0) = 1.0f;
    Ppri_h(0, 1) = 0.0f;
    Ppri_h(1, 0) = 0.0f;
    Ppri_h(1, 1) = 1.0f;

    dspm::Mat Xpost_h(2, 1);
    Xpost_h(0, 0) = h_data;
    Xpost_h(1, 0) = 0.0f;

    dspm::Mat Ppost_h(2, 2);
    Ppost_h(0, 0) = 1.0f;
    Ppost_h(0, 1) = 0.0f;
    Ppost_h(1, 0) = 0.0f;
    Ppost_h(1, 1) = 1.0f;

    dspm::Mat U_h(1, 1);
    U_h(0, 0) = 0.0f;

    dspm::Mat Y_h(1, 1);
    Y_h(0, 0) = 0.0f;

    dspm::Mat E_h(1, 1);
    dspm::Mat S_h(1, 1);
    dspm::Mat K_h(2, 1);

    // time variables
    TickType_t time = 0;
    TickType_t last_time = 0;
    TickType_t mutex_wait = 0;

    // Kalman filter
    while (1)
    {
        last_time = xTaskGetTickCount();

        // new measurement
        imu_get_data(&acce_data, &gyro_data, &mag_data, &h_data, V_h, W_h);

        calculate_euler_angle_from_accel(&acce_data, &mag_data, &acce_euler_angle);

        U_e(0, 0) = gyro_data.gyro_x;
        U_e(0, 1) = gyro_data.gyro_y;
        U_e(0, 2) = gyro_data.gyro_z;

        Y_e(0, 0) = acce_euler_angle.roll;
        Y_e(0, 1) = acce_euler_angle.pitch;
        Y_e(0, 2) = acce_euler_angle.yaw;

        // euler angles kalman

        // Xpri_e
        Xpri_e = (A_e * Xpost_e) + (B_e * U_e);

        // Ppri_e
        Ppri_e = (A_e * Ppost_e * A_e.t()) + V_e;

        // E_e
        E_e = Y_e - (C_e * Xpri_e);

        // S_e
        S_e = (C_e * Ppri_e * C_e.t()) + W_e;

        // K_e
        K_e = (Ppri_e * C_e.t()) * S_e.inverse();

        // Xpost_e
        Xpost_e = Xpri_e + (K_e * E_e);

        // Ppost_e
        Ppost_e = Ppri_e - (K_e * S_e * K_e.t());

        // calculate euler angle from gyro
        task_kalman_data.acce_roll = Xpost_e(0, 0);
        task_kalman_data.acce_pitch = Xpost_e(0, 1);
        task_kalman_data.mag_yaw = Xpost_e(0, 2);

        task_kalman_data.gyro_roll += (gyro_data.gyro_x - Xpost_e(1, 0)) * dt;
        task_kalman_data.gyro_pitch += (gyro_data.gyro_y - Xpost_e(1, 1)) * dt;
        task_kalman_data.gyro_yaw += (gyro_data.gyro_z - Xpost_e(1, 2)) * dt;

        // height kalman

        U_h(0, 0) = calculate_z_accel(&acce_data, &task_kalman_data);
        Y_h(0, 0) = h_data;

        // Xpri_h
        Xpri_h = (A_h * Xpost_h) + (B_h * U_h);

        // Ppri_h
        Ppri_h = (A_h * Ppost_h * A_h.t()) + V_h;

        // E_h
        E_h = Y_h - (C_h * Xpri_h);

        // S_h
        S_h = (C_h * Ppri_h * C_h.t()) + W_h;

        // K_h
        K_h = (Ppri_h * C_h.t()) * S_h.inverse();

        // Xpost_h
        Xpost_h = Xpri_h + (K_h * E_h);

        // Ppost_h
        Ppost_h = Ppri_h - (K_h * S_h * K_h.t());

        // update height
        task_kalman_data.height = Xpost_h(0, 0);

        // calculate time left to wait
        time = xTaskGetTickCount();
        mutex_wait = DT - (time - last_time) * portTICK_PERIOD_MS;

        if (xSemaphoreTake(mutex, mutex_wait / portTICK_PERIOD_MS) == pdTRUE)
        {
            static_kalman_data.acce_roll = task_kalman_data.acce_roll;
            static_kalman_data.acce_pitch = task_kalman_data.acce_pitch;
            static_kalman_data.mag_yaw = task_kalman_data.mag_yaw;

            static_kalman_data.gyro_roll = task_kalman_data.gyro_roll;
            static_kalman_data.gyro_pitch = task_kalman_data.gyro_pitch;
            static_kalman_data.gyro_yaw = task_kalman_data.gyro_yaw;

            static_kalman_data.raw_height = h_data;
            static_kalman_data.height = task_kalman_data.height;

            xSemaphoreGive(mutex);

            xTaskDelayUntil(&last_time, DT / portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to take mutex to update");
        }
    }

    vTaskDelete(NULL);
}


/**
 * @brief initialize imu sensor
 * 
 * @param imu_conf imu I2C configuration
*/
extern "C" void imu_init(imu_conf_t imu_conf)
{
    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // initialize I2C
    i2c_master_bus_config_t i2c_mst_config;
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = imu_conf.i2c_port;
    i2c_mst_config.scl_io_num = imu_conf.scl_pin;
    i2c_mst_config.sda_io_num = imu_conf.sda_pin;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;
    i2c_mst_config.intr_priority = 0;
    i2c_mst_config.trans_queue_depth = 0;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // initialize MPU6050 sensor
    mpu = mpu6050_create(imu_conf.i2c_port, MPU6050_I2C_ADDRESS, imu_conf.i2c_freq);
    if (mpu == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mpu6050");
        return;
    }
    ESP_ERROR_CHECK(mpu6050_wake_up(mpu));
    ESP_ERROR_CHECK(mpu6050_i2c_passthrough(mpu));

    // initialize HMC5883L sensor
    hmc.i2c_port = imu_conf.i2c_port;
    hmc.sda_pin = imu_conf.sda_pin;
    hmc.scl_pin = imu_conf.scl_pin;
    hmc.i2c_freq = imu_conf.i2c_freq;
    hmc.drdy_pin = GPIO_NUM_NC;

    hmc5883l_init(hmc);
    hmc5883l_write_config(hmc, HMC5883L_OVER_SAMPLE_8, HMC5883L_DATA_OUTPUT_RATE_75_HZ, HMC5883L_MODE_NORMAL, HMC5883L_GAIN_1090);
    hmc5883l_write_mode(hmc, HMC5883L_CONTINUOUS_MODE);

    // initialize BMP280 sensor
    bmp.i2c_port = imu_conf.i2c_port;
    bmp.i2c_addr = BMP280_I2C_ADDRESS_1;
    bmp.sda_pin = imu_conf.sda_pin;
    bmp.scl_pin = imu_conf.scl_pin;
    bmp.i2c_freq = imu_conf.i2c_freq;
    bmp280_init(bmp, BMP280_ULTRA_HIGH_RES);

    // initialize TF-L02 sensor
    tflc.uart_port = imu_conf.uart_port;
    tflc.tx_pin = imu_conf.uart_tx_pin;
    tflc.rx_pin = imu_conf.uart_rx_pin;
    tflc02_init(tflc);

    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    {
        static_kalman_data.acce_roll = 0.0f;
        static_kalman_data.acce_pitch = 0.0f;
        static_kalman_data.mag_yaw = 0.0f;
        static_kalman_data.gyro_roll = 0.0f;
        static_kalman_data.gyro_pitch = 0.0f;
        static_kalman_data.gyro_yaw = 0.0f;

        static_kalman_data.raw_height = 0.0f;
        static_kalman_data.height = 0.0f;

        xSemaphoreGive(mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take mutex to init");
    }

    xTaskCreate(kalman_data_read, "kalman filter", 8192, NULL, 10, NULL);
}


/**
 * @brief get data from kalman filter
 * 
 * @param kalman_data kalman data
*/
extern "C" void imu_get_kalman_data(kalman_data_t* kalman_data)
{
    if (xSemaphoreTake(mutex, 0) == pdTRUE)
    {
        kalman_data->acce_roll = static_kalman_data.acce_roll;
        kalman_data->acce_pitch = static_kalman_data.acce_pitch;
        kalman_data->mag_yaw = static_kalman_data.mag_yaw;

        kalman_data->gyro_roll = static_kalman_data.gyro_roll;
        kalman_data->gyro_pitch = static_kalman_data.gyro_pitch;
        kalman_data->gyro_yaw = static_kalman_data.gyro_yaw;

        kalman_data->raw_height = static_kalman_data.raw_height;
        kalman_data->height = static_kalman_data.height;

        xSemaphoreGive(mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take mutex to read");
    }
}
