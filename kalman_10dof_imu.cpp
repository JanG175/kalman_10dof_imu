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

// I2C handles
static mpu6050_conf_t mpu;
static hmc5883l_conf_t hmc;
static bmp280_conf_t bmp;
static tflc02_conf_t tflc;

// I2C bus handle for all components
i2c_master_bus_handle_t bus_handle;

// kalman data protected by mutex
static SemaphoreHandle_t mutex = NULL;
static kalman_data_t static_kalman_data;


/**
 * @brief calculate Z acceleration from accelerometer data
 * 
 * @param acce_data accelerometer data
 * @param kalman_data kalman data
*/
static float calculate_z_accel(acce_raw_t* acce_data, kalman_data_t* kalman_data)
{
    float roll = kalman_data->gyro_roll * M_PI / 180.0f;
    float pitch = -kalman_data->gyro_pitch * M_PI / 180.0f;

    float a_z = acce_data->z * cosf(pitch) * cosf(roll) - acce_data->x * sinf(pitch) + 
                acce_data->y * cosf(pitch) * sinf(roll);

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
static void calculate_euler_angle_from_accel(acce_raw_t* acce_data, mag_raw_t* mag_data, euler_angle_t* euler_angle)
{
    dspm::Mat q(1, 4);

    // normalize accelerometer data
    float norm_accel = sqrtf(powf(acce_data->x, 2.0f) + powf(acce_data->y, 2.0f) + powf(acce_data->z, 2.0f));

    // calculate accelerometer quaternion
    if (acce_data->z >= 0)
    {
        q(0, 0) = sqrtf((acce_data->z/norm_accel + 1.0f) / 2.0f);
        q(0, 1) = -acce_data->y/norm_accel / sqrtf(2.0f * (acce_data->z/norm_accel + 1.0f));
        q(0, 2) = acce_data->x/norm_accel / sqrtf(2.0f * (acce_data->z/norm_accel + 1.0f));
        q(0, 3) = 0.0f;
    }
    else
    {
        q(0, 0) = -acce_data->y/norm_accel / sqrtf(2.0f * (1.0f - acce_data->z/norm_accel));
        q(0, 1) = sqrtf((1.0f - acce_data->z/norm_accel) / 2.0f);
        q(0, 2) = 0.0f;
        q(0, 3) = acce_data->x/norm_accel / sqrtf(2.0f * (1.0f - acce_data->z/norm_accel));
    }

    // calculate euler angles from quaternion
    euler_angle->roll = atan2f(q(0, 2) * q(0, 3) + q(0, 0) * q(0, 1),
                               0.5f - (powf(q(0, 1), 2.0f) + powf(q(0, 2), 2.0f)));
    euler_angle->pitch = asinf(2.0f * (q(0, 0) * q(0, 2) - q(0, 1) * q(0, 3)));

    // calculate yaw angle from magnetometer data

    float mag_x = mag_data->x * cosf(euler_angle->pitch) +
                  mag_data->z * cosf(euler_angle->roll) * sinf(euler_angle->pitch) +
                  mag_data->y * sinf(euler_angle->roll) * sinf(euler_angle->pitch);
    float mag_y = mag_data->y * cosf(euler_angle->roll) - mag_data->z * sinf(euler_angle->roll);

    euler_angle->yaw = -atan2f(mag_y, mag_x);

    euler_angle->roll = euler_angle->roll * 180.0f / M_PI;
    euler_angle->pitch = euler_angle->pitch * 180.0f / M_PI;
    euler_angle->yaw = euler_angle->yaw * 180.0f / M_PI;

    // prevent NaN accumulation
    if (isnanf(euler_angle->roll))
        euler_angle->roll = 0.0f;
    if (isnanf(euler_angle->pitch))
        euler_angle->pitch = 0.0f;
    if (isnanf(euler_angle->yaw))
        euler_angle->yaw = 0.0f;
}


/**
 * @brief get data from sensors
 * 
 * @param acce acce data
 * @param gyro gyro data
 * @param mag magnetometer data
 * @param height height data
 * @param height_offset height offset
 * @param new_offset_flag new height offset flag
*/
static void imu_get_data(acce_raw_t* acce, gyro_raw_t* gyro, mag_raw_t* mag, float* height,
                         float* height_offset, bool* new_offset_flag)
{
    mpu6050_read_accelerometer(mpu, &(acce->x), &(acce->y), &(acce->z));
    mpu6050_read_gyroscope(mpu, &(gyro->x), &(gyro->y), &(gyro->z));

    // correct accelerometer data (for positive roll and pitch angles)
    acce->x = -acce->x;
    acce->y = -acce->y;

    hmc5883l_read_magnetometer(hmc, &(mag->x), &(mag->y), &(mag->z));

    uint16_t tof_height = (uint16_t)(*height * 1000.0f);
    esp_err_t err = tflc02_measure_distance(tflc, &tof_height);

    if (err == ESP_OK) // use TOF sensor
    {
        *height = (float)tof_height / 1000.0f; // convert to meters

        *new_offset_flag = true;
    }
    else if (*height > 2.0f) // if TOF sensor fails and the drone is high enough, use BMP280 sensor
    {
        // if previous measurement was from TOF sensor, set new height offset for linear transition
        if (*new_offset_flag == true)
        {
            *height_offset = *height;
            bmp280_set_sea_level_pressure(bmp);
        }
        else
        {
            bmp280_read_height(bmp, height);
            *height += *height_offset;
        }

        *new_offset_flag = false;
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

    float height_offset = 0.0f;
    bool new_offset_flag = false;

    // init measurement
    acce_raw_t acce_data;
    gyro_raw_t gyro_data;
    mag_raw_t mag_data;
    float h_data;

    // height sensor noise
    dspm::Mat V_h(2, 2);
    V_h(0, 0) = powf(STD_DEV_V_H, 2.0f);
    V_h(0, 1) = 0.0f;
    V_h(1, 0) = 0.0f;
    V_h(1, 1) = powf(STD_DEV_V_H, 2.0f);

    dspm::Mat W_h(1, 1);
    W_h(0, 0) = powf(STD_DEV_W_H, 2.0f);

    imu_get_data(&acce_data, &gyro_data, &mag_data, &h_data, &height_offset, &new_offset_flag);

    kalman_data_t task_kalman_data;
    euler_angle_t euler_angle;
    calculate_euler_angle_from_accel(&acce_data, &mag_data, &euler_angle);

    task_kalman_data.acce_roll = euler_angle.roll;
    task_kalman_data.acce_pitch = euler_angle.pitch;
    task_kalman_data.mag_yaw = euler_angle.yaw;
    task_kalman_data.gyro_roll = euler_angle.roll;
    task_kalman_data.gyro_pitch = euler_angle.pitch;
    task_kalman_data.gyro_yaw = euler_angle.yaw;

    // zero roll, pitch and yaw with accelerometer
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    {
        static_kalman_data.acce_roll = euler_angle.roll;
        task_kalman_data.acce_pitch = euler_angle.pitch;
        task_kalman_data.mag_yaw = euler_angle.yaw;
        static_kalman_data.gyro_roll = euler_angle.roll;
        static_kalman_data.gyro_pitch = euler_angle.pitch;
        static_kalman_data.gyro_yaw = euler_angle.yaw;

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

    --      --   --    --   --    --   --        --
    | h[k+1] |   | 1 dt |   | h[k] |   | 0.5*dt^2 |   --     --
    |        | = |      | * |      | + |          | * | Az[k] |
    | v[k+1] |   | 0  1 |   | v[k] |   |     0    |   --     --
    --      --   --    --   --    --   --        --

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
    Xpost_e(0, 0) = euler_angle.roll;
    Xpost_e(0, 1) = euler_angle.pitch;
    Xpost_e(0, 2) = euler_angle.yaw;
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

    bool init = true;

    // yaw wrapping variables
    const uint8_t N = 5;
    int8_t is_wrap[N];
    for (uint8_t i = 0; i < N; i++)
        is_wrap[i] = 0;
    uint8_t wrap_index = 0;
    int8_t wrap_allowed = 0;
    uint8_t wrap_count = 0;

    // Kalman filter
    while (1)
    {
#ifdef TEST_PERFORMANCE
        int64_t loop_start = esp_timer_get_time();
#endif

        last_time = xTaskGetTickCount();

        // new measurement
        imu_get_data(&acce_data, &gyro_data, &mag_data, &h_data, &height_offset, &new_offset_flag);

        calculate_euler_angle_from_accel(&acce_data, &mag_data, &euler_angle);

        // prevent yaw wrapping
        if (init == false)
        {
            // check if yaw sign is stable on wrap threshold
            if (task_kalman_data.mag_yaw > YAW_WRAP_TRESH)
                is_wrap[wrap_index] = 1;
            else if (task_kalman_data.mag_yaw < -YAW_WRAP_TRESH)
                is_wrap[wrap_index] = -1;
            else
            {
                is_wrap[wrap_index] = 0;
                wrap_count++;
            }

            int8_t sum_is_wrap = 0;
            if (is_wrap[wrap_index] != 0)
                for (uint8_t i = 0; i < N; i++)
                    sum_is_wrap += is_wrap[i];
            wrap_index = (wrap_index + 1) % N;

            if (wrap_allowed == 0)
            {
                // check if if yaw wrap can occur (wrap_allowed = 1 or -1)
                if (abs(sum_is_wrap) == N)
                {
                    wrap_allowed = sum_is_wrap / N;

                    for (uint8_t i = 0; i < N; i++)
                        is_wrap[i] = 0;
                }
            }
            else if (abs(wrap_allowed) == 1)
            {
                // check if yaw wrap has occurred (wrap_allowed = 2 or -2)
                if (sum_is_wrap == -wrap_allowed * N)
                    wrap_allowed = wrap_allowed * 2;
            }

            // prevent waiting for wrap if it does not occur
            if (wrap_count == 50)
            {
                wrap_count = 0;
                wrap_allowed = 0;
            }
        }

        // update input and output matrices

        U_e(0, 0) = gyro_data.x;
        U_e(0, 1) = gyro_data.y;
        U_e(0, 2) = gyro_data.z;

        Y_e(0, 0) = euler_angle.roll;
        Y_e(0, 1) = euler_angle.pitch;
        Y_e(0, 2) = euler_angle.yaw;

        // euler angles kalman

        // Xpri_e
        Xpri_e = A_e * Xpost_e + B_e * U_e;

        // Ppri_e
        Ppri_e = A_e * Ppost_e * A_e.t() + V_e;

        // E_e
        E_e = Y_e - C_e * Xpri_e;

        // S_e
        S_e = C_e * Ppri_e * C_e.t() + W_e;

        // K_e
        K_e = Ppri_e * C_e.t() * S_e.inverse();

        // Xpost_e
        Xpost_e = Xpri_e + K_e * E_e;

        // Ppost_e
        Ppost_e = Ppri_e - K_e * S_e * K_e.t();

        // calculate euler angle from gyro
        task_kalman_data.acce_roll = Xpost_e(0, 0);
        task_kalman_data.acce_pitch = Xpost_e(0, 1);
        task_kalman_data.mag_yaw = Xpost_e(0, 2);

        task_kalman_data.gyro_roll += (gyro_data.x - Xpost_e(1, 0)) * dt;
        task_kalman_data.gyro_pitch += (gyro_data.y - Xpost_e(1, 1)) * dt;

        // yaw wrapping
        float wrap = 0.0f;
        if (wrap_allowed == 2 || wrap_allowed == -2)
            wrap = task_kalman_data.mag_yaw - task_kalman_data.gyro_yaw;

        task_kalman_data.gyro_yaw += (gyro_data.z - Xpost_e(1, 2)) * dt + wrap;

        if (wrap_allowed == 2 || wrap_allowed == -2)
        {
            Xpost_e(1, 2) = 0.0f; // reset gyro drift

            // reset wrap check
            wrap_allowed = 0;
            wrap_count = 0;
            for (uint8_t i = 0; i < N; i++)
                is_wrap[i] = 0;
        }

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

#ifdef TEST_PERFORMANCE
            ESP_LOGI(TAG, "%lld us", esp_timer_get_time() - loop_start);
#endif

            xTaskDelayUntil(&last_time, DT / portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to take mutex to update");
        }

        init = false;
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
    i2c_mst_config.intr_priority = 3;
    i2c_mst_config.trans_queue_depth = 0;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // initialize MPU6050 sensor
    mpu.i2c_port = imu_conf.i2c_port;
    mpu.sda_pin = imu_conf.sda_pin;
    mpu.scl_pin = imu_conf.scl_pin;
    mpu.i2c_freq = imu_conf.i2c_freq;
    mpu.i2c_addr = MPU6050_ADDR_0;
    mpu6050_init(mpu);
    mpu6050_i2c_passthrough(mpu);

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
    bmp.i2c_addr = BMP280_I2C_ADDRESS_0;
    bmp.sda_pin = imu_conf.sda_pin;
    bmp.scl_pin = imu_conf.scl_pin;
    bmp.i2c_freq = imu_conf.i2c_freq;
    bmp280_init(bmp, BMP280_ULTRA_HIGH_RES);

    // initialize TF-L02 sensor
    tflc.uart_port = imu_conf.uart_port;
    tflc.tx_pin = imu_conf.uart_tx_pin;
    tflc.rx_pin = imu_conf.uart_rx_pin;
    tflc02_init(tflc);

    // probe sensors
    ESP_ERROR_CHECK(i2c_master_probe(bus_handle, mpu.i2c_addr, 0));
    ESP_ERROR_CHECK(i2c_master_probe(bus_handle, HMC5883L_ADDR, 0));
    ESP_ERROR_CHECK(i2c_master_probe(bus_handle, bmp.i2c_addr, 0));

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

    xTaskCreate(kalman_data_read, "kalman filter", 8192, NULL, 17, NULL);
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
