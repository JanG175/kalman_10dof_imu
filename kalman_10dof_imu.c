/**
 * @file kalman_10dof_imu.c
 * @author JanG175
 * @brief 10 DOF IMU sensor made from sensor fusion of MPU6050 accelerometer and gyroscope, QMC5883L magnetometer
 * and BMP280 pressure sensor
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "kalman_10dof_imu.h"

static const char* TAG = "kalman_10_dof_imu";
static mpu6050_handle_t mpu;
static qmc5883l_conf_t qmc;
static bmp280_conf_t bmp;

static SemaphoreHandle_t mutex = NULL;
static kalman_data_t static_kalman_data;


/**
 * @brief calculate Z acceleration from accelerometer data
 * 
 * @param acce_data accelerometer data
 * @param kalman_data kalman data
*/
static float calculate_z_accel(mpu6050_acce_value_t* acce_data, kalman_data_t* kalman_data)
{
    float roll = kalman_data->gyro_roll * M_PI / 180.0f;
    float pitch = -kalman_data->gyro_pitch * M_PI / 180.0f; // minus caused by MPU6050 mounted backwards

    float a_z = acce_data->acce_z*cosf(pitch)*cosf(roll) - acce_data->acce_x*sinf(pitch) + 
                    acce_data->acce_y*cosf(pitch)*sinf(roll);

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

    euler_angle->yaw = atan2f(mag_y, mag_x);

    euler_angle->roll = euler_angle->roll * 180.0f / M_PI;
    euler_angle->pitch = euler_angle->pitch * 180.0f / M_PI;
    euler_angle->yaw = euler_angle->yaw * 180.0f / M_PI;
}


/**
 * @brief kalman filter task
 * 
 * @param pvParameters task parameters
*/
static IRAM_ATTR void kalman_data_read(void* pvParameters)
{
    double dt = (double)DT / 1000.0;

    // init measurement
    mpu6050_acce_value_t acce_data;
    mpu6050_gyro_value_t gyro_data;
    magnetometer_raw_t mag_data;
    float h_data;

    imu_get_data(&acce_data, &gyro_data, &mag_data, &h_data);

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
    matrix_t A_e;
    matrix_alloc(&A_e, 2, 2);
    A_e.array[0][0] = 1.0;
    A_e.array[0][1] = -dt;
    A_e.array[1][0] = 0.0;
    A_e.array[1][1] = 1.0;

    matrix_t B_e;
    matrix_alloc(&B_e, 2, 1);
    B_e.array[0][0] = dt;
    B_e.array[1][0] = 0.0;

    matrix_t C_e;
    matrix_alloc(&C_e, 1, 2);
    C_e.array[0][0] = 1.0;
    C_e.array[0][1] = 0.0;

    // noise
    matrix_t V_e;
    matrix_alloc(&V_e, 2, 2);
    V_e.array[0][0] = pow(STD_DEV_V_E, 2.0);
    V_e.array[0][1] = 0.0;
    V_e.array[1][0] = 0.0;
    V_e.array[1][1] = pow(STD_DEV_V_E, 2.0);

    matrix_t W_e;
    matrix_alloc(&W_e, 1, 1);
    W_e.array[0][0] = pow(STD_DEV_W_E, 2.0);

    // initial states
    matrix_t Xpri_e;
    matrix_alloc(&Xpri_e, 2, 3);
    Xpri_e.array[0][0] = 0.0;
    Xpri_e.array[0][1] = 0.0;
    Xpri_e.array[0][2] = 0.0;
    Xpri_e.array[1][0] = 0.0;
    Xpri_e.array[1][1] = 0.0;
    Xpri_e.array[1][2] = 0.0;

    matrix_t Ppri_e;
    matrix_alloc(&Ppri_e, 2, 2);
    Ppri_e.array[0][0] = 1.0;
    Ppri_e.array[0][1] = 0.0;
    Ppri_e.array[1][0] = 0.0;
    Ppri_e.array[1][1] = 1.0;

    matrix_t Xpost_e;
    matrix_alloc(&Xpost_e, 2, 3);
    Xpost_e.array[0][0] = acce_euler_angle.roll;
    Xpost_e.array[0][1] = acce_euler_angle.pitch;
    Xpost_e.array[0][2] = acce_euler_angle.yaw;
    Xpost_e.array[1][0] = 0.0;
    Xpost_e.array[1][1] = 0.0;
    Xpost_e.array[1][2] = 0.0;

    matrix_t Ppost_e;
    matrix_alloc(&Ppost_e, 2, 2);
    Ppost_e.array[0][0] = 1.0;
    Ppost_e.array[0][1] = 0.0;
    Ppost_e.array[1][0] = 0.0;
    Ppost_e.array[1][1] = 1.0;

    matrix_t U_e;
    matrix_alloc(&U_e, 1, 3);
    U_e.array[0][0] = 0.0;
    U_e.array[0][1] = 0.0;
    U_e.array[0][2] = 0.0;

    matrix_t Y_e;
    matrix_alloc(&Y_e, 1, 3);
    Y_e.array[0][0] = 0.0;
    Y_e.array[0][1] = 0.0;
    Y_e.array[0][2] = 0.0;

    matrix_t E_e;
    matrix_alloc(&E_e, 1, 3);

    matrix_t S_e;
    matrix_alloc(&S_e, 1, 1);

    matrix_t K_e;
    matrix_alloc(&K_e, 2, 1);

    // auxilary matrices
    matrix_t AXpost_e;
    matrix_alloc(&AXpost_e, A_e.rows, Xpost_e.cols);

    matrix_t BU_e;
    matrix_alloc(&BU_e, B_e.rows, U_e.cols);

    matrix_t APpost_e;
    matrix_alloc(&APpost_e, A_e.rows, Ppost_e.cols);

    matrix_t At_e;
    matrix_alloc(&At_e, A_e.cols, A_e.rows);
    matrix_trans(&A_e, &At_e);

    matrix_t APpostAt_e;
    matrix_alloc(&APpostAt_e, APpost_e.rows, At_e.cols);

    matrix_t CXpri_e;
    matrix_alloc(&CXpri_e, C_e.rows, Xpri_e.cols);

    matrix_t CPpri_e;
    matrix_alloc(&CPpri_e, C_e.rows, Ppri_e.cols);

    matrix_t Ct_e;
    matrix_alloc(&Ct_e, C_e.cols, C_e.rows);
    matrix_trans(&C_e, &Ct_e);

    matrix_t CPpriCt_e;
    matrix_alloc(&CPpriCt_e, CPpri_e.rows, Ct_e.cols);

    matrix_t Sinv_e;
    matrix_alloc(&Sinv_e, S_e.rows, S_e.cols);

    matrix_t PpriCt_e;
    matrix_alloc(&PpriCt_e, Ppri_e.rows, Ct_e.cols);

    matrix_t KE_e;
    matrix_alloc(&KE_e, K_e.rows, E_e.cols);

    matrix_t KS_e;
    matrix_alloc(&KS_e, K_e.rows, S_e.cols);

    matrix_t Kt_e;
    matrix_alloc(&Kt_e, K_e.cols, K_e.rows);

    matrix_t KSKt_e;
    matrix_alloc(&KSKt_e, KS_e.rows, Kt_e.cols);

    // state space model for height
    matrix_t A_h;
    matrix_alloc(&A_h, 2, 2);
    A_h.array[0][0] = 1.0;
    A_h.array[0][1] = dt;
    A_h.array[1][0] = 0.0;
    A_h.array[1][1] = 1.0;

    matrix_t B_h;
    matrix_alloc(&B_h, 2, 1);
    B_h.array[0][0] = 0.5*pow(dt, 2.0);
    B_h.array[1][0] = dt;

    matrix_t C_h;
    matrix_alloc(&C_h, 1, 2);
    C_h.array[0][0] = 1.0;
    C_h.array[0][1] = 0.0;

    // noise
    matrix_t V_h;
    matrix_alloc(&V_h, 2, 2);
    V_h.array[0][0] = pow(STD_DEV_V_H, 2.0);
    V_h.array[0][1] = 0.0;
    V_h.array[1][0] = 0.0;
    V_h.array[1][1] = pow(STD_DEV_V_H, 2.0);

    matrix_t W_h;
    matrix_alloc(&W_h, 1, 1);
    W_h.array[0][0] = pow(STD_DEV_W_H, 2.0);

    // initial states
    matrix_t Xpri_h;
    matrix_alloc(&Xpri_h, 2, 1);
    Xpri_h.array[0][0] = 0.0;
    Xpri_h.array[1][0] = 0.0;

    matrix_t Ppri_h;
    matrix_alloc(&Ppri_h, 2, 2);
    Ppri_h.array[0][0] = 1.0;
    Ppri_h.array[0][1] = 0.0;
    Ppri_h.array[1][0] = 0.0;
    Ppri_h.array[1][1] = 1.0;

    matrix_t Xpost_h;
    matrix_alloc(&Xpost_h, 2, 1);
    Xpost_h.array[0][0] = h_data;
    Xpost_h.array[1][0] = 0.0;

    matrix_t Ppost_h;
    matrix_alloc(&Ppost_h, 2, 2);
    Ppost_h.array[0][0] = 1.0;
    Ppost_h.array[0][1] = 0.0;
    Ppost_h.array[1][0] = 0.0;
    Ppost_h.array[1][1] = 1.0;

    matrix_t U_h;
    matrix_alloc(&U_h, 1, 1);
    U_h.array[0][0] = 0.0;

    matrix_t Y_h;
    matrix_alloc(&Y_h, 1, 1);
    Y_h.array[0][0] = 0.0;

    matrix_t E_h;
    matrix_alloc(&E_h, 1, 1);

    matrix_t S_h;
    matrix_alloc(&S_h, 1, 1);

    matrix_t K_h;
    matrix_alloc(&K_h, 2, 1);

    // auxilary matrices
    matrix_t AXpost_h;
    matrix_alloc(&AXpost_h, A_h.rows, Xpost_h.cols);

    matrix_t BU_h;
    matrix_alloc(&BU_h, B_h.rows, U_h.cols);

    matrix_t APpost_h;
    matrix_alloc(&APpost_h, A_h.rows, Ppost_h.cols);

    matrix_t At_h;
    matrix_alloc(&At_h, A_h.cols, A_h.rows);
    matrix_trans(&A_h, &At_h);

    matrix_t APpostAt_h;
    matrix_alloc(&APpostAt_h, APpost_h.rows, At_h.cols);

    matrix_t CXpri_h;
    matrix_alloc(&CXpri_h, C_h.rows, Xpri_h.cols);

    matrix_t CPpri_h;
    matrix_alloc(&CPpri_h, C_h.rows, Ppri_h.cols);

    matrix_t Ct_h;
    matrix_alloc(&Ct_h, C_h.cols, C_h.rows);
    matrix_trans(&C_h, &Ct_h);

    matrix_t CPpriCt_h;
    matrix_alloc(&CPpriCt_h, CPpri_h.rows, Ct_h.cols);

    matrix_t Sinv_h;
    matrix_alloc(&Sinv_h, S_h.rows, S_h.cols);

    matrix_t PpriCt_h;
    matrix_alloc(&PpriCt_h, Ppri_h.rows, Ct_h.cols);

    matrix_t KE_h;
    matrix_alloc(&KE_h, K_h.rows, E_h.cols);

    matrix_t KS_h;
    matrix_alloc(&KS_h, K_h.rows, S_h.cols);

    matrix_t Kt_h;
    matrix_alloc(&Kt_h, K_h.cols, K_h.rows);

    matrix_t KSKt_h;
    matrix_alloc(&KSKt_h, KS_h.rows, Kt_h.cols);

    // time variables
    TickType_t time = 0;
    TickType_t last_time = 0;
    TickType_t mutex_wait = 0;

    // Kalman filter
    while (1)
    {
        last_time = xTaskGetTickCount();

        // new measurement
        imu_get_data(&acce_data, &gyro_data, &mag_data, &h_data);

        calculate_euler_angle_from_accel(&acce_data, &mag_data, &acce_euler_angle);

        U_e.array[0][0] = gyro_data.gyro_x;
        U_e.array[0][1] = gyro_data.gyro_y;
        U_e.array[0][2] = gyro_data.gyro_z;

        Y_e.array[0][0] = acce_euler_angle.roll;
        Y_e.array[0][1] = acce_euler_angle.pitch;
        Y_e.array[0][2] = acce_euler_angle.yaw;

        // euler angles kalman

        // Xpri_e
        matrix_mul(&A_e, &Xpost_e, &AXpost_e);
        matrix_mul(&B_e, &U_e, &BU_e);
        matrix_add(&AXpost_e, &BU_e, &Xpri_e);

        // Ppri_e
        matrix_mul(&A_e, &Ppost_e, &APpost_e);
        matrix_mul(&APpost_e, &At_e, &APpostAt_e);
        matrix_add(&APpostAt_e, &V_e, &Ppri_e);

        // E_e
        matrix_mul(&C_e, &Xpri_e, &CXpri_e);
        matrix_sub(&Y_e, &CXpri_e, &E_e);

        // S_e
        matrix_mul(&C_e, &Ppri_e, &CPpri_e);
        matrix_mul(&CPpri_e, &Ct_e, &CPpriCt_e);
        matrix_add(&CPpriCt_e, &W_e, &S_e);

        // K_e
        matrix_inv(&S_e, &Sinv_e);
        matrix_mul(&Ppri_e, &Ct_e, &PpriCt_e);
        matrix_mul(&PpriCt_e, &Sinv_e, &K_e);

        // Xpost_e
        matrix_mul(&K_e, &E_e, &KE_e);
        matrix_add(&Xpri_e, &KE_e, &Xpost_e);

        // Ppost_e
        matrix_mul(&K_e, &S_e, &KS_e);
        matrix_trans(&K_e, &Kt_e);
        matrix_mul(&KS_e, &Kt_e, &KSKt_e);
        matrix_sub(&Ppri_e, &KSKt_e, &Ppost_e);

        // calculate euler angle from gyro
        task_kalman_data.acce_roll = Xpost_e.array[0][0];
        task_kalman_data.acce_pitch = Xpost_e.array[0][1];
        task_kalman_data.mag_yaw = Xpost_e.array[0][2];

        task_kalman_data.gyro_roll += (gyro_data.gyro_x - Xpost_e.array[1][0]) * dt;
        task_kalman_data.gyro_pitch += (gyro_data.gyro_y - Xpost_e.array[1][1]) * dt;
        task_kalman_data.gyro_yaw += (gyro_data.gyro_z - Xpost_e.array[1][2]) * dt;

        // height kalman

        U_h.array[0][0] = calculate_z_accel(&acce_data, &task_kalman_data);
        Y_h.array[0][0] = h_data;

        // Xpri_h
        matrix_mul(&A_h, &Xpost_h, &AXpost_h);
        matrix_mul(&B_h, &U_h, &BU_h);
        matrix_add(&AXpost_h, &BU_h, &Xpri_h);

        // Ppri_h
        matrix_mul(&A_h, &Ppost_h, &APpost_h);
        matrix_mul(&APpost_h, &At_h, &APpostAt_h);
        matrix_add(&APpostAt_h, &V_h, &Ppri_h);

        // E_h
        matrix_mul(&C_h, &Xpri_h, &CXpri_h);
        matrix_sub(&Y_h, &CXpri_h, &E_h);

        // S_h
        matrix_mul(&C_h, &Ppri_h, &CPpri_h);
        matrix_mul(&CPpri_h, &Ct_h, &CPpriCt_h);
        matrix_add(&CPpriCt_h, &W_h, &S_h);

        // K_h
        matrix_inv(&S_h, &Sinv_h);
        matrix_mul(&Ppri_h, &Ct_h, &PpriCt_h);
        matrix_mul(&PpriCt_h, &Sinv_h, &K_h);

        // Xpost_h
        matrix_mul(&K_h, &E_h, &KE_h);
        matrix_add(&Xpri_h, &KE_h, &Xpost_h);

        // Ppost_h
        matrix_mul(&K_h, &S_h, &KS_h);
        matrix_trans(&K_h, &Kt_h);
        matrix_mul(&KS_h, &Kt_h, &KSKt_h);
        matrix_sub(&Ppri_h, &KSKt_h, &Ppost_h);

        // update height
        task_kalman_data.height = Xpost_h.array[0][0];

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

    // deallocate memory
    matrix_dealloc(&A_e);
    matrix_dealloc(&B_e);
    matrix_dealloc(&C_e);
    matrix_dealloc(&V_e);
    matrix_dealloc(&W_e);
    matrix_dealloc(&E_e);
    matrix_dealloc(&S_e);
    matrix_dealloc(&K_e);
    matrix_dealloc(&U_e);
    matrix_dealloc(&Y_e);
    matrix_dealloc(&Xpri_e);
    matrix_dealloc(&Ppri_e);
    matrix_dealloc(&Xpost_e);
    matrix_dealloc(&Ppost_e);
    matrix_dealloc(&AXpost_e);
    matrix_dealloc(&BU_e);
    matrix_dealloc(&APpost_e);
    matrix_dealloc(&At_e);
    matrix_dealloc(&APpostAt_e);
    matrix_dealloc(&CXpri_e);
    matrix_dealloc(&CPpri_e);
    matrix_dealloc(&Ct_e);
    matrix_dealloc(&CPpriCt_e);
    matrix_dealloc(&Sinv_e);
    matrix_dealloc(&PpriCt_e);
    matrix_dealloc(&KE_e);
    matrix_dealloc(&KS_e);
    matrix_dealloc(&Kt_e);
    matrix_dealloc(&KSKt_e);

    matrix_dealloc(&A_h);
    matrix_dealloc(&B_h);
    matrix_dealloc(&C_h);
    matrix_dealloc(&V_h);
    matrix_dealloc(&W_h);
    matrix_dealloc(&E_h);
    matrix_dealloc(&S_h);
    matrix_dealloc(&K_h);
    matrix_dealloc(&U_h);
    matrix_dealloc(&Y_h);
    matrix_dealloc(&Xpri_h);
    matrix_dealloc(&Ppri_h);
    matrix_dealloc(&Xpost_h);
    matrix_dealloc(&Ppost_h);
    matrix_dealloc(&AXpost_h);
    matrix_dealloc(&BU_h);
    matrix_dealloc(&APpost_h);
    matrix_dealloc(&At_h);
    matrix_dealloc(&APpostAt_h);
    matrix_dealloc(&CXpri_h);
    matrix_dealloc(&CPpri_h);
    matrix_dealloc(&Ct_h);
    matrix_dealloc(&CPpriCt_h);
    matrix_dealloc(&Sinv_h);
    matrix_dealloc(&PpriCt_h);
    matrix_dealloc(&KE_h);
    matrix_dealloc(&KS_h);
    matrix_dealloc(&Kt_h);
    matrix_dealloc(&KSKt_h);

    vTaskDelete(NULL);
}


/**
 * @brief initialize imu sensor
 * 
 * @param imu_conf imu I2C configuration
*/
void imu_init(imu_i2c_conf_t imu_conf)
{
    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // initialize I2C
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = imu_conf.sda_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = imu_conf.scl_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = imu_conf.i2c_freq;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    ESP_ERROR_CHECK(i2c_param_config(imu_conf.i2c_num, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(imu_conf.i2c_num, conf.mode, 0, 0, 0));

    // initialize QMC5883L sensor
    qmc.i2c_port = imu_conf.i2c_num;
    qmc.sda_pin = imu_conf.sda_pin;
    qmc.scl_pin = imu_conf.scl_pin;
    qmc.i2c_freq = imu_conf.i2c_freq;
    qmc.drdy_pin = -1;

    qmc5883l_init(qmc);
    qmc5883l_write_control(qmc, QMC5883L_OVER_SAMPLE_RATIO_512, QMC5883L_FULL_SCALE_8G,
                        QMC5883L_DATA_OUTPUT_RATE_200, QMC5883L_CONTINUOUS_MODE, 
                        QMC5883L_POINTER_ROLLOVER_FUNCTION_NORMAL, QMC5883L_INTERRUPT_DISABLE);

    // initialize BMP280 sensor
    bmp.i2c_port = imu_conf.i2c_num;
    bmp.i2c_addr = BMP_I2C_ADDRESS_1;
    bmp.sda_pin = imu_conf.sda_pin;
    bmp.scl_pin = imu_conf.scl_pin;
    bmp.i2c_freq = imu_conf.i2c_freq;
    bmp280_init(bmp, BMP280_ULTRA_HIGH_RES);

    // initialize MPU6050 sensor
    mpu = mpu6050_create(imu_conf.i2c_num, MPU6050_I2C_ADDRESS);
    if (mpu == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mpu6050");
        return;
    }

    ESP_ERROR_CHECK(mpu6050_wake_up(mpu));

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

    xTaskCreate(kalman_data_read, "kalman filter", 4096, NULL, 10, NULL);
}


/**
 * @brief get data from sensors
 * 
 * @param acce acce data
 * @param gyro gyro data
 * @param mag magnetometer data
 * @param height height data
*/
void imu_get_data(mpu6050_acce_value_t* acce, mpu6050_gyro_value_t* gyro, magnetometer_raw_t* mag, float* height)
{
    ESP_ERROR_CHECK(mpu6050_get_acce(mpu, acce));
    ESP_ERROR_CHECK(mpu6050_get_gyro(mpu, gyro));

    // MPU6050 is mounted backwards
    acce->acce_y = -acce->acce_y;
    gyro->gyro_x = -gyro->gyro_x;
    gyro->gyro_y = -gyro->gyro_y;
    gyro->gyro_z = -gyro->gyro_z;

    float x, y, z;
    qmc5883l_read_magnetometer(qmc, &x, &y, &z);

    mag->x = y;  // MPU6050 OX axis is QMC OY axis
    mag->y = -x; // MPU6050 OY axis is QMC -OX axis
    mag->z = z;  // MPU6050 OZ axis is QMC OZ axis

    double pres_height = 0.0;
    bmp280_read_height(bmp, &pres_height);

    *height = (float)pres_height;
}


/**
 * @brief get data from kalman filter
 * 
 * @param kalman_data kalman data
*/
void imu_get_kalman_data(kalman_data_t* kalman_data)
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
