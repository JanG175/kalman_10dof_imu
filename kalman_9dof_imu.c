/**
 * @file kalman_9dof_imu.c
 * @author JanG175
 * @brief 9DOF IMU sensor made from sensor fusion of MPU6050 accelerometer and gyroscope and QMC5883L magnetometer
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "kalman_9dof_imu.h"

static const char* TAG = "kalman_9_dof_imu";
static mpu6050_handle_t mpu;
static qmc5883l_conf_t qmc;
static bmp280_conf_t bmp;

static SemaphoreHandle_t mutex = NULL;
static kalman_data_t static_kalman_data;


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
    double std_dev_v = STD_DEV_V;
    double std_dev_w = STD_DEV_W;

    // init measurement
    mpu6050_acce_value_t acce_data;
    mpu6050_gyro_value_t gyro_data;
    magnetometer_raw_t mag_data;
    float pres_h_data;

    imu_get_data(&acce_data, &gyro_data, &mag_data, &pres_h_data);

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

    Xpri' = A * Xpri + B * U
      Y   = C * Xpri + D * U

    --    --   --     --   --   --   --  --            
    | phi' |   | 1 -dt |   | phi |   | dt |   --     --
    |      | = |       | * |     | + |    | * | Wgyro |
    |  g'  |   | 0   1 |   |  g  |   |  0 |   --     --
    --    --   --     --   --   --   --  --            

                  --   --
        --   --   | phi |
    Y = | 1 0 | * |     |
        --   --   |  g  |
                  --   --

    */

    // state space model
    matrix_t A;
    matrix_alloc(&A, 2, 2);
    A.array[0][0] = 1.0;
    A.array[0][1] = -dt;
    A.array[1][0] = 0.0;
    A.array[1][1] = 1.0;

    matrix_t B;
    matrix_alloc(&B, 2, 1);
    B.array[0][0] = dt;
    B.array[1][0] = 0.0;

    matrix_t C;
    matrix_alloc(&C, 1, 2);
    C.array[0][0] = 1.0;
    C.array[0][1] = 0.0;

    // noise
    matrix_t V;
    matrix_alloc(&V, 2, 2);
    V.array[0][0] = pow(std_dev_v, 2.0);
    V.array[0][1] = 0.0;
    V.array[1][0] = 0.0;
    V.array[1][1] = pow(std_dev_v, 2.0);

    matrix_t W;
    matrix_alloc(&W, 1, 1);
    W.array[0][0] = pow(std_dev_w, 2.0);

    // initial states
    matrix_t Xpri;
    matrix_alloc(&Xpri, 2, 3);
    Xpri.array[0][0] = 0.0;
    Xpri.array[0][1] = 0.0;
    Xpri.array[0][2] = 0.0;
    Xpri.array[1][0] = 0.0;
    Xpri.array[1][1] = 0.0;
    Xpri.array[1][2] = 0.0;

    matrix_t Ppri;
    matrix_alloc(&Ppri, 2, 2);
    Ppri.array[0][0] = 1.0;
    Ppri.array[0][1] = 0.0;
    Ppri.array[1][0] = 0.0;
    Ppri.array[1][1] = 1.0;

    matrix_t Xpost;
    matrix_alloc(&Xpost, 2, 3);
    Xpost.array[0][0] = acce_euler_angle.roll;
    Xpost.array[0][1] = acce_euler_angle.pitch;
    Xpost.array[0][2] = acce_euler_angle.yaw;
    Xpost.array[1][0] = 0.0;
    Xpost.array[1][1] = 0.0;
    Xpost.array[1][2] = 0.0;

    matrix_t Ppost;
    matrix_alloc(&Ppost, 2, 2);
    Ppost.array[0][0] = 1.0;
    Ppost.array[0][1] = 0.0;
    Ppost.array[1][0] = 0.0;
    Ppost.array[1][1] = 1.0;

    matrix_t U;
    matrix_alloc(&U, 1, 3);
    U.array[0][0] = 0.0;
    U.array[0][1] = 0.0;
    U.array[0][2] = 0.0;

    matrix_t Y;
    matrix_alloc(&Y, 1, 3);
    U.array[0][0] = 0.0;
    U.array[0][1] = 0.0;
    U.array[0][2] = 0.0;

    matrix_t E;
    matrix_alloc(&E, 1, 3);

    matrix_t S;
    matrix_alloc(&S, 1, 1);

    matrix_t K;
    matrix_alloc(&K, 2, 1);

    // auxilary matrices
    matrix_t AXpost;
    matrix_alloc(&AXpost, A.rows, Xpost.cols);

    matrix_t BU;
    matrix_alloc(&BU, B.rows, U.cols);

    matrix_t APpost;
    matrix_alloc(&APpost, A.rows, Ppost.cols);

    matrix_t At;
    matrix_alloc(&At, A.cols, A.rows);
    matrix_trans(&A, &At);

    matrix_t APpostAt;
    matrix_alloc(&APpostAt, APpost.rows, At.cols);

    matrix_t CXpri;
    matrix_alloc(&CXpri, C.rows, Xpri.cols);

    matrix_t CPpri;
    matrix_alloc(&CPpri, C.rows, Ppri.cols);

    matrix_t Ct;
    matrix_alloc(&Ct, C.cols, C.rows);
    matrix_trans(&C, &Ct);

    matrix_t CPpriCt;
    matrix_alloc(&CPpriCt, CPpri.rows, Ct.cols);

    matrix_t Sinv;
    matrix_alloc(&Sinv, S.rows, S.cols);

    matrix_t PpriCt;
    matrix_alloc(&PpriCt, Ppri.rows, Ct.cols);

    matrix_t KE;
    matrix_alloc(&KE, K.rows, E.cols);

    matrix_t KS;
    matrix_alloc(&KS, K.rows, S.cols);

    matrix_t Kt;
    matrix_alloc(&Kt, K.cols, K.rows);

    matrix_t KSKt;
    matrix_alloc(&KSKt, KS.rows, Kt.cols);

    TickType_t time = 0;
    TickType_t last_time = 0;
    TickType_t mutex_wait = 0;

    // Kalman filter
    while (1)
    {
        last_time = xTaskGetTickCount();

        // new measurement
        imu_get_data(&acce_data, &gyro_data, &mag_data, &pres_h_data);

        calculate_euler_angle_from_accel(&acce_data, &mag_data, &acce_euler_angle);

        U.array[0][0] = gyro_data.gyro_x;
        U.array[0][1] = gyro_data.gyro_y;
        U.array[0][2] = gyro_data.gyro_z;

        Y.array[0][0] = acce_euler_angle.roll;
        Y.array[0][1] = acce_euler_angle.pitch;
        Y.array[0][2] = acce_euler_angle.yaw;

        // Xpri
        matrix_mul(&A, &Xpost, &AXpost);
        matrix_mul(&B, &U, &BU);
        matrix_add(&AXpost, &BU, &Xpri);

        // Ppri
        matrix_mul(&A, &Ppost, &APpost);
        matrix_mul(&APpost, &At, &APpostAt);
        matrix_add(&APpostAt, &V, &Ppri);

        // E
        matrix_mul(&C, &Xpri, &CXpri);
        matrix_sub(&Y, &CXpri, &E);

        // S
        matrix_mul(&C, &Ppri, &CPpri);
        matrix_mul(&CPpri, &Ct, &CPpriCt);
        matrix_add(&CPpriCt, &W, &S);

        // K
        matrix_inv(&S, &Sinv);
        matrix_mul(&Ppri, &Ct, &PpriCt);
        matrix_mul(&PpriCt, &Sinv, &K);

        // Xpost
        matrix_mul(&K, &E, &KE);
        matrix_add(&Xpri, &KE, &Xpost);

        // Ppost
        matrix_mul(&K, &S, &KS);
        matrix_trans(&K, &Kt);
        matrix_mul(&KS, &Kt, &KSKt);
        matrix_sub(&Ppri, &KSKt, &Ppost);

        // calculate euler angle from gyro
        task_kalman_data.acce_roll = Xpost.array[0][0];
        task_kalman_data.acce_pitch = Xpost.array[0][1];
        task_kalman_data.mag_yaw = Xpost.array[0][2];

        task_kalman_data.gyro_roll += (gyro_data.gyro_x - Xpost.array[1][0]) * dt;
        task_kalman_data.gyro_pitch += (gyro_data.gyro_y - Xpost.array[1][1]) * dt;
        task_kalman_data.gyro_yaw += (gyro_data.gyro_z - Xpost.array[1][2]) * dt;

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

            static_kalman_data.pres_height = pres_h_data;

            xSemaphoreGive(mutex);

            xTaskDelayUntil(&last_time, DT / portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to take mutex to update");
        }
    }

    // deallocate memory
    matrix_dealloc(&A);
    matrix_dealloc(&B);
    matrix_dealloc(&C);
    matrix_dealloc(&V);
    matrix_dealloc(&W);
    matrix_dealloc(&E);
    matrix_dealloc(&S);
    matrix_dealloc(&K);
    matrix_dealloc(&U);
    matrix_dealloc(&Y);
    matrix_dealloc(&Xpri);
    matrix_dealloc(&Ppri);
    matrix_dealloc(&Xpost);
    matrix_dealloc(&Ppost);
    matrix_dealloc(&AXpost);
    matrix_dealloc(&BU);
    matrix_dealloc(&APpost);
    matrix_dealloc(&At);
    matrix_dealloc(&APpostAt);
    matrix_dealloc(&CXpri);
    matrix_dealloc(&CPpri);
    matrix_dealloc(&Ct);
    matrix_dealloc(&CPpriCt);
    matrix_dealloc(&Sinv);
    matrix_dealloc(&PpriCt);
    matrix_dealloc(&KE);
    matrix_dealloc(&KS);
    matrix_dealloc(&Kt);
    matrix_dealloc(&KSKt);

    vTaskDelete(NULL);
}


/**
 * @brief initialize imu sensor
 * 
 * @param mpu_conf imu I2C configuration
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

        xSemaphoreGive(mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take mutex to init");
    }

    xTaskCreate(kalman_data_read, "kalman filter", 4096, NULL, 10, NULL);
}


/**
 * @brief get data from imu sensor
 * 
 * @param acce acce data
 * @param gyro gyro data
 * @param mag magnetometer data
 * @param pres_h pressure sensor height data
*/
void imu_get_data(mpu6050_acce_value_t* acce, mpu6050_gyro_value_t* gyro, magnetometer_raw_t* mag, float* pres_h)
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

    double height;
    bmp280_read_height(bmp, &height);

    *pres_h = (float)height;
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

        kalman_data->pres_height = static_kalman_data.pres_height;

        xSemaphoreGive(mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take mutex to read");
    }
}
