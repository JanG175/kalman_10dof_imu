idf_component_register(SRCS "kalman_10dof_imu.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver espressif__mpu6050 esp_matrix esp_qmc5883l esp_bmp280)
