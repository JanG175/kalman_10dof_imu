idf_component_register(SRCS "kalman_10dof_imu.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_mpu6050 esp_matrix esp_hmc5883l esp_bmp280)
