idf_component_register(SRCS "kalman_10dof_imu.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_mpu6050 esp_hmc5883l esp_bmp280 esp_tf-lc02 espressif__esp-dsp)
