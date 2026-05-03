#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <zephyr/drivers/sensor.h>

struct imu_data {
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    struct sensor_value magn[3];
};

/**
 * Initializes the IMU sensors and sets up triggers if supported.
 * @return 0 on success, negative error code on failure.
 */
int imu_init(void);

/**
 * Polls any sensors that fell back to polling mode.
 * Automatically skips sensors handled by interrupts.
 */
void imu_poll_update(void);

/**
 * Retrieves the latest copied IMU data safely.
 * @param out_data Pointer to a buffer where the data will be copied.
 */
void imu_get_latest_data(struct imu_data *out_data);

/**
 * Helper to print the IMU data to the console using the Zephyr logging subsystem.
 */
void imu_print_data(void);

/**
 * Computes and prints the actual frequency (Hz) of each sensor.
 * Designed to be called continuously; will only print once per second.
 */
void imu_analyze_frequency(void);

#endif /* IMU_SENSOR_H */
