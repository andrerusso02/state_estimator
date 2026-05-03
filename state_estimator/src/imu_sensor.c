#include "imu_sensor.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>

#define ACCEL_NODE DT_ALIAS(accel0)
#define GYRO_NODE  DT_ALIAS(gyro0)
#define MAGN_NODE  DT_ALIAS(magn0)

#if !DT_NODE_HAS_STATUS(ACCEL_NODE, okay) || \
    !DT_NODE_HAS_STATUS(GYRO_NODE, okay) || \
    !DT_NODE_HAS_STATUS(MAGN_NODE, okay)
#error "Unsupported board: accel0, gyro0, and/or magn0 aliases missing."
#endif

static const struct device *accel_dev = DEVICE_DT_GET(ACCEL_NODE);
static const struct device *gyro_dev  = DEVICE_DT_GET(GYRO_NODE);
static const struct device *magn_dev  = DEVICE_DT_GET(MAGN_NODE);

static struct imu_data current_data;
static bool use_poll_accel, use_poll_gyro, use_poll_magn;

/* Helper to concisely extract X, Y, Z channels */
static void extract_xyz(const struct device *dev, 
                        enum sensor_channel cx, enum sensor_channel cy, enum sensor_channel cz, 
                        struct sensor_value *out) 
{
    sensor_channel_get(dev, cx, &out[0]);
    sensor_channel_get(dev, cy, &out[1]);
    sensor_channel_get(dev, cz, &out[2]);
}

/* Single fetch per physical device, handles bundled chips automatically */
static void read_device_data(const struct device *dev)
{
    if (sensor_sample_fetch(dev) < 0) {
        return;
    }
    if (dev == accel_dev) {
        extract_xyz(dev, SENSOR_CHAN_ACCEL_X, SENSOR_CHAN_ACCEL_Y, SENSOR_CHAN_ACCEL_Z, current_data.accel);
    }
    if (dev == gyro_dev) {
        extract_xyz(dev, SENSOR_CHAN_GYRO_X, SENSOR_CHAN_GYRO_Y, SENSOR_CHAN_GYRO_Z, current_data.gyro);
    }
    if (dev == magn_dev) {
        extract_xyz(dev, SENSOR_CHAN_MAGN_X, SENSOR_CHAN_MAGN_Y, SENSOR_CHAN_MAGN_Z, current_data.magn);
    }
}

static void trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
    read_device_data(dev);
}

int imu_init(void)
{
    if (!device_is_ready(accel_dev) || !device_is_ready(gyro_dev) || !device_is_ready(magn_dev)) {
        printk("One or more IMU devices not ready.\n");
        return -ENODEV;
    }

    struct sensor_trigger trig = { .type = SENSOR_TRIG_DATA_READY, .chan = SENSOR_CHAN_ALL };

    /* Accel Trigger setup */
    if (sensor_trigger_set(accel_dev, &trig, trigger_handler) < 0) {
        use_poll_accel = true;
        printk("Accelerometer in polling mode.\n");
    }

    /* Gyro Trigger setup */
    if (gyro_dev != accel_dev) {
        if (sensor_trigger_set(gyro_dev, &trig, trigger_handler) < 0) {
            use_poll_gyro = true;
            printk("Gyroscope in polling mode.\n");
        }
    } else {
        use_poll_gyro = use_poll_accel;
    }

    /* Magn Trigger setup */
    if (magn_dev != accel_dev && magn_dev != gyro_dev) {
        if (sensor_trigger_set(magn_dev, &trig, trigger_handler) < 0) {
            use_poll_magn = true;
            printk("Magnetometer in polling mode.\n");
        }
    } else {
        use_poll_magn = (magn_dev == accel_dev) ? use_poll_accel : use_poll_gyro;
    }

    return 0;
}

void imu_poll_update(void)
{
    /* Only fetch if polling is required, and prevent duplicate fetches for bundled sensors */
    if (use_poll_accel) {
        read_device_data(accel_dev);
    }
    if (use_poll_gyro && gyro_dev != accel_dev) {
        read_device_data(gyro_dev);
    }
    if (use_poll_magn && magn_dev != accel_dev && magn_dev != gyro_dev) {
        read_device_data(magn_dev);
    }
}

const struct imu_data* imu_get_latest_data(void)
{
    return &current_data;
}

void imu_print_data(void)
{
    printf("9-Axis Sensor Data:\n");
    printf("Accel x:%f y:%f z:%f\n", 
           sensor_value_to_double(&current_data.accel[0]), 
           sensor_value_to_double(&current_data.accel[1]), 
           sensor_value_to_double(&current_data.accel[2]));
    printf("Gyro  x:%f y:%f z:%f\n", 
           sensor_value_to_double(&current_data.gyro[0]), 
           sensor_value_to_double(&current_data.gyro[1]), 
           sensor_value_to_double(&current_data.gyro[2]));
    printf("Magn  x:%f y:%f z:%f\n\n", 
           sensor_value_to_double(&current_data.magn[0]), 
           sensor_value_to_double(&current_data.magn[1]), 
           sensor_value_to_double(&current_data.magn[2]));
}
