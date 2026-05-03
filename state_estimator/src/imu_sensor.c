#include "imu_sensor.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <stdint.h>

#define ACCEL_NODE DT_ALIAS(accel0)
#define GYRO_NODE  DT_ALIAS(gyro0)
#define MAGN_NODE  DT_ALIAS(magn0)

#if !DT_NODE_HAS_STATUS(ACCEL_NODE, okay) || \
    !DT_NODE_HAS_STATUS(GYRO_NODE, okay) || \
    !DT_NODE_HAS_STATUS(MAGN_NODE, okay)
#error "Unsupported board: accel0, gyro0, and/or magn0 aliases missing."
#endif

/* Define your expected polling intervals based on your Kconfig ODRs */
#define POLL_INTERVAL_ACCEL_MS 10  /* 100 Hz */
#define POLL_INTERVAL_GYRO_MS  10  /* 100 Hz */
#define POLL_INTERVAL_MAGN_MS  6   /* ~166 Hz (Closest to 155Hz) */

static const struct device *accel_dev = DEVICE_DT_GET(ACCEL_NODE);
static const struct device *gyro_dev  = DEVICE_DT_GET(GYRO_NODE);
static const struct device *magn_dev  = DEVICE_DT_GET(MAGN_NODE);

static struct imu_data current_data;
static bool use_poll_accel, use_poll_gyro, use_poll_magn;

static uint32_t accel_count = 0;
static uint32_t gyro_count = 0;
static uint32_t magn_count = 0;
static uint32_t last_freq_calc_time = 0;

/* Polling rate limit trackers */
static uint32_t last_poll_accel = 0;
static uint32_t last_poll_gyro = 0;
static uint32_t last_poll_magn = 0;

static void extract_xyz(const struct device *dev, 
                        enum sensor_channel cx, enum sensor_channel cy, enum sensor_channel cz, 
                        struct sensor_value *out) 
{
    sensor_channel_get(dev, cx, &out[0]);
    sensor_channel_get(dev, cy, &out[1]);
    sensor_channel_get(dev, cz, &out[2]);
}

static void read_device_data(const struct device *dev)
{
    if (sensor_sample_fetch(dev) < 0) {
        return;
    }
    if (dev == accel_dev) {
        extract_xyz(dev, SENSOR_CHAN_ACCEL_X, SENSOR_CHAN_ACCEL_Y, SENSOR_CHAN_ACCEL_Z, current_data.accel);
        accel_count++;
    }
    if (dev == gyro_dev) {
        extract_xyz(dev, SENSOR_CHAN_GYRO_X, SENSOR_CHAN_GYRO_Y, SENSOR_CHAN_GYRO_Z, current_data.gyro);
        gyro_count++;
    }
    if (dev == magn_dev) {
        extract_xyz(dev, SENSOR_CHAN_MAGN_X, SENSOR_CHAN_MAGN_Y, SENSOR_CHAN_MAGN_Z, current_data.magn);
        magn_count++;
    }
}

void imu_analyze_frequency(void)
{
    uint32_t now = k_uptime_get_32();
    uint32_t delta_ms = now - last_freq_calc_time;

    if (delta_ms >= 1000) {
        if (last_freq_calc_time != 0) {
            uint32_t accel_hz = (accel_count * 1000) / delta_ms;
            uint32_t gyro_hz  = (gyro_count * 1000) / delta_ms;
            uint32_t magn_hz  = (magn_count * 1000) / delta_ms;

            printf("[ANALYSIS] Actual ODR -> Accel: %u Hz | Gyro: %u Hz | Magn: %u Hz\n",
                   accel_hz, gyro_hz, magn_hz);
        }
        accel_count = 0;
        gyro_count = 0;
        magn_count = 0;
        last_freq_calc_time = now;
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

    if (sensor_trigger_set(accel_dev, &trig, trigger_handler) < 0) {
        use_poll_accel = true;
        printk("Accelerometer in polling mode.\n");
    }

    if (gyro_dev != accel_dev) {
        if (sensor_trigger_set(gyro_dev, &trig, trigger_handler) < 0) {
            use_poll_gyro = true;
            printk("Gyroscope in polling mode.\n");
        }
    } else {
        use_poll_gyro = use_poll_accel;
    }

    if (magn_dev != accel_dev && magn_dev != gyro_dev) {
        if (sensor_trigger_set(magn_dev, &trig, trigger_handler) < 0) {
            use_poll_magn = true;
            printk("Magnetometer in polling mode.\n");
        }
    } else {
        use_poll_magn = (magn_dev == accel_dev) ? use_poll_accel : use_poll_gyro;
    }

    /* Initialize polling timers */
    uint32_t now = k_uptime_get_32();
    last_poll_accel = now;
    last_poll_gyro = now;
    last_poll_magn = now;

    return 0;
}

void imu_poll_update(void)
{
    uint32_t now = k_uptime_get_32();

    /* Rate-limited Accel Fetch */
    if (use_poll_accel && (now - last_poll_accel >= POLL_INTERVAL_ACCEL_MS)) {
        read_device_data(accel_dev);
        last_poll_accel = now;
        
        /* Synchronize timers for bundled sensors to prevent immediate duplicate fetches */
        if (gyro_dev == accel_dev) last_poll_gyro = now;
        if (magn_dev == accel_dev) last_poll_magn = now;
    }

    /* Rate-limited Gyro Fetch (if separate from Accel) */
    if (use_poll_gyro && gyro_dev != accel_dev && (now - last_poll_gyro >= POLL_INTERVAL_GYRO_MS)) {
        read_device_data(gyro_dev);
        last_poll_gyro = now;
        
        if (magn_dev == gyro_dev) last_poll_magn = now;
    }

    /* Rate-limited Magn Fetch (if separate from Accel and Gyro) */
    if (use_poll_magn && magn_dev != accel_dev && magn_dev != gyro_dev && (now - last_poll_magn >= POLL_INTERVAL_MAGN_MS)) {
        read_device_data(magn_dev);
        last_poll_magn = now;
    }
}