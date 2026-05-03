#include "imu_sensor.h"
#include "freq_monitor.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <stdint.h>

LOG_MODULE_REGISTER(imu_sensor, LOG_LEVEL_INF);

#define ACCEL_NODE DT_ALIAS(accel0)
#define GYRO_NODE  DT_ALIAS(gyro0)
#define MAGN_NODE  DT_ALIAS(magn0)

#if !DT_NODE_HAS_STATUS(ACCEL_NODE, okay) || \
    !DT_NODE_HAS_STATUS(GYRO_NODE, okay) || \
    !DT_NODE_HAS_STATUS(MAGN_NODE, okay)
#error "Unsupported board: accel0, gyro0, and/or magn0 aliases missing."
#endif

/* Polling intervals based on Kconfig ODRs (104 Hz and 155 Hz) */
#define POLL_INTERVAL_ACCEL_MS 9   /* ~111 Hz to ensure we don't miss 104 Hz samples */
#define POLL_INTERVAL_GYRO_MS  9   
#define POLL_INTERVAL_MAGN_MS  6   /* ~166 Hz for 155 Hz target */

static const struct device *accel_dev = DEVICE_DT_GET(ACCEL_NODE);
static const struct device *gyro_dev  = DEVICE_DT_GET(GYRO_NODE);
static const struct device *magn_dev  = DEVICE_DT_GET(MAGN_NODE);

K_MUTEX_DEFINE(imu_mutex);
static struct imu_data current_data;

static bool use_poll_accel = false;
static bool use_poll_gyro  = false;
static bool use_poll_magn  = false;

static struct freq_monitor accel_freq;
static struct freq_monitor gyro_freq;
static struct freq_monitor magn_freq;

/* 64-bit trackers to prevent 49.7-day rollover bugs */
static int64_t last_poll_accel     = 0;
static int64_t last_poll_gyro      = 0;
static int64_t last_poll_magn      = 0;

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
        LOG_ERR("Failed to fetch sample from %s", dev->name);
        return;
    }

    k_mutex_lock(&imu_mutex, K_FOREVER);
    if (dev == accel_dev) {
        extract_xyz(dev, SENSOR_CHAN_ACCEL_X, SENSOR_CHAN_ACCEL_Y, SENSOR_CHAN_ACCEL_Z, current_data.accel);
        freq_monitor_tick(&accel_freq);
    }
    if (dev == gyro_dev) {
        extract_xyz(dev, SENSOR_CHAN_GYRO_X, SENSOR_CHAN_GYRO_Y, SENSOR_CHAN_GYRO_Z, current_data.gyro);
        freq_monitor_tick(&gyro_freq);
    }
    if (dev == magn_dev) {
        extract_xyz(dev, SENSOR_CHAN_MAGN_X, SENSOR_CHAN_MAGN_Y, SENSOR_CHAN_MAGN_Z, current_data.magn);
        freq_monitor_tick(&magn_freq);
    }
    k_mutex_unlock(&imu_mutex);
}

static void trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
    read_device_data(dev);
}

int imu_init(void)
{
    if (!device_is_ready(accel_dev) || !device_is_ready(gyro_dev) || !device_is_ready(magn_dev)) {
        LOG_ERR("One or more IMU devices not ready.");
        return -ENODEV;
    }

    struct sensor_trigger trig = { .type = SENSOR_TRIG_DATA_READY, .chan = SENSOR_CHAN_ALL };

    if (sensor_trigger_set(accel_dev, &trig, trigger_handler) < 0) {
        use_poll_accel = true;
        LOG_WRN("Accelerometer in polling mode.");
    }

    if (gyro_dev != accel_dev) {
        if (sensor_trigger_set(gyro_dev, &trig, trigger_handler) < 0) {
            use_poll_gyro = true;
            LOG_WRN("Gyroscope in polling mode.");
        }
    } else {
        use_poll_gyro = use_poll_accel;
    }

    if (magn_dev != accel_dev && magn_dev != gyro_dev) {
        if (sensor_trigger_set(magn_dev, &trig, trigger_handler) < 0) {
            use_poll_magn = true;
            LOG_WRN("Magnetometer in polling mode.");
        }
    } else {
        use_poll_magn = (magn_dev == accel_dev) ? use_poll_accel : use_poll_gyro;
    }

    int64_t now = k_uptime_get();
    last_poll_accel = now;
    last_poll_gyro  = now;
    last_poll_magn  = now;

    LOG_INF("IMU initialized successfully.");
    return 0;
}

void imu_poll_update(void)
{
    int64_t now = k_uptime_get();

    if (use_poll_accel && (now - last_poll_accel >= POLL_INTERVAL_ACCEL_MS)) {
        read_device_data(accel_dev);
        last_poll_accel = now;
        
        if (gyro_dev == accel_dev) last_poll_gyro = now;
        if (magn_dev == accel_dev) last_poll_magn = now;
    }

    if (use_poll_gyro && gyro_dev != accel_dev && (now - last_poll_gyro >= POLL_INTERVAL_GYRO_MS)) {
        read_device_data(gyro_dev);
        last_poll_gyro = now;
        
        if (magn_dev == gyro_dev) last_poll_magn = now;
    }

    if (use_poll_magn && magn_dev != accel_dev && magn_dev != gyro_dev && (now - last_poll_magn >= POLL_INTERVAL_MAGN_MS)) {
        read_device_data(magn_dev);
        last_poll_magn = now;
    }
}

void imu_get_latest_data(struct imu_data *out_data)
{
    if (out_data == NULL) return;

    k_mutex_lock(&imu_mutex, K_FOREVER);
    *out_data = current_data;
    k_mutex_unlock(&imu_mutex);
}

void imu_print_data(void)
{
    struct imu_data local_data;
    imu_get_latest_data(&local_data);

    LOG_INF("Accel (m/s^2): X=%f, Y=%f, Z=%f", 
            sensor_value_to_double(&local_data.accel[0]),
            sensor_value_to_double(&local_data.accel[1]),
            sensor_value_to_double(&local_data.accel[2]));

    LOG_INF("Gyro (rad/s):  X=%f, Y=%f, Z=%f", 
            sensor_value_to_double(&local_data.gyro[0]),
            sensor_value_to_double(&local_data.gyro[1]),
            sensor_value_to_double(&local_data.gyro[2]));

    LOG_INF("Magn (gauss):  X=%f, Y=%f, Z=%f", 
            sensor_value_to_double(&local_data.magn[0]),
            sensor_value_to_double(&local_data.magn[1]),
            sensor_value_to_double(&local_data.magn[2]));
}

void imu_analyze_frequency(void)
{
    bool accel_updated = freq_monitor_update(&accel_freq);
    bool gyro_updated  = freq_monitor_update(&gyro_freq);
    bool magn_updated  = freq_monitor_update(&magn_freq);

    if (accel_updated || gyro_updated || magn_updated) {
        LOG_INF("Accel: %u Hz | Gyro: %u Hz | Magn: %u Hz", 
                freq_monitor_get_hz(&accel_freq), 
                freq_monitor_get_hz(&gyro_freq),
                freq_monitor_get_hz(&magn_freq));
    }

}