#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "imu_sensor.h"
#include "udp_stream.h"

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

/* Network Configuration */
#define DEST_ADDR "255.255.255.255"
#define SERVER_PORT 5005

#define RATE_HZ 100
#define PERIOD_K_TICKS K_MSEC(1000 / RATE_HZ)

int main(void)
{
    int ret;

    /* 1. Initialize Network Subsystem */
    ret = udp_stream_init(DEST_ADDR, SERVER_PORT);
    if (ret < 0) {
        LOG_ERR("UDP initialization failed (%d). Halting.", ret);
        return -1;
    }

    /* 2. Initialize IMU Subsystem */
    ret = imu_init();
    if (ret < 0) {
        LOG_ERR("IMU initialization failed (%d). Halting.", ret);
        return -1;
    }

    LOG_INF("IMU UDP Streaming started to %s:%d", DEST_ADDR, SERVER_PORT);

    struct imu_data stream_payload;

	struct k_timer loop_timer;
    k_timer_init(&loop_timer, NULL, NULL);
	k_timer_start(&loop_timer, PERIOD_K_TICKS, PERIOD_K_TICKS);

    while (1) {
        imu_poll_update();
        imu_get_latest_data(&stream_payload);

        ret = udp_stream_send(&stream_payload, sizeof(stream_payload));
        if (ret < 0) {
            LOG_DBG("Dropped UDP packet: %d", ret); /* Using DBG to prevent console spam */
        }

        imu_analyze_frequency();

        uint32_t overruns = k_timer_status_sync(&loop_timer);
        if (overruns > 1) {
            LOG_WRN("Loop rate overrun! Missed %u cycles", overruns - 1);
        }
    }

    return 0;
}