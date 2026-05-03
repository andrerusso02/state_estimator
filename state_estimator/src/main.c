#include <zephyr/kernel.h>
#include "imu_sensor.h"

int main(void)
{
    if (imu_init() < 0) {
        return 0;
    }

    while (1) {
        /* Updates any sensors functioning in poll mode. 
           Sensors using interrupts are updated automatically in the background. */
        imu_poll_update();

        imu_print_data();

        k_sleep(K_MSEC(10));
    }
}