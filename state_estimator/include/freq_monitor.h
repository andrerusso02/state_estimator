#ifndef FREQ_MONITOR_H
#define FREQ_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/atomic.h>

struct freq_monitor {
    atomic_t event_count;
    int64_t last_calc_time;
    uint32_t last_hz;
};

/**
 * Initializes the frequency monitor.
 */
void freq_monitor_init(struct freq_monitor *mon);

/**
 * Increments the event counter. 
 * Safe to call from any context (Thread or ISR).
 */
void freq_monitor_tick(struct freq_monitor *mon);

/**
 * Calculates the current frequency if at least 1000ms have passed.
 * @return true if the frequency was updated, false otherwise.
 */
bool freq_monitor_update(struct freq_monitor *mon);

/**
 * Returns the last calculated frequency in Hz.
 */
uint32_t freq_monitor_get_hz(struct freq_monitor *mon);

#endif /* FREQ_MONITOR_H */