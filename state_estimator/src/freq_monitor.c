#include "freq_monitor.h"
#include <zephyr/kernel.h>

void freq_monitor_init(struct freq_monitor *mon)
{
    atomic_set(&mon->event_count, 0);
    mon->last_calc_time = k_uptime_get();
    mon->last_hz = 0;
}

void freq_monitor_tick(struct freq_monitor *mon)
{
    atomic_inc(&mon->event_count);
}

bool freq_monitor_update(struct freq_monitor *mon)
{
    int64_t now = k_uptime_get();
    int64_t delta_ms = now - mon->last_calc_time;

    if (delta_ms >= 1000) {
        /* Atomically read and clear the counter to prevent race conditions */
        atomic_val_t count = atomic_set(&mon->event_count, 0);
        
        mon->last_hz = (uint32_t)((count * 1000) / delta_ms);
        mon->last_calc_time = now;
        return true;
    }
    return false;
}

uint32_t freq_monitor_get_hz(struct freq_monitor *mon)
{
    return mon->last_hz;
}