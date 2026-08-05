/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * esp_stubs.h — test-side control surface for the ESP-IDF stubs
 * ============================================================================
 * The stub headers next to this one mimic the IDF API exactly so the real
 * firmware sources compile against them unmodified. Everything a TEST needs —
 * driving the clock, seeding the RNG, observing a reboot — lives here instead,
 * and is not visible to the code under test.
 * ============================================================================
 */
#ifndef ESP_STUBS_H
#define ESP_STUBS_H

#include <stdint.h>

/* Zero every recorded observation and reseed the RNG. Call at the top of any
 * test that inspects a stub. */
void stub_reset(void);

/* esp_timer_get_time() backing store (microseconds since "boot"). */
void    stub_time_set_us(int64_t us);
void    stub_time_advance_us(int64_t us);
int64_t stub_time_now_us(void);

/* esp_random() is a fixed-seed LCG. stub_random_peek() returns the value the
 * NEXT esp_random() call will produce, without consuming it. */
void     stub_random_seed(uint32_t seed);
uint32_t stub_random_peek(void);

/* esp_restart() observations: the stub records and returns. */
unsigned stub_restart_count(void);

/* vTaskDelay() observations: ticks are accumulated, never slept. */
uint32_t stub_delay_ticks_total(void);

#endif /* ESP_STUBS_H */
