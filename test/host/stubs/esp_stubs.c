/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * esp_stubs.c — host implementations of the ESP-IDF symbols the modules use
 * ============================================================================
 * pcan_time.c needs esp_timer_get_time(); uds.c needs esp_random(),
 * esp_restart() and vTaskDelay(). None of them do anything platform-specific
 * here: the clock is whatever the test set, the RNG is a fixed-seed LCG so a
 * SecurityAccess seed (and the key derived from it) is identical on every run,
 * and a reboot is counted rather than taken so the test survives it.
 * ============================================================================
 */
#include "esp_stubs.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Arbitrary but fixed: any value works as long as it never changes. */
#define STUB_RANDOM_SEED_DEFAULT 0x0C72000Cu

static int64_t  s_now_us;
static uint32_t s_rng;
static unsigned s_restarts;
static uint32_t s_delay_ticks;

void stub_reset(void)
{
    s_now_us      = 0;
    s_rng         = STUB_RANDOM_SEED_DEFAULT;
    s_restarts    = 0;
    s_delay_ticks = 0;
}

void stub_time_set_us(int64_t us)      { s_now_us = us; }
void stub_time_advance_us(int64_t us)  { s_now_us += us; }
int64_t stub_time_now_us(void)         { return s_now_us; }

void stub_random_seed(uint32_t seed)   { s_rng = seed; }

/* Numerical Recipes LCG — not cryptographic, but exactly reproducible, which
 * is the only property a test needs from it. */
static uint32_t stub_rng_next(uint32_t s)
{
    return (uint32_t)(s * 1664525u + 1013904223u);
}

uint32_t stub_random_peek(void)        { return stub_rng_next(s_rng); }

unsigned stub_restart_count(void)      { return s_restarts; }
uint32_t stub_delay_ticks_total(void)  { return s_delay_ticks; }

/* ---- the stubbed IDF symbols --------------------------------------------- */

int64_t esp_timer_get_time(void)
{
    return s_now_us;
}

uint32_t esp_random(void)
{
    s_rng = stub_rng_next(s_rng);
    return s_rng;
}

void esp_restart(void)
{
    ++s_restarts;
}

void vTaskDelay(TickType_t ticks)
{
    s_delay_ticks += ticks;
}
