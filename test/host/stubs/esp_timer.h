/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * esp_timer.h — host stub
 * ============================================================================
 * Declares only the one symbol pcan_time.c uses, so the real source compiles
 * unmodified. The clock is driven from the test via stub_time_* (esp_stubs.h).
 * ============================================================================
 */
#ifndef STUB_ESP_TIMER_H
#define STUB_ESP_TIMER_H

#include <stdint.h>

int64_t esp_timer_get_time(void);

#endif /* STUB_ESP_TIMER_H */
