/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * freertos/FreeRTOS.h — host stub
 * ============================================================================
 * Only the tick type and the ms->tick conversion uds.c needs. 1000 Hz matches
 * the default CONFIG_FREERTOS_HZ the firmware builds against, so the tick
 * counts the tests observe are the ones the device would delay for.
 * ============================================================================
 */
#ifndef STUB_FREERTOS_H
#define STUB_FREERTOS_H

#include <stdint.h>

typedef uint32_t TickType_t;

#define configTICK_RATE_HZ   1000u
#define portTICK_PERIOD_MS   (1000u / configTICK_RATE_HZ)
#define pdMS_TO_TICKS(ms)    ((TickType_t)(((uint64_t)(ms) * configTICK_RATE_HZ) / 1000u))

#endif /* STUB_FREERTOS_H */
