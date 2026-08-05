/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * freertos/task.h — host stub
 * ============================================================================
 * vTaskDelay() accumulates the requested ticks instead of blocking, so a test
 * can assert that a deferred action waited before it fired.
 * ============================================================================
 */
#ifndef STUB_FREERTOS_TASK_H
#define STUB_FREERTOS_TASK_H

#include "freertos/FreeRTOS.h"

void vTaskDelay(TickType_t ticks);

#endif /* STUB_FREERTOS_TASK_H */
