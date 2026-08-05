/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * esp_random.h — host stub
 * ============================================================================
 * Backed by a fixed-seed LCG (esp_stubs.c) so SecurityAccess seeds — and hence
 * every key the tests compute from them — are reproducible run to run.
 * ============================================================================
 */
#ifndef STUB_ESP_RANDOM_H
#define STUB_ESP_RANDOM_H

#include <stdint.h>

uint32_t esp_random(void);

#endif /* STUB_ESP_RANDOM_H */
