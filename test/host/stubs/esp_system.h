/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * esp_system.h — host stub
 * ============================================================================
 * esp_restart() is recorded rather than performed: the real one is noreturn,
 * but a test that lost the process could not assert on what happened next.
 * Deliberately NOT declared noreturn — the stub returns to its caller.
 * ============================================================================
 */
#ifndef STUB_ESP_SYSTEM_H
#define STUB_ESP_SYSTEM_H

void esp_restart(void);

#endif /* STUB_ESP_SYSTEM_H */
