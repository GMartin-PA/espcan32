/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * test_main.c — host test entry point
 * ============================================================================
 * Owns the harness state (TEST_IMPL) and runs every suite. Exit status is 0
 * only if every assertion in every suite passed.
 * ============================================================================
 */
#define TEST_IMPL
#include "test.h"

void suite_pcan_msg(void);
void suite_isotp(void);
void suite_uds(void);

int main(void)
{
    suite_pcan_msg();
    suite_isotp();
    suite_uds();
    return t_report();
}
