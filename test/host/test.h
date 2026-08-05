/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * test.h — the whole test harness
 * ============================================================================
 * Assert macros, a case runner and a summary printer. No framework, no
 * registration boilerplate: a case is a `static void fn(void)`, a suite is a
 * function that calls RUN() on each of them.
 *
 * A failing CHECK records the failure and lets the case continue, so one run
 * reports every broken expectation rather than only the first. REQUIRE is for
 * a precondition the rest of the case would read past — it returns from the
 * case instead.
 *
 * Exactly ONE translation unit must #define TEST_IMPL before including this.
 * ============================================================================
 */
#ifndef PCAN_TEST_H
#define PCAN_TEST_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern unsigned t_checks;      /* assertions evaluated, all cases             */
extern unsigned t_fails;       /* assertions failed, all cases                */
extern unsigned t_cases;       /* cases run                                   */
extern unsigned t_cases_bad;   /* cases with >=1 failed assertion             */

void t_failf(const char *file, int line, const char *fmt, ...);
void t_run(const char *name, void (*fn)(void));
void t_hex(const char *label, const void *p, size_t n);
int  t_report(void);

/* ---- assertions ---------------------------------------------------------- */

#define CHECK(cond)                                                           \
    do {                                                                      \
        ++t_checks;                                                           \
        if (!(cond)) {                                                        \
            t_failf(__FILE__, __LINE__, "CHECK(%s)", #cond);                  \
        }                                                                     \
    } while (0)

#define CHECK_MSG(cond, ...)                                                  \
    do {                                                                      \
        ++t_checks;                                                           \
        if (!(cond)) {                                                        \
            t_failf(__FILE__, __LINE__, __VA_ARGS__);                         \
        }                                                                     \
    } while (0)

/* Integer equality. Both sides are widened to long long, which is lossless for
 * every type the wire code uses (nothing here exceeds 32 bits). */
#define CHECK_EQ(got, want)                                                   \
    do {                                                                      \
        ++t_checks;                                                           \
        long long t_g_ = (long long)(got), t_w_ = (long long)(want);          \
        if (t_g_ != t_w_) {                                                   \
            t_failf(__FILE__, __LINE__, "%s: got %lld (0x%llx), want %lld (0x%llx)", \
                    #got, t_g_, (unsigned long long)t_g_,                     \
                    t_w_, (unsigned long long)t_w_);                          \
        }                                                                     \
    } while (0)

#define CHECK_MEM(got, want, n)                                               \
    do {                                                                      \
        ++t_checks;                                                           \
        if (memcmp((got), (want), (size_t)(n)) != 0) {                        \
            t_failf(__FILE__, __LINE__, "%s != %s (%u bytes)", #got, #want,   \
                    (unsigned)(n));                                           \
            t_hex("  got ", (got),  (size_t)(n));                             \
            t_hex("  want", (want), (size_t)(n));                             \
        }                                                                     \
    } while (0)

/* Precondition: on failure the case stops rather than reading past the end of
 * whatever it was about to inspect. */
#define REQUIRE(cond)                                                         \
    do {                                                                      \
        ++t_checks;                                                           \
        if (!(cond)) {                                                        \
            t_failf(__FILE__, __LINE__, "REQUIRE(%s)", #cond);                \
            return;                                                           \
        }                                                                     \
    } while (0)

#define REQUIRE_EQ(got, want)                                                 \
    do {                                                                      \
        ++t_checks;                                                           \
        long long t_g_ = (long long)(got), t_w_ = (long long)(want);          \
        if (t_g_ != t_w_) {                                                   \
            t_failf(__FILE__, __LINE__, "%s: got %lld (0x%llx), want %lld (0x%llx)", \
                    #got, t_g_, (unsigned long long)t_g_,                     \
                    t_w_, (unsigned long long)t_w_);                          \
            return;                                                           \
        }                                                                     \
    } while (0)

#define RUN(fn) t_run(#fn, fn)

/* ==========================================================================
 * Implementation (one TU only).
 * ========================================================================== */
#ifdef TEST_IMPL

unsigned t_checks;
unsigned t_fails;
unsigned t_cases;
unsigned t_cases_bad;

static unsigned t_case_fails;
static const char *t_case_name = "<none>";

void t_failf(const char *file, int line, const char *fmt, ...)
{
    va_list ap;
    ++t_fails;
    ++t_case_fails;
    /* Basename only: the absolute build path is noise in a failure report. */
    const char *base = strrchr(file, '/');
    fprintf(stderr, "    FAIL %s:%d [%s] ", base ? base + 1 : file, line,
            t_case_name);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

void t_hex(const char *label, const void *p, size_t n)
{
    const uint8_t *b = (const uint8_t *)p;
    fprintf(stderr, "      %s:", label);
    for (size_t i = 0; i < n; ++i) {
        fprintf(stderr, " %02X", b[i]);
    }
    fputc('\n', stderr);
}

void t_run(const char *name, void (*fn)(void))
{
    t_case_name  = name;
    t_case_fails = 0;
    ++t_cases;
    fn();
    if (t_case_fails != 0) {
        ++t_cases_bad;
        printf("  FAIL  %s (%u failed assertion%s)\n", name, t_case_fails,
               t_case_fails == 1 ? "" : "s");
    } else {
        printf("  ok    %s\n", name);
    }
    t_case_name = "<none>";
}

int t_report(void)
{
    printf("\n%u cases, %u assertions, %u failed assertion%s in %u case%s\n",
           t_cases, t_checks, t_fails, t_fails == 1 ? "" : "s",
           t_cases_bad, t_cases_bad == 1 ? "" : "s");
    if (t_cases_bad == 0) {
        printf("PASS\n");
        return 0;
    }
    printf("FAILED\n");
    return 1;
}

#endif /* TEST_IMPL */
#endif /* PCAN_TEST_H */
