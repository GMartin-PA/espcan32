#!/bin/sh
# SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
# SPDX-License-Identifier: AGPL-3.0-or-later
# Print the strongest -fsanitize flag set this toolchain can both BUILD and RUN.
#
# Linking is not a sufficient test: on some hosts the AddressSanitizer runtime
# deadlocks inside its own initialiser (before main), which would hang the test
# suite rather than sanitise it. So each candidate is compiled AND executed
# under a watchdog, and the first one that exits cleanly wins.
#
# Output is a (possibly empty) flag string on stdout. CC is honoured.
set -u

CC=${CC:-cc}
PROBE_TIMEOUT=${PROBE_TIMEOUT:-10}

tmp=$(mktemp -d 2>/dev/null) || exit 0
trap 'rm -rf "$tmp"' EXIT INT TERM
printf 'int main(void){return 0;}\n' > "$tmp/p.c"

try() {
    $CC $1 "$tmp/p.c" -o "$tmp/p" >/dev/null 2>&1 || return 1
    "$tmp/p" >/dev/null 2>&1 &
    probe=$!
    (sleep "$PROBE_TIMEOUT"; kill -9 "$probe") >/dev/null 2>&1 &
    watchdog=$!
    wait "$probe" 2>/dev/null
    rc=$?
    kill "$watchdog" >/dev/null 2>&1
    [ "$rc" -eq 0 ]
}

for flags in "-fsanitize=address,undefined" "-fsanitize=undefined" ""; do
    if try "$flags"; then
        printf '%s\n' "$flags"
        exit 0
    fi
done

printf '\n'
