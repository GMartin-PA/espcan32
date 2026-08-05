#!/bin/sh
# SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
# SPDX-License-Identifier: AGPL-3.0-or-later
# Build and run the host test suite. Exits non-zero if anything fails.
#
#   ./run_tests.sh            build with sanitizers (where supported) and run
#   SAN=0 ./run_tests.sh      build without them
#
# Requires only a C99 compiler and make. No ESP-IDF.
set -eu
cd "$(dirname "$0")"
exec make -s test
