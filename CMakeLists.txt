# SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
# SPDX-License-Identifier: AGPL-3.0-or-later
# Top-level ESP-IDF project file for the PCAN-USB ESP32-S3 emulator.
# Build:  idf.py set-target esp32s3 && idf.py build flash monitor
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(espcan32)
