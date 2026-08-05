# SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
# SPDX-License-Identifier: AGPL-3.0-or-later
# Component build for the firmware sources.
#
# esp_tinyusb is pulled via idf_component.yml (managed component). REQUIRES
# lists the IDF components each source needs:
#   - esp_tinyusb : TinyUSB device stack (usb_glue, usb_descriptors)
#   - driver      : legacy TWAI + GPIO (twai_hal, config)
#   - esp_timer   : monotonic clock (pcan_time)
#   - esp_psram   : PSRAM heap caps (ringbuf)
#   - nvs_flash   : mode persistence (main)
#   - freertos    : tasks/queues/semaphores (implicit, listed for clarity)

idf_component_register(
    SRCS
        "main.c"
        "usb_descriptors.c"
        "usb_glue.c"
        "pcan_cmd.c"
        "pcan_msg.c"
        "pcan_time.c"
        "twai_hal.c"
        "ringbuf.c"
        "isotp.c"
        "uds.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_tinyusb
        driver
        esp_timer
        esp_psram
        nvs_flash
        freertos
)

# The legacy TWAI driver (driver/twai.h) is still functional in IDF 6.0 but its
# header emits a #warning that -Werror=cpp promotes to an error. twai_hal.c uses
# it deliberately (migrating to the new esp_twai node API is a separate future
# task), so allow that one deprecation #warning for this component only.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=cpp)
