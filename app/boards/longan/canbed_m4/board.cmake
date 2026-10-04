# SPDX-License-Identifier: Apache-2.0

# `west flash` copies zephyr.uf2 to the SAME51G19A bootloader drive
# (double-tap RESET, or the 1200-baud touch handled by bossa.c when the
# application is running).  `west flash -r bossac` and `-r openocd` also work.
include(${ZEPHYR_BASE}/boards/common/uf2.board.cmake)
include(${ZEPHYR_BASE}/boards/common/bossac.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
