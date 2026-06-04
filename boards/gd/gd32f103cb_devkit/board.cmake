# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0
#
# J-Link: device name must match your Segger J-Link software (JFlashDevices.xml).
# If "GD32F103CB" is not listed, try "GD32F103C8" or update J-Link / use --device=... on the command line.

board_runner_args(jlink "--device=GD32F103CB" "--iface=SWD" "--speed=4000")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
