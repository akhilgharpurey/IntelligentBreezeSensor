#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)
# set(BOARD nrf52832_custom)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(TMP117_fan_trigger)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
)
# NORDIC SDK APP END
