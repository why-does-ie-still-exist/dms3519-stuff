# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

# The CMSIS device header for the ATSAME51G19A is not shipped by hal_atmel;
# soc.h (patched, see zephyr/patches/) includes <same51g19a.h> from here.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(canbed_m4_can_loopback)

zephyr_include_directories(include)

target_sources(app PRIVATE
  src/main.c
  src/dm/dm_motor.c
  src/dm/dm_motor_pack.c
  src/dm/dm_shell.c
)
target_include_directories(app PRIVATE src/dm)
