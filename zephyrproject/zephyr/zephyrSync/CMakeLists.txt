# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyrSync)

target_sources(app PRIVATE src/main.c)

# Code and Data Relocation 
# zephyr_code_relocate(FILES src/task1.h LOCATION SRAM2)
