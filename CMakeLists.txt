# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lapan_imubk)

target_sources(app PRIVATE
    src/main.cpp
    src/sen_mti3.cpp
    src/sen_adxl.cpp
    src/sen_lsm6.cpp
    src/sen_lis3.cpp
    src/sen_ms56.cpp
    src/sen_nmea.cpp
    src/com_uart.cpp
    src/util.cpp
    )
