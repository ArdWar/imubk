#ifndef UTIL_HPP
#define UTIL_HPP

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <stdio.h>

char *dump_bytes(char *buf, const uint8_t *p, uint8_t count);

#endif