#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

void insertion_sort(uint16_t arr[], int32_t count);
const char *get_field(char *line, int32_t num);
uint32_t crc32(const void *buf, uint32_t size);

#endif /* COMMON_UTILS_H */
