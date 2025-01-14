#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <canard.h>
#include <math.h>
#include <string.h>

void uintToStringInBuffer(uint8_t value, char *buffer, size_t buffer_size);
void uint16ToStringInBuffer(uint16_t value, char *buffer, size_t buffer_size);
void floatToStringInBuffer(float value, char *buffer, size_t buffer_size, int decimal_places);