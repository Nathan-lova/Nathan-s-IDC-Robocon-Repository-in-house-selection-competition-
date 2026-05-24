#ifndef __LINE_FOLLOW_H
#define __LINE_FOLLOW_H

#include "stdint.h"

#define LF_SPEED_RPM   2000.0f
#define LF_DURATION_MS 15000

void    lf_start(void);
void    lf_stop(void);
uint8_t lf_is_active(void);
float   lf_update(uint8_t sensor_bits);

#endif
