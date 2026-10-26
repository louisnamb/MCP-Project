#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>

typedef struct {
    uint16_t ultrasonic_distance;   /* mm */
    uint8_t  object_detected;
    uint8_t  cliff_detected;
    uint8_t  bumper_detected;
    uint8_t wheeldrop;
} Navigation_t;

void Navigation_init(void);
void Navigation_update(const Navigation_t *Nav, uint32_t time_ms);

#endif