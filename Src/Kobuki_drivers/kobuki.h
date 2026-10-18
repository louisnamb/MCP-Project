#ifndef KOBUKI_H
#define KOBUKI_H

#include <stdint.h>

// Kobuki structure to hold decoded sensor data
typedef struct {
    uint8_t cliff;
    uint8_t bumper;
    uint8_t wheeldrop;
    uint8_t button;
} Kobuki_Typedef;

// Feedback buffer size and sub-payload byte offsets
#define SIZE_FEEDBACK 14

#define BUMPER        4
#define WHEEL_DROP    5
#define CLIFF         6
#define BUTTON        13

// Speed limit (mm/s) to prevent runaway/damage
#define SPEED_LIMIT   (int16_t) 150

// Kobuki driver subroutines
uint8_t Kobuki_Rx(uint8_t *feedback, uint32_t size_feedback);
void Kobuki_Tx(uint8_t *payload, uint8_t size_payload);
void Kobuki_Read(Kobuki_Typedef *kobuki);
void Kobuki_Drive(int16_t speed);
void Kobuki_Rotate(int16_t speed);

#endif