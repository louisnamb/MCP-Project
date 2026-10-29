#ifndef KOBUKI_H
#define KOBUKI_H

#include <stdint.h>
#include <stdlib.h>

// Define buffer for feedback
#define SIZE_FEEDBACK 14

// Buffer definitions for payload
#define BUMPER 4
#define WHEEL_DROP 5
#define CLIFF 6
#define BUTTON 13
#define PI (3.14159)
#define WHEEL_RADIUS (35.0) // mm
#define REVOLUTIONS_PER_TICK (132.0/6545.0) /52.0
#define MM_PER_TICK (2.0 * PI * WHEEL_RADIUS * REVOLUTIONS_PER_TICK )
#define DEG_PER_TICK (1.0/100.0)
#define GAIN_ROTATE (1.0/0.90)
#define GAIN_DRIVE (100.0/100.0)
#define sign(x)((int8_t)((x) >= 0 ? 1 : -1))
#define DIST_LSB 7 // only needs 1 wheel
#define DIST_MSB 8
#define BUTTON 13
#define ANGLE_LSB 24 // only needs 1 wheel
#define ANGLE_MSB 25
// Truncated speed
#define SPEED_LIMIT (int16_t)150 // mm / s
#define SEARCH_SPEED ((int16_t)75) // mm / s
#define STOP ((int16_t)0)// mm / s
#define MIN_SPEED ((int16_t)50) // mm / s

// Kobuki structure to hold decoded sensor data
typedef struct {
    uint8_t cliff;
    uint8_t bumper;
    uint8_t wheeldrop;
    uint8_t button;
    int16_t angle;
    uint8_t rotation_complete;
    uint8_t distance_complete;
    uint32_t distance;
} Kobuki_Typedef;

// Kobuki driver subroutines
uint8_t Kobuki_Rx(uint8_t *feedback, uint32_t size_feedback);
void Kobuki_Tx(uint8_t *payload, uint8_t size_payload);
void Kobuki_Read(Kobuki_Typedef *kobuki);
void Kobuki_Drive(int16_t speed);
void Kobuki_Rotate(int16_t speed);
void Kobuki_Rotate_Angle(int16_t current_angle_tick);
void Kobuki_Rotate_Angle_Setpoint(int16_t current_angle_tick, int16_t rot_angle_deg);
void Kobuki_Drive_Distance_Setpoint(uint16_t current_distance_tick, int16_t distance);
void Kobuki_Drive_Distance(uint16_t current_distance_tick);
#endif
