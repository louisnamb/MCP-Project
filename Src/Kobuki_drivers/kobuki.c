#include <stdint.h>
//#define STM32G474xx
#include "stm32g4xx.h"
#include "kobuki.h"
#include "uart.h"
#include "ultrasonic.h"
#include "align_deposit.h"

uint32_t kobuki_good_packets = 0, kobuki_bad_packets = 0;
uint8_t  kobuki_rx_buffer[SIZE_FEEDBACK];
// static variables and flags , keep these here in kobuki . c
static volatile int32_t setpoint_distance_tick ;
static volatile int16_t setpoint_angle_deg ;
static uint8_t rotation_complete = 0;
static uint8_t distance_complete = 0;

void Kobuki_Rotate_Angle(int16_t current_angle_tick) {
    volatile int16_t speed ;
    volatile int16_t error ;
    volatile int32_t current_angle_deg ;
    current_angle_deg = (int32_t)((double)(current_angle_tick+(int32_t)18000) * DEG_PER_TICK);
    error = (int16_t) ( setpoint_angle_deg - current_angle_deg);
    // Check for non - optimal rotation angle
    if (abs(error) > 180) {
        // Always rotate with the minimum angle e . g . +270 -> -90.
        // Introduces singularity - loses sign and truncates error .
        error = - sign (error) *(abs(error) % 180);
        // Special cases in circular buffer . Offset to correct error .
        if ((error < 0) && (current_angle_deg < setpoint_angle_deg)) {
            error = -(error + 180);
        }
        else if ((error > 0) && (current_angle_deg > setpoint_angle_deg)) {
            error = -(error - 180);
        // Does this make sense ? Is there a simpler way ?
        }
    }
    if (abs(error) <= 0) {
        rotation_complete = 1;
    }
    else {
        speed = (sign(error) * MIN_SPEED) + (int16_t) (GAIN_ROTATE * error);
        Kobuki_Rotate(speed);
    }
}

void Kobuki_Rotate_Angle_Setpoint(int16_t current_angle_tick, int16_t rot_angle_deg) {
    volatile int32_t init_angle ;
    rotation_complete = 0;
    // initial angle [ -18000 ,18000] ->[0 -360] deg
    init_angle = (int32_t)((double)(current_angle_tick + (int32_t)18000) * DEG_PER_TICK);
    // setpoint angle
    setpoint_angle_deg = (init_angle + rot_angle_deg + 360) % 360; // [0 -360]
}

// Kobuki read datastream
void Kobuki_Read(Kobuki_Typedef *kobuki) {
    // Init buffer & checksum
    uint8_t kobuki_rx_buffer[SIZE_FEEDBACK];
    uint8_t checksum_result;
    
    // Read kobuki Tx data stream
    checksum_result = Kobuki_Rx(kobuki_rx_buffer, SIZE_FEEDBACK);
    
    // If checksum 'checks out', update sensor variables from buffer
    if (checksum_result == 0) {
        kobuki_good_packets++;
        kobuki->bumper    = kobuki_rx_buffer[BUMPER];
        kobuki->wheeldrop = kobuki_rx_buffer[WHEEL_DROP];
        kobuki->cliff     = kobuki_rx_buffer[CLIFF];
        kobuki->button    = kobuki_rx_buffer[BUTTON];
    } else {
        kobuki_bad_packets++;
    }
}

void Kobuki_Tx(uint8_t *payload, uint8_t size_payload) {
    uint8_t checksum = 0, i;
    UART_Tx(0xAA); // Send header 0
    UART_Tx(0x55); // Send header 1
    UART_Tx(size_payload); // Send size of payload in bytes
    checksum ^= size_payload; // Calculate running checksum
    
    // Send each byte of payload one-by-one
    for (i = 0; i < size_payload; i++) {
        UART_Tx(payload[i]);
        checksum ^= payload[i];
    }
    UART_Tx(checksum); // Send checksum
}

uint8_t Kobuki_Rx(uint8_t *feedback, uint32_t size_feedback) {
    uint8_t feedback_header = 0;
    uint8_t size_payload = 0;
    uint8_t checksum = 0;
    uint8_t rx_checksum = 0;
    uint32_t i = 0;

    while (1) {
        feedback_header = UART_Rx();
        if (feedback_header == 0xAA) {
            feedback_header = UART_Rx();
            if (feedback_header == 0x55) {
                break;
            }
        }
    }

    size_payload = UART_Rx();
    checksum = size_payload;

    uint32_t limit = (size_payload < size_feedback) ? size_payload : size_feedback;

    for (i = 0; i < limit; i++) {
        feedback[i] = UART_Rx();
        checksum ^= feedback[i];
    }

    for (i = limit; i < size_payload; i++) {
        checksum ^= UART_Rx();
    }

    rx_checksum = UART_Rx();
    return (checksum != rx_checksum);
}

void Kobuki_Drive(int16_t speed) {
    uint8_t speed_lsb = 0;
    uint8_t speed_msb = 0;
    
    // Construct payload
    uint8_t payload[] = {0x01, 0x04, speed_lsb, speed_msb, 0x00, 0x00};
    
    // Saturation to avoid going over speed limit
    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    } else if (speed < -SPEED_LIMIT) {
        speed = (~SPEED_LIMIT) + 1;
    }
    
    // Split LSB & MSB for payload
    payload[2] = (uint8_t)speed;
    payload[3] = (uint8_t)(speed >> 0x08);
    Kobuki_Tx(payload, sizeof(payload));
}

void Kobuki_Rotate(int16_t speed) {
    uint8_t speed_lsb = 0;
    uint8_t speed_msb = 0;
    
    // Construct payload
    uint8_t payload[] = {0x01, 0x04, speed_lsb, speed_msb, 0x01, 0x00};
    
    // Saturation to avoid going over speed limit
    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    } else if (speed < -SPEED_LIMIT) {
        speed = (~SPEED_LIMIT) + 1;
    }
    
    // Split LSB & MSB for payload
    payload[2] = (uint8_t)speed;
    payload[3] = (uint8_t)(speed >> 0x08);
    Kobuki_Tx(payload, sizeof(payload));
}

void Kobuki_Drive_Distance_Setpoint(uint16_t current_distance_tick, int16_t distance) {
    distance_complete = 0;
    setpoint_distance_tick = current_distance_tick + (int32_t) (distance / MM_PER_TICK);
}

void Kobuki_Drive_Distance(uint16_t current_distance_tick) {
    volatile int16_t speed;
    volatile int16_t error;
    volatile uint16_t setpoint_alias;
    // standard conditoon
    error = (int16_t)((double)(setpoint_distance_tick - current_distance_tick ) * MM_PER_TICK);
    setpoint_alias = ( uint16_t ) ( ( setpoint_distance_tick + 0x10000) % 0x10000);
    if (((setpoint_distance_tick > 0xFFFF) && (current_distance_tick < setpoint_alias)) || ((setpoint_distance_tick < 0) && (current_distance_tick > setpoint_alias)))
        error = (int16_t)((double)(setpoint_alias - current_distance_tick) * MM_PER_TICK);
    if (abs(error) <= 2)
        distance_complete = 1;
    else {
        speed = (sign(error) * MIN_SPEED) + (int16_t) (GAIN_DRIVE * error);
        Kobuki_Drive(speed);
    }
}
