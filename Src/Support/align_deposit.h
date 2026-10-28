#ifndef ALIGN_DEPOSIT_H
#define ALIGN_DEPOSIT_H

#include <stdint.h>
#include <stdbool.h>
#include "kobuki.h"
#include "ultrasonic.h"

/* Detection thresholds (in mm) */
#define D_THRESH       800   // Detection distance threshold 
#define D_STOP         100   // Stop distance for deposit
#define ROTATE_SPEED    60    // Speed (mm/s or deg/s) for manual sweeps
#define APPROACH_SPEED 80    // Speed (mm/s) when advancing toward deposit

typedef enum {
    ALIGN_IDLE = 0,
    ALIGN_SCAN_TARGET,
    ALIGN_ROTATE_CCW,
    ALIGN_ROTATE_CW,
    ALIGN_SET_CENTER,
    ALIGN_ROT_COMPLETE,
    ALIGN_SET_DRIVE_SETPOINT, 
    ALIGN_MOVE_FORWARD,
    ALIGN_ARRIVED,
    ALIGN_ERROR
} Align_State_t;

/* Main FSM function */
bool Run_Align_And_Deposit(Kobuki_Typedef *kobuki, US_Typedef *object);

/* Call to reset the sub-FSM before initializing align & deposit */
void Reset_Align_And_Deposit(void);

#endif /* ALIGN_DEPOSIT_H */
