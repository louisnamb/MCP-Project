#include <stdint.h>
//#define STM32G474xx
#include "stm32g4xx.h"
#include "kobuki.h"
#include "uart.h"
#include "ultrasonic.h"
#include "align_deposit.h"

#define DEG_PER_TICK (1.0/100.0)

static bool object_reacquired = false;
static Align_State_t align_state = ALIGN_IDLE;
static int16_t theta_left = 0;
static int16_t theta_right = 0;

/* Convert raw Kobuki gyro ticks to [0, 359] degrees */
static int16_t Ticks_To_Degrees(int16_t ticks) {
    int32_t deg = (int32_t)((double)(ticks + 18000) * DEG_PER_TICK);
    deg = deg % 360;
    if (deg < 0) deg += 360;
    return (int16_t)deg;
}

/* Calculate the shortest signed angular offset (-180 to +180 deg) */
static int16_t Calc_Shortest_Rotation(int16_t from_deg, int16_t theta_target) {
    /* Normalize angular difference strictly into [0, 359] */
    int16_t diff = ((theta_target - from_deg) % 360 + 360) % 360;
    
    /* Map (180, 359] down to (-180, -1] for optimal turn direction */
    if (diff > 180) {
        diff -= 360;
    }
    return diff;
}

void Reset_Align_And_Deposit(void) {
    align_state = ALIGN_SCAN_TARGET;
    theta_left = 0;
    theta_right = 0;
    object_reacquired = false; // Reset reacquisition flag 
    Kobuki_Drive(0);
}

bool Run_Align_And_Deposit(Kobuki_Typedef *kobuki, US_Typedef *object) {
    /* Safety check: Stop immediately on wheel drop, cliff, or bumper hit */
    if (kobuki->wheeldrop || (kobuki->bumper + kobuki->cliff) != 0) {
        Kobuki_Drive(0);
        align_state = ALIGN_ERROR;
        return false;
    }

    uint16_t current_dist = object->range_mm;
    int16_t current_deg = Ticks_To_Degrees(kobuki->angle);

    switch (align_state) {
        case ALIGN_IDLE:
            Kobuki_Drive(0);
            return false;

        /* Step 1: Scan if an obstacle is in range */
        case ALIGN_SCAN_TARGET:
            if (current_dist <= D_THRESH && current_dist > 0) {
                /* Target confirmed: start rotating CCW (positive angular velocity) */
                Kobuki_Rotate(ROTATE_SPEED);
                align_state = ALIGN_ROTATE_CCW;
            } else {
                /* No target detected in range -> Exit */
                Kobuki_Drive(0);
                align_state = ALIGN_IDLE;
                return false;
            }
            break;

        /* Step 2: Rotate CCW until the beam clears the left edge of the object */
        case ALIGN_ROTATE_CCW:
            if (current_dist > D_THRESH || current_dist == 0){
                Kobuki_Rotate(0);
                theta_left = current_deg; // Record theta_left
               /* Reset flag and start rotating CW */
               object_reacquired = false;
               Kobuki_Rotate(-ROTATE_SPEED);
               align_state = ALIGN_ROTATE_CW;
            } else {
               Kobuki_Rotate(ROTATE_SPEED); // Positive = CCW
            }
            break;

       /* Step 3: Rotate CW across the object until it re-enters and then clears the right edge */
       case ALIGN_ROTATE_CW:
            /* Wait until the sensor beam re-enters the object */
            if (object_reacquired == false) {
            if (current_dist <= D_THRESH && current_dist > 0) {
               object_reacquired =true; // Object back in view, keep sweeping CW
            }
               Kobuki_Rotate(-ROTATE_SPEED);
            } 
            /* wait until we drop off the right edge */
            else {
            if (current_dist > D_THRESH || current_dist == 0) {
             Kobuki_Rotate(0);
             theta_right = current_deg; // Record true theta_right
             align_state = ALIGN_SET_CENTER;
            } else {
            Kobuki_Rotate(-ROTATE_SPEED);
            }
            }
            break;

       /* Step 4: Compute centerline */
       case ALIGN_SET_CENTER: {
           /* Calculate the swept arc width (for 0deg/360deg rollover) */
           int16_t span = (theta_left - theta_right + 360) % 360;

           /* Target heading is halfway across the swept arc */
           int16_t theta_target = (theta_right + (span / 2)) % 360;

           /* Get current heading in deg */
           int16_t init_angle = Ticks_To_Degrees(kobuki->angle);

           /* Find the shortest turn in degrees */
           int16_t rot_angle_deg = Calc_Shortest_Rotation(init_angle, theta_target);

           /*Set the rotation setpoint using raw ticks and most suitable turn in degrees */
           Kobuki_Rotate_Angle_Setpoint(kobuki->angle, rot_angle_deg);
           align_state = ALIGN_ROT_COMPLETE;
        break;
        }

        /* Step 5: Execute turn to centerline */
        case ALIGN_ROT_COMPLETE:
            if (kobuki->rotation_complete) {
                Kobuki_Drive(STOP);
                align_state = ALIGN_SET_DRIVE_SETPOINT;
            } else {
                Kobuki_Rotate_Angle(kobuki->angle);
            }
            break;

        /* Step 6A: Set drive_distance_setpoint */
        case ALIGN_SET_DRIVE_SETPOINT: {
            // Target approach distance = current distance minus desired stopping distance
            int16_t drive_distance_mm = (int16_t)current_dist - D_STOP;

            //  Safety check: if too close or ultrasonic echo failed,force completion
            if (drive_distance_mm <= 0 || current_dist == 0) {
                Kobuki_Drive(STOP);
                align_state = ALIGN_ARRIVED;
            } else {
                // Arm the P-controller distance setpoint using current wheel ticks
                Kobuki_Drive_Distance_Setpoint(kobuki->distance, drive_distance_mm); 
                align_state = ALIGN_MOVE_FORWARD;
            }
            break;
        }

        /* Step 6B: Execute closed-loop forward drive until destination attained */
        case ALIGN_MOVE_FORWARD:
            // stop if ultrasonic sensor detects bollard closer than expected
            if (current_dist <= D_STOP && current_dist > 0) {
                Kobuki_Drive(STOP);
                align_state = ALIGN_ARRIVED;
            }
            // Safe arrival without triggers
            else if (kobuki->distance_complete) {
                Kobuki_Drive(STOP);
                align_state = ALIGN_ARRIVED;
            } 
            // Keep running the loop
            else {
                Kobuki_Drive_Distance(kobuki->distance); //
            }
            break;

        case ALIGN_ARRIVED:
            Kobuki_Drive(STOP);
            return true; // Successful completion

        case ALIGN_ERROR:
        default:
            Kobuki_Drive(STOP);
            return false;
    }
    return false; // Still running
}
