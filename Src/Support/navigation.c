#include "Navigation.h"
#include "kobuki.h"

#define DRIVE_SPEED_NAV   100
#define ROTATE_SPEED_NAV  60
#define REVERSE_SPEED_NAV 100

#define REVERSE_TIME_MS   1000U   /* back up this long after a cliff */
#define MIN_ROTATE_MS     800U    /* always turn at least this long  */

typedef enum {
    NAV_DRIVE,
    NAV_REVERSE,
    NAV_ROTATE
} NavState_t;

static NavState_t nav_state = NAV_DRIVE;
static uint32_t action_start_ms = 0U;

void Navigation_init(void)
{
    nav_state = NAV_DRIVE;
    action_start_ms = 0U;
    Kobuki_Drive(0);
}

void Navigation_update(const Navigation_t *Nav, uint32_t time_ms)
{
    if (Nav == 0) {
        Kobuki_Drive(0);
        return;
    }

    uint32_t elapsed = time_ms - action_start_ms;

    switch (nav_state)
    {
        case NAV_DRIVE:
            if (Nav->cliff_detected || Nav->bumper_detected) {
                nav_state = NAV_REVERSE;
                action_start_ms = time_ms;
                Kobuki_Drive(0);
            }
            else if (Nav->object_detected) {
                nav_state = NAV_ROTATE;
                action_start_ms = time_ms;
                Kobuki_Drive(0);
            }
            else {
                Kobuki_Drive(DRIVE_SPEED_NAV);
            }
            break;

        case NAV_REVERSE:
            Kobuki_Drive(-REVERSE_SPEED_NAV);
            if (elapsed >= REVERSE_TIME_MS) {
                nav_state = NAV_ROTATE;
                action_start_ms = time_ms;
            }
            break;

        case NAV_ROTATE:
            if ((Nav->cliff_detected) ||(Nav->bumper_detected)){
                nav_state = NAV_REVERSE;
                action_start_ms = time_ms;
            }
            else {
                Kobuki_Rotate(ROTATE_SPEED_NAV);
                /* keep turning until the way ahead is clear */
                if ((elapsed >= MIN_ROTATE_MS) && (Nav->object_detected == 0U)) {
                    nav_state = NAV_DRIVE;
                    action_start_ms = time_ms;
                }
            }
            break;

        default:
            nav_state = NAV_DRIVE;
            break;
    }
}