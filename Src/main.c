#include "Navigation.h"
#include <stdint.h>
#define STM32G474xx
#include "stm32g4xx.h"
#include "kobuki.h"
#include "uart.h"
#include "adc.h"
#include "ultrasonic.h"
#include "navigation.h"

#define BUTTON_0_START   0x01U   /* start autonomous navigation */
#define BUTTON_1_STOP    0x02U
#define BUTTON_2_ROTATE  0x04U   /* start rotating scan */
#define ROTATE_SPEED_MM_S ((int16_t) 100)
#define MAX_SAMPLES      100U
#define TURN_TIMEOUT_MS   10000U


Kobuki_Typedef my_kobuki = {0};
volatile uint32_t loop_count = 0;     /* above main() */

typedef enum {
    MODE_IDLE = 0, 
    MODE_NAVIGATE = 1, 
    MODE_SCAN = 2,
    MODE_ALIGN_DEPOSIT=3,
    MODE_STOP_FAULT=4
} Mode_t;

typedef struct {
    uint16_t arc_positions[MAX_SAMPLES];   /* ultrasonic distance at each sample        */
    int16_t  arc_angles[MAX_SAMPLES];      /* gyro heading when each sample was taken   */
    uint16_t index_pos;                    /* array index                               */
    uint16_t curr_min_value;               /* smallest distance so far                  */
    uint16_t curr_min_index;               /* where that smallest distance is           */
} Master_t;

/* Globals so they can be watched in the debugger */
Kobuki_Typedef kobuki = {0};          /* also used by align_and_deposit.c and kobuki.c */
Master_t rover = {0};
volatile Mode_t   mode = MODE_IDLE;

static void Send_Kobuki_Move( int16_t speed , int16_t radius )
{
    uint8_t payload [6];
    payload [0] = 0x01U ;
    payload [1] = 0x04U ;
    payload [2] = ( uint8_t ) ( speed &0xFF ) ;
    payload [3] = ( uint8_t ) (( uint16_t ) speed >> 8) ;
    payload [4] = ( uint8_t ) ( radius &0xFF ) ;
    payload [5] = ( uint8_t ) (( uint16_t ) radius >> 8) ;
    Kobuki_Tx( payload , sizeof(payload ) ) ;
}

/* Take one sample. Returns 1 when the array is full (one revolution done). */
static uint8_t scan_state(Master_t *r)
{
    uint16_t d = Ultrasonic_Read();
    r->arc_positions[r->index_pos] = d;
    r->arc_angles[r->index_pos]    = kobuki.angle;     /* remember which way we were facing */
    if ((d != 0U) && (d < r->curr_min_value)) {         /* ignore 0 = no echo */
        r->curr_min_value = d;
        r->curr_min_index = r->index_pos;
    }
    r->index_pos++;
    return (r->index_pos >= MAX_SAMPLES);
}

/* TIM17 polled every 100 ms: used for the scan sampling and as the time base */
static void SampleTimer_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    TIM17->PSC = 15;        /* 16 MHz / 16 = 1 MHz. Change if your timer clock differs */
    TIM17->ARR = 99999;     /* 100 ms */
    TIM17->EGR = TIM_EGR_UG;
    TIM17->SR  = 0;
    TIM17->CR1 |= TIM_CR1_CEN;
}

int main(void) {
    US_Typedef ultrasonic_object = {0};
    Navigation_t navigation = {0};
    Mode_t mode = MODE_IDLE;
    uint32_t time_ms = 0U;
    uint8_t sample_due;
    uint32_t turn_start_ms = 0U;

    UART_Init();
    ADC_Init();
    Ultrasonic_Init();
    SampleTimer_Init();
    Send_Kobuki_Move(0 , 0);
/* once, after the other init calls */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER &= ~GPIO_MODER_MODE5;
    GPIOA->MODER |= GPIO_MODER_MODE5_0;          /* output */
    while (1)
    {
        Kobuki_Read(&my_kobuki);
        if (my_kobuki.bumper != 0U) GPIOA->BSRR = GPIO_BSRR_BS5;
        else                        GPIOA->BSRR = GPIO_BSRR_BR5;
        Ultrasonic_Detect(&ultrasonic_object);

        /* 100 ms tick */
        sample_due = 0U;
        if (TIM17->SR & TIM_SR_UIF) {
            TIM17->SR &= ~TIM_SR_UIF;
            time_ms += 100U;
            sample_due = 1U;
        }

        /* ---- Mode selection ---- */
        if (((my_kobuki.button & BUTTON_0_START) != 0U) && (mode == MODE_IDLE)) {
            Navigation_init();
            mode = MODE_NAVIGATE;
        }
        else if (((my_kobuki.button & BUTTON_2_ROTATE) != 0U) && (mode == MODE_IDLE)) {
            rover.index_pos      = 0;
            rover.curr_min_value = 5000U;
            rover.curr_min_index = 0;
            sample_due = 0U;            /* discard stale tick */
            mode = MODE_SCAN;
        }

        /* Button 1 or bumper stops everything */
        if (((my_kobuki.button & BUTTON_1_STOP) != 0U)) {
            mode = MODE_IDLE;
        }
        /* ---- Mode actions ---- */
        switch (mode)
        {
            case MODE_IDLE:
                Send_Kobuki_Move(0 , 0);
                break;
            case MODE_SCAN:
                Kobuki_Rotate(ROTATE_SPEED_MM_S);
                if (sample_due) {
                    if (scan_state(&rover)) {
                        Send_Kobuki_Move(0, 0);
                        /* how far from where we face now to where the minimum was seen */
                        int32_t delta = (int32_t)rover.arc_angles[rover.curr_min_index] - kobuki.angle;
                        if (delta > 18000)  delta -= 36000;    /* take the short way round */
                        if (delta < -18000) delta += 36000;
                        Kobuki_Rotate_Angle_Setpoint(kobuki.angle, (int16_t)(delta / 100));
                        turn_start_ms = time_ms;
                        Kobuki_Rotate_Angle(kobuki.angle);
                        if ((kobuki.rotation_complete != 0U) || ((time_ms - turn_start_ms) >= TURN_TIMEOUT_MS)) {
                            Send_Kobuki_Move(0 , 0);
                            mode = MODE_IDLE;           /* now facing the closest object */
                        }
                    }
                }
                break;
            case MODE_NAVIGATE:
                navigation.ultrasonic_distance = ultrasonic_object.range_mm;
                navigation.object_detected     = ultrasonic_object.detected;
                navigation.cliff_detected      = (my_kobuki.cliff != 0U);
                navigation.bumper_detected = (my_kobuki.bumper !=0U);
                navigation.wheeldrop = my_kobuki.wheeldrop !=0U;
                Navigation_update(&navigation, time_ms);
                if (ultrasonic_object.detected && (ultrasonic_object.range_mm <= 400U)) {
                    Send_Kobuki_Move(0, 0);       // Safely halt navigation motion
                    Reset_Align_And_Deposit();    // Initialize align_deposit state machine variables
                    mode = MODE_ALIGN_DEPOSIT;    // Switch FSM state!
                }
                break;
            case MODE_ALIGN_DESOPIT:
                break;
            default:
                Send_Kobuki_Move(0 , 0);
                break;
        }

        for (volatile uint32_t i = 0; i < 5000U; i++) {
            __NOP();
        }
        loop_count++;
    }
}
