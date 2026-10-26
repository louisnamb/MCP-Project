#include <stdint.h>
#define STM32G474xx
#include "stm32g4xx.h"
#include "kobuki.h"
#include "uart.h"
#include "adc.h"
#include "ultrasonic.h"
#define BUTTON_0_START 0x01U
#define BUTTON_1_STOP 0x02U
#define BUTTON_2_ROTATE 0x04U
#define DRIVE_SPEED_MM_S ((int16_t) 100)
#define ROTATE_SPEED_MM_S ((int16_t) 100)
#define MAX_SAMPLES 100U
typedef struct {
    uint16_t arc_positions[MAX_SAMPLES];
    uint16_t index_pos;
    uint16_t curr_min_value;
    uint16_t curr_min_index;
} Master_t;
static Master_t rover = {0}; /* static so you can watch it in the debugger */
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
/* Takes one sample, returns 1 when all samples are collected */
static uint8_t scan_state(Master_t *r)
{
    uint16_t d = Ultrasonic_Read();
    r->arc_positions[r->index_pos] = d;
    if (d < r->curr_min_value) {
        r->curr_min_value = d;
        r->curr_min_index = r->index_pos;
    }
        r->index_pos++;
    return (r->index_pos >= MAX_SAMPLES);
}
/* TIM17 as a free-running sample timer. Polled (no interrupt, no NVIC). */
static void SampleTimer_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    TIM17->PSC = 15; /* 16 MHz 16 = 1 MHz. Change if your timer clock differs */
    TIM17->ARR = 99999; /* 100 ms per sample */
    TIM17->EGR = TIM_EGR_UG;
    TIM17->SR = 0;
    TIM17->CR1 |= TIM_CR1_CEN;
}

int main(void) {
    Kobuki_Typedef my_kobuki = {0};
    US_Typedef ultrasonic_object = {0};
    uint8_t is_running = 0;
    uint8_t is_scanning = 0;
    UART_Init();
    ADC_Init();
    Ultrasonic_Init();
    SampleTimer_Init();
    Send_Kobuki_Move(0 , 0);
    while (1)
    {
        Kobuki_Read(&my_kobuki);
        Ultrasonic_Detect(&ultrasonic_object);
        /* Button 0: drive forward when clear (unchanged from working code) */
        if (((my_kobuki.button & BUTTON_0_START) != 0U) && (ultrasonic_object.detected == 0U) && (is_scanning == 0U)) {
            is_running = 1U;
        }
        /* Button 2: start a rotating scan */
        else if (((my_kobuki.button & BUTTON_2_ROTATE) != 0U) && (is_scanning == 0U)) {
            rover.index_pos = 0;
            rover.curr_min_value = 5000U;
            rover.curr_min_index = 0;
            TIM17->CNT = 0;
            TIM17->SR = 0; /* discard stale tick */
            is_scanning = 1U;
            is_running = 0U;
        }
        /* Button 1 or bumper stops everything, including the scan */
        if (((my_kobuki.button & BUTTON_1_STOP) != 0U) || (my_kobuki.bumper != 0U)) {
            is_running = 0U;
            is_scanning = 0U;
        }
        /* Ultrasonic detection stops driving only (the scan needs to see objects) */
        if (ultrasonic_object.detected != 0U) {
            is_running = 0U;
        }
        if (is_scanning != 0U) {
            Kobuki_Rotate(ROTATE_SPEED_MM_S);
            if (TIM17->SR & TIM_SR_UIF) {
                TIM17->SR &= ~TIM_SR_UIF;
                if (scan_state(&rover)) {
                    is_scanning = 0U; /* done: results in rover.curr_min_ */
                }
            }
        }
        else if (is_running != 0U) {
            Kobuki_Drive(DRIVE_SPEED_MM_S);
        }
        else {
            Send_Kobuki_Move(0 , 0);
        }
        for (volatile uint32_t i = 0; i < 5000U; i++) {
            __NOP();
        }
    }
}