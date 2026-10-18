#include <stdint.h>
#define STM32G474xx
#include "stm32g4xx.h"
#include "uart.h"
#include "kobuki.h"

#define MAX_SAMPLES 200
uint32_t pos = 0;
uint32_t curr_min_value = ULTRASONIC_MAX_RANGE_MM;
uint32_t curr_min_index = 0;
uint32_t arc_positions[200];

void Timer_Init(TIM_TypeDef *timer, IRQn_Type irq_type, uint16_t psc, uint16_t arr, bool enable_interrupt)
{
    timer->PSC = psc;
    timer->ARR = arr;
    if (enable_interrupt) {
        timer->DIER |= 1<<0; // Turn on interrupts for the Timer
        NVIC_EnableIRQ(irq_type); // Allow timer to interrupt Cortex processor
    }
    // Enable Timer/Counter
    timer->CR1 |= 1<<0;
}

void Port_Clock_Enable(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN;
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
    RCC->APBENR2  |= RCC_APBENR2_TIM1EN;
}

uint8_t scan_state()
{
    // Detect current distance
    US_Typedef ultrasonic_object = {0};
    Ultrasonic_Detect(&ultrasonic_object);
    arc_positions[pos] = ultrasonic_object.range_mm;
    if (ultrasonic_object.detected && arc_positions[pos] < curr_min_value) //update max value
    {
        curr_min_value = arc_positions[pos];
        curr_min_index = pos;
    }
    if pos > MAX_SAMPLES
    {
        pos = 0
        curr_min_value = ULTRASONIC_MAX_RANGE_MM;
        curr_min_index = 0;
        return (1);
    }
    pos++;
    return (0); 
}