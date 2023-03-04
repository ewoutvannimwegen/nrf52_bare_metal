#include "nrf52_bitfields.h"
#include <nrf.h>

#define BUT1 (13UL)
#define LED1 (17UL)

int main(void)
{
    NRF_GPIO->PIN_CNF[BUT1] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
        (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
    
    NRF_GPIO->DETECTMODE |= (1UL << BUT1);
    
    NRF_GPIOTE->CONFIG[GPIOTE_INTENSET_IN0_Pos] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
        (BUT1 << GPIOTE_CONFIG_PSEL_Pos) |
        (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);

    NRF_GPIOTE->INTENSET |= (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos);
   
    NVIC_EnableIRQ(GPIOTE_IRQn);

    NRF_GPIO->PIN_CNF[LED1] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    NRF_GPIO->OUTSET |= (1UL << LED1);

    while (1);
    return 0;
}

void GPIOTE_IRQHandler(void) {
    if (NRF_GPIOTE->EVENTS_IN[GPIOTE_INTENSET_IN0_Pos]) {
        NRF_GPIOTE->EVENTS_IN[GPIOTE_INTENSET_IN0_Pos] = 0; 
        NRF_GPIO->OUT ^= (1UL << LED1); // Toggle LED1
    }
}
