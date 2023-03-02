#include <nrf.h>

#define LED1 (17UL)
#define LED2 (18UL)

void led_init()
{
    NRF_GPIO->PIN_CNF[LED1] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                               (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                               (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                               (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                               (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    
    NRF_GPIO->PIN_CNF[LED2] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                               (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                               (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                               (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                               (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

int main(void)
{
    led_init();
    while (1)
    {
        uint32_t volatile tmo;

        tmo = 10000000;
        while (tmo--)
            ;
        NRF_GPIO->OUTSET = (1UL << LED1);
        NRF_GPIO->OUTCLR = (1UL << LED2);

        tmo = 10000000;
        while (tmo--)
            ;
        NRF_GPIO->OUTCLR = (1UL << LED1);
        NRF_GPIO->OUTSET = (1UL << LED2);
    }
}
