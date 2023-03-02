#include <nrf.h>

#define BUT1 (13UL)
#define LED1 (17UL)

void button_init()
{
    NRF_GPIO->PIN_CNF[BUT1] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

void led_init()
{
    NRF_GPIO->PIN_CNF[LED1] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

int main(void)
{
    button_init();
    led_init();

    while (1)
    {
        uint32_t tmo;

        tmo = 100000;
        while (tmo--)
            ;

        if ((NRF_GPIO->IN >> BUT1) & 1UL)
        {
            NRF_GPIO->OUTSET = (1UL << LED1); // LED1 off
        }
        else
        {
            // BUT1 being pressed
            NRF_GPIO->OUTCLR = (1UL << LED1); // LED1 on
        }
    }
}
