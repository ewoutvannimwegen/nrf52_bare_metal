#include "nrf52_bitfields.h"
#include <nrf.h>

#define UART_TX (2UL)
#define UART_RX (4UL)
#define BUT1 (13UL)
#define LED1 (17UL)
#define LED2 (18UL)
#define LED3 (19UL)

volatile static const uint8_t buf[] = {'p', 'e', 'l', 'l', 'o', ',', 'w', 'o', 'r', 'l', 'd', '!', '\r', '\n'};

void uarte_endtx_handler(void)
{
    NRF_UARTE0->TXD.PTR = (uint32_t)&buf[0];
    NRF_UARTE0->TXD.MAXCNT = sizeof(buf);
    NRF_UARTE0->TASKS_STARTTX = 1;
    while (!(NRF_UARTE0->EVENTS_TXSTARTED))
        ;
    NRF_UARTE0->EVENTS_TXSTARTED = 0;
}

int main(void)
{
    NRF_CLOCK->TASKS_HFCLKSTART = 1; 
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
        ;
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Disabled << UART_ENABLE_ENABLE_Pos; // Disable other peripherals with same ID
    NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Disabled << UARTE_ENABLE_ENABLE_Pos;
    NRF_UARTE0->PSEL.RTS = UARTE_PSEL_RTS_CONNECT_Disconnected << UARTE_PSEL_RTS_CONNECT_Pos;
    NRF_UARTE0->PSEL.CTS = UARTE_PSEL_CTS_CONNECT_Disconnected << UARTE_PSEL_CTS_CONNECT_Pos;
    NRF_UARTE0->TASKS_STOPRX = 1;
    NRF_UARTE0->PSEL.RXD = UARTE_PSEL_RXD_CONNECT_Disconnected << UARTE_PSEL_RXD_CONNECT_Pos;

    NRF_GPIO->PIN_CNF[UART_TX] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    NRF_GPIO->OUTSET |= (1UL << UART_TX);

    NRF_UARTE0->TASKS_STOPTX = 1;
    NRF_UARTE0->INTENSET = (UARTE_INTENSET_ERROR_Enabled << UARTE_INTENSET_ERROR_Pos) |
                           (UARTE_INTENSET_TXDRDY_Enabled << UARTE_INTENSET_TXDRDY_Pos) |
                           (UARTE_INTENSET_ENDTX_Enabled << UARTE_INTENSET_ENDTX_Pos);
    NRF_UARTE0->PSEL.TXD = UART_TX;
    NRF_UARTE0->BAUDRATE = UARTE_BAUDRATE_BAUDRATE_Baud9600 << UARTE_BAUDRATE_BAUDRATE_Pos;
    NRF_UARTE0->CONFIG = (UARTE_CONFIG_HWFC_Disabled << UARTE_CONFIG_HWFC_Pos) |
                         (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos);

    NVIC_EnableIRQ(UARTE0_UART0_IRQn);
    NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos;

    NRF_UARTE0->TXD.PTR = (uint32_t)&buf[0];
    NRF_UARTE0->TXD.MAXCNT = sizeof(buf);

    NRF_UARTE0->TASKS_STARTTX = 1;
    while (!(NRF_UARTE0->EVENTS_TXSTARTED))
        ;
    NRF_UARTE0->EVENTS_TXSTARTED = 0;

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

    NRF_TIMER0->TASKS_STOP = 1;
    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->PRESCALER = 8; // 16MHz / (2^prescaler); 16MHz/8=62.5k
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER0->CC[0] = 62500;
    NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
    NVIC_EnableIRQ(TIMER0_IRQn);
    NRF_TIMER0->TASKS_START = 1;

    NRF_TIMER1->TASKS_STOP = 1;
    NRF_TIMER1->TASKS_CLEAR = 1;
    NRF_TIMER1->PRESCALER = 8; // 16MHz / (2^prescaler); 16MHz/8=62.5k
    NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER1->CC[1] = 31250;
    NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos;
    NVIC_EnableIRQ(TIMER1_IRQn);
    NRF_TIMER1->TASKS_START = 1;

    NRF_GPIO->PIN_CNF[LED1] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    NRF_GPIO->PIN_CNF[LED2] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    NRF_GPIO->PIN_CNF[LED3] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    NRF_GPIO->DETECTMODE |= (1UL << LED3);

    NRF_GPIOTE->TASKS_OUT[GPIOTE_INTENSET_IN1_Pos] = 1;
    NRF_GPIOTE->CONFIG[GPIOTE_INTENSET_IN1_Pos] = (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos) |
                                                  (LED3 << GPIOTE_CONFIG_PSEL_Pos) |
                                                  (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
                                                  (GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos);

    NRF_GPIO->OUTSET |= (1UL << LED1);
    NRF_GPIO->OUTSET |= (1UL << LED2);

    while (1)
    {
        __WFE();
    }
    return 0;
}

void UARTE0_UART0_IRQHandler(void)
{
    if (NRF_UARTE0->EVENTS_ERROR)
    {
        NRF_UARTE0->EVENTS_ERROR = 0;
        if ((NRF_UARTE0->ERRORSRC >> UART_ERRORSRC_OVERRUN_Pos) & 1UL)
        {
            __WFE();
        }
        if ((NRF_UARTE0->ERRORSRC >> UART_ERRORSRC_PARITY_Pos) & 1UL)
        {
            __WFE();
        }
        if ((NRF_UARTE0->ERRORSRC >> UART_ERRORSRC_FRAMING_Pos) & 1UL)
        {
            __WFE();
        }
        if ((NRF_UARTE0->ERRORSRC >> UART_ERRORSRC_BREAK_Pos) & 1UL)
        {
            __WFE();
        }
    }
    if (NRF_UARTE0->EVENTS_TXDRDY)
    {
        NRF_UARTE0->EVENTS_TXDRDY = 0;
    }
    if (NRF_UARTE0->EVENTS_ENDTX)
    {
        NRF_UARTE0->EVENTS_ENDTX = 0;
        uarte_endtx_handler();
    }
}

void GPIOTE_IRQHandler(void)
{
    if (NRF_GPIOTE->EVENTS_IN[GPIOTE_INTENSET_IN0_Pos])
    {
        NRF_GPIOTE->EVENTS_IN[GPIOTE_INTENSET_IN0_Pos] = 0;
        NRF_GPIO->OUT ^= (1UL << LED1); // Toggle LED1
    }
}

void TIMER0_IRQHandler(void)
{
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_GPIO->OUT ^= (1UL << LED2); // Toggle LED2
}

void TIMER1_IRQHandler(void)
{
    NRF_TIMER1->EVENTS_COMPARE[1] = 0;
    NRF_TIMER1->TASKS_CLEAR = 1;
    NRF_GPIOTE->TASKS_OUT[GPIOTE_INTENSET_IN1_Pos] = 1; // Toggle LED3
}
