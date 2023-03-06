#include "nrf52_bitfields.h"
#include "sx1509_reg.h"
#include <nrf.h>

#define UART_TX (2UL)
#define UART_RX (4UL)
#define BUT1 (11UL)
#define LED1 (13UL) // Active low
#define LED2 (14UL)
#define LED3 (15UL)
#define PIN_SDA (7UL)
#define PIN_SCL (8UL)
#define PIN_SX_OSCIO (5UL)
#define LIS_INT1 (12UL)
#define SDA_EXT (14UL)
#define SCL_EXT (15UL)
#define SX1509B (0x3E)
#define PIN_VDD_PWD_CTRL (30UL)

volatile static uint8_t twim0_tx_buf[] = {REG_RESET,
                                          0x12,
                                          REG_RESET,
                                          0x34,
                                          REG_INPUT_DISABLE_B,
                                          0xFF,
                                          REG_INPUT_DISABLE_A,
                                          0xFF,
                                          REG_PULL_UP_B,
                                          0x0,
                                          REG_PULL_UP_A,
                                          0x0,
                                          REG_OPEN_DRAIN_B,
                                          0xFF,
                                          REG_OPEN_DRAIN_A,
                                          0xFF,
                                          REG_DIR_B,
                                          0x0,
                                          REG_DIR_A,
                                          0x0,
                                          REG_CLOCK,
                                          0b01000001,
                                          REG_MISC,
                                          0b00010000,
                                          REG_LED_DRIVER_ENABLE_B,
                                          0xFF,
                                          REG_LED_DRIVER_ENABLE_A,
                                          0xFF,
                                          REG_DATA_B,
                                          0x0,
                                          REG_DATA_A,
                                          0x0};
volatile static uint8_t twim0_rx_buf[] = {};

int main(void)
{
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
        ;
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

    NRF_GPIO->PIN_CNF[PIN_VDD_PWD_CTRL] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    NRF_GPIO->OUTSET |= (1UL << PIN_VDD_PWD_CTRL);

    // Disable other peripherals with same ID
    NRF_SPIM0->ENABLE = SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos;
    NRF_SPIS0->ENABLE = SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos;
    NRF_SPI0->ENABLE = SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos;
    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;
    NRF_TWIS0->ENABLE = TWIS_ENABLE_ENABLE_Disabled << TWIS_ENABLE_ENABLE_Pos;
    NRF_TWI0->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;

    NRF_GPIO->PIN_CNF[PIN_SDA] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    NRF_GPIO->PIN_CNF[PIN_SCL] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    NRF_TWIM0->PSEL.SDA =
        (TWIM_PSEL_SDA_CONNECT_Connected << TWIM_PSEL_SDA_CONNECT_Pos) | (PIN_SDA << TWIM_PSEL_SDA_PIN_Pos);
    NRF_TWIM0->PSEL.SCL =
        (TWIM_PSEL_SCL_CONNECT_Connected << TWIM_PSEL_SCL_CONNECT_Pos) | (PIN_SCL << TWIM_PSEL_SCL_PIN_Pos);
    NRF_TWIM0->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K400 << TWIM_FREQUENCY_FREQUENCY_Pos;

    NRF_TWIM0->ADDRESS = SX1509B << TWIM_ADDRESS_ADDRESS_Pos;
    NRF_TWIM0->TXD.PTR = (uint32_t)&twim0_tx_buf[0];
    NRF_TWIM0->TXD.MAXCNT = sizeof(twim0_tx_buf);
    NRF_TWIM0->RXD.PTR = (uint32_t)&twim0_rx_buf[0];
    NRF_TWIM0->RXD.MAXCNT = 1;

    NRF_TWIM0->INTENSET = (TWIM_INTENSET_STOPPED_Enabled << TWIM_INTENSET_STOPPED_Pos) |
                          (TWIM_INTENSET_ERROR_Enabled << TWIM_INTENSET_ERROR_Pos) |
                          (TWIM_INTENSET_SUSPENDED_Enabled << TWIM_INTENSET_SUSPENDED_Pos) |
                          (TWIM_INTENSET_RXSTARTED_Enabled << TWIM_INTENSET_RXSTARTED_Pos) |
                          (TWIM_INTENSET_TXSTARTED_Enabled << TWIM_INTENSET_TXSTARTED_Pos) |
                          (TWIM_INTENSET_LASTRX_Enabled << TWIM_INTENSET_LASTRX_Pos) |
                          (TWIM_INTENSET_LASTTX_Enabled << TWIM_INTENSET_LASTTX_Pos);
    NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;
    NRF_TWIM0->TASKS_STARTTX = 1;

    while (1)
    {
        __WFE();
    }
    return 0;
}

void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void)
{
    if (NRF_TWIM0->EVENTS_STOPPED)
    {
        NRF_TWIM0->EVENTS_STOPPED = 0;
    }
    if (NRF_TWIM0->EVENTS_ERROR)
    {
        NRF_TWIM0->EVENTS_ERROR = 0;
        if ((NRF_TWIM0->ERRORSRC >> TWIM_ERRORSRC_OVERRUN_Pos) & 1UL)
        {
            __NOP();
        }
        if ((NRF_TWIM0->ERRORSRC >> TWIM_ERRORSRC_ANACK_Pos) & 1UL)
        {
            __NOP();
        }
        if ((NRF_TWIM0->ERRORSRC >> TWIM_ERRORSRC_DNACK_Pos) & 1UL)
        {
            __NOP();
        }
    }
    if (NRF_TWIM0->EVENTS_SUSPENDED)
    {
        NRF_TWIM0->EVENTS_SUSPENDED = 0;
    }
    if (NRF_TWIM0->EVENTS_RXSTARTED)
    {
        NRF_TWIM0->EVENTS_RXSTARTED = 0;
    }
    if (NRF_TWIM0->EVENTS_TXSTARTED)
    {
        NRF_TWIM0->EVENTS_TXSTARTED = 0;
    }
    if (NRF_TWIM0->EVENTS_LASTRX)
    {
        NRF_TWIM0->EVENTS_LASTRX = 0;
    }
    if (NRF_TWIM0->EVENTS_LASTTX)
    {
        NRF_TWIM0->EVENTS_LASTTX = 0;
    }
}
