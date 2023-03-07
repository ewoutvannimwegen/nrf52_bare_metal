#include "nrf52_bitfields.h"
#include "sx1509_reg.h"
#include <nrf.h>

#define PIN_SDA (7UL)
#define PIN_SCL (8UL)
#define SX1509B (0x3E)
#define PIN_VDD_PWD_CTRL (30UL)

void delay_ms(uint32_t ms)
{
    for (uint32_t cycle = 0; cycle < SystemCoreClock / 8 / 1000 * ms; cycle++)
        ;
}

void i2c_write(uint8_t addr, uint8_t data)
{
    uint8_t tx_buf[2];

    tx_buf[0] = addr;
    tx_buf[1] = data;

    NRF_TWIM0->SHORTS = TWIM_SHORTS_LASTTX_STOP_Enabled << TWIM_SHORTS_LASTTX_STOP_Pos;

    delay_ms(1);

    NRF_TWIM0->TXD.MAXCNT = sizeof(tx_buf); 
    NRF_TWIM0->TXD.PTR = (uint32_t)&tx_buf;
    NRF_TWIM0->EVENTS_STOPPED = 0;
    NRF_TWIM0->TASKS_STARTTX = 1;
    while (NRF_TWIM0->EVENTS_STOPPED == 0)
        ;
}

uint8_t i2c_read(uint8_t addr)
{
    uint8_t tx_buf[1], rx_buf[1];

    tx_buf[0] = addr;

    NRF_TWIM0->SHORTS = (TWIM_SHORTS_LASTTX_STARTRX_Enabled << TWIM_SHORTS_LASTTX_STARTRX_Pos) |
                        (TWIM_SHORTS_LASTRX_STOP_Enabled << TWIM_SHORTS_LASTRX_STOP_Pos);

    NRF_TWIM0->TXD.MAXCNT = sizeof(tx_buf);
    NRF_TWIM0->RXD.MAXCNT = sizeof(rx_buf);

    delay_ms(1);

    NRF_TWIM0->TXD.MAXCNT = sizeof(tx_buf);
    NRF_TWIM0->RXD.MAXCNT = sizeof(rx_buf);
    NRF_TWIM0->TXD.PTR = (uint32_t)&tx_buf;
    NRF_TWIM0->RXD.PTR = (uint32_t)&rx_buf;

    NRF_TWIM0->EVENTS_STOPPED = 0;
    NRF_TWIM0->TASKS_STARTTX = 1;
    while (NRF_TWIM0->EVENTS_STOPPED == 0)
        ;
    return rx_buf[0];
}

int main(void)
{
    volatile static uint8_t sx1509_init_seq[] = {REG_RESET,
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
    volatile static uint8_t sx1509_write_buf[] = {}, sx1509_read_buf[] = {};

    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
        ;
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

    NRF_GPIO->PIN_CNF[PIN_VDD_PWD_CTRL] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    NRF_GPIO->OUTSET |= (1UL << PIN_VDD_PWD_CTRL);

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
    NRF_TWIM0->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K100 << TWIM_FREQUENCY_FREQUENCY_Pos;
    NRF_TWIM0->ADDRESS = SX1509B << TWIM_ADDRESS_ADDRESS_Pos;

    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;

    for (int idx = 0; idx < sizeof(sx1509_init_seq); idx=idx+2)
    {
        i2c_write(sx1509_init_seq[idx], sx1509_init_seq[idx + 1]);
    }

    for (int addr = 0; addr < sizeof(sx1509_init_seq); addr = addr + 2)
    {
        sx1509_read_buf[addr] = i2c_read(sx1509_init_seq[addr]);
    }
  
    sx1509_write_buf[0] = REG_DATA_A;
    sx1509_write_buf[1] = 0x0;
    sx1509_write_buf[2] = REG_DATA_A;
    sx1509_write_buf[3] = 0xFF;

    while (1) {
        i2c_write(sx1509_write_buf[0], sx1509_write_buf[1]);
        delay_ms(500);
        i2c_write(sx1509_write_buf[2], sx1509_write_buf[3]);
        delay_ms(500);
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
