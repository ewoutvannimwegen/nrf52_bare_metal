#include "lis2dh12_reg.h"
#include "nrf52_bitfields.h"
#include "sx1509_reg.h"
#include <nrf.h>

#define PIN_SDA (7UL)
#define PIN_SCL (8UL)
#define PIN_SDA_EXT (14UL)
#define PIN_SCL_EXT (15UL)
#define PIN_LIS_INT1 (12UL)
#define SX1509B (0x3E)
#define LIS2DH12 (0x19)
#define PIN_VDD_PWD_CTRL (30UL)

#define TWIM0_BUS 0
#define TWIM1_BUS 1
#define LIS2DH12_FIFO_LEN (10UL)
volatile static uint32_t lis2dh12_ptr;
volatile static uint8_t lis2dh12_xy[LIS2DH12_FIFO_LEN*4] = {};

void delay_ms(uint32_t ms)
{
    for (uint32_t cycle = 0; cycle < SystemCoreClock / 8 / 1000 * ms; cycle++)
        ;
}

void i2c_write(uint32_t bus, uint8_t addr, uint8_t data)
{
    uint8_t tx_buf[2];

    tx_buf[0] = addr;
    tx_buf[1] = data;

    if (bus == TWIM0_BUS)
    {
        NRF_TWIM0->SHORTS = TWIM_SHORTS_LASTTX_STOP_Enabled << TWIM_SHORTS_LASTTX_STOP_Pos;
    }
    else if(bus == TWIM1_BUS)
    {
        NRF_TWIM1->SHORTS = TWIM_SHORTS_LASTTX_STOP_Enabled << TWIM_SHORTS_LASTTX_STOP_Pos;
    }
    delay_ms(1);
    if (bus == TWIM0_BUS)
    {
        NRF_TWIM0->TXD.MAXCNT = sizeof(tx_buf);
        NRF_TWIM0->TXD.PTR = (uint32_t)&tx_buf;
        NRF_TWIM0->EVENTS_STOPPED = 0;
        NRF_TWIM0->TASKS_STARTTX = 1;
        while (NRF_TWIM0->EVENTS_STOPPED == 0)
            ;
    }
    else if (bus == TWIM1_BUS)
    {
        NRF_TWIM1->TXD.MAXCNT = sizeof(tx_buf);
        NRF_TWIM1->TXD.PTR = (uint32_t)&tx_buf;
        NRF_TWIM1->EVENTS_STOPPED = 0;
        NRF_TWIM1->TASKS_STARTTX = 1;
        while (NRF_TWIM1->EVENTS_STOPPED == 0)
            ;
    }
}

uint8_t i2c_read(uint32_t bus, uint8_t addr)
{
    uint8_t tx_buf[1], rx_buf[1];

    tx_buf[0] = addr;

    if (bus == TWIM0_BUS)
    {
        NRF_TWIM0->SHORTS = (TWIM_SHORTS_LASTTX_STARTRX_Enabled << TWIM_SHORTS_LASTTX_STARTRX_Pos) |
                            (TWIM_SHORTS_LASTRX_STOP_Enabled << TWIM_SHORTS_LASTRX_STOP_Pos);

        NRF_TWIM0->TXD.MAXCNT = sizeof(tx_buf);
        NRF_TWIM0->RXD.MAXCNT = sizeof(rx_buf);
    }
    else if(bus == TWIM1_BUS)
    {
        NRF_TWIM1->SHORTS = (TWIM_SHORTS_LASTTX_STARTRX_Enabled << TWIM_SHORTS_LASTTX_STARTRX_Pos) |
                            (TWIM_SHORTS_LASTRX_STOP_Enabled << TWIM_SHORTS_LASTRX_STOP_Pos);

        NRF_TWIM1->TXD.MAXCNT = sizeof(tx_buf);
        NRF_TWIM1->RXD.MAXCNT = sizeof(rx_buf);
    }
    delay_ms(1);
    if (bus == TWIM0_BUS)
    {
        NRF_TWIM0->TXD.MAXCNT = sizeof(tx_buf);
        NRF_TWIM0->RXD.MAXCNT = sizeof(rx_buf);
        NRF_TWIM0->TXD.PTR = (uint32_t)&tx_buf;
        NRF_TWIM0->RXD.PTR = (uint32_t)&rx_buf;

        NRF_TWIM0->EVENTS_STOPPED = 0;
        NRF_TWIM0->TASKS_STARTTX = 1;
        while (NRF_TWIM0->EVENTS_STOPPED == 0)
            ;
    }
    else if(bus == TWIM1_BUS)
    {
        NRF_TWIM1->TXD.MAXCNT = sizeof(tx_buf);
        NRF_TWIM1->RXD.MAXCNT = sizeof(rx_buf);
        NRF_TWIM1->TXD.PTR = (uint32_t)&tx_buf;
        NRF_TWIM1->RXD.PTR = (uint32_t)&rx_buf;

        NRF_TWIM1->EVENTS_STOPPED = 0;
        NRF_TWIM1->TASKS_STARTTX = 1;
        while (NRF_TWIM1->EVENTS_STOPPED == 0)
            ;
    }
    return rx_buf[0];
}

void lis2dh12_wtm_handler(void) {
    lis2dh12_xy[lis2dh12_ptr] = i2c_read(TWIM1_BUS, LIS_REG_OUT_X_L_ADDR);
    lis2dh12_xy[lis2dh12_ptr+1] = i2c_read(TWIM1_BUS, LIS_REG_OUT_X_H_ADDR);
    lis2dh12_xy[lis2dh12_ptr+2] = i2c_read(TWIM1_BUS, LIS_REG_OUT_Y_L_ADDR);
    lis2dh12_xy[lis2dh12_ptr+3] = i2c_read(TWIM1_BUS, LIS_REG_OUT_Y_H_ADDR);
    lis2dh12_ptr += 4;
    if(lis2dh12_ptr > sizeof(lis2dh12_xy)) {
        lis2dh12_ptr = 0;
    }
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
    volatile static uint8_t sx1509_write_buf[32] = {}, sx1509_read_buf[32] = {};
    volatile static uint8_t lis2dh12_write_buf[32] = {}, lis2dh12_read_buf[32] = {};

    volatile static uint8_t lis2dh12_init_seq[] = {
        LIS_REG_CTRL0_ADDR, 
        LIS_CTRL0_DEFAULT, 
        LIS_REG_TEMP_CFG_ADDR, 
        LIS_TEMP_CFG_DEFAULT,
        LIS_REG_CTRL1_ADDR, 
        LIS_CTRL1_DEFAULT,        
        LIS_REG_CTRL2_ADDR,
        LIS_CTRL2_DEFAULT,
        LIS_REG_CTRL3_ADDR,
        LIS_CTRL3_DEFAULT,
        LIS_REG_CTRL4_ADDR,
        LIS_CTRL4_DEFAULT,
        LIS_REG_CTRL5_ADDR,
        LIS_CTRL5_DEFAULT,
        LIS_REG_CTRL6_ADDR,
        LIS_CTRL6_DEFAULT,
        LIS_REG_FIFO_CTRL_ADDR,
        LIS_FIFO_CTRL_DEFAULT,
        LIS_REG_INT1_CFG_ADDR,
        LIS_INT1_CFG_DEFAULT,
        LIS_REG_INT1_THS_ADDR,
        LIS_INT1_THS_DEFAULT,
        LIS_REG_INT1_DURATION_ADDR,
        LIS_INT2_DURATION_DEFAULT,
        LIS_REG_INT2_CFG_ADDR,
        LIS_INT2_CFG_DEFAULT,
        LIS_REG_INT2_THS_ADDR,
        LIS_INT2_THS_DEFAULT,
        LIS_REG_INT2_DURATION_ADDR,
        LIS_INT2_DURATION_DEFAULT,
        LIS_REG_CLICK_CFG_ADDR,
        LIS_CLICK_CFG_DEFAULT,
        LIS_REG_CLICK_THS_ADDR,
        LIS_CLICK_THS_DEFAULT,
        LIS_REG_TIME_LIMIT_ADDR,
        LIS_TIME_LIMIT_DEFAULT,
        LIS_REG_TIME_LATENCY_ADDR,
        LIS_TIME_LATENCY_DEFAULT,
        LIS_REG_TIME_WINDOW_ADDR,
        LIS_TIME_WINDOW_DEFAULT,
        LIS_REG_ACT_THS_ADDR,
        LIS_ACT_THS_DEFAULT,
        LIS_REG_ACT_DUR_ADDR,
        LIS_ACT_DUR_DEFAULT, // End of reset
    };
    lis2dh12_ptr = 0;

    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
        ;
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    
    NRF_TIMER0->TASKS_STOP = 1;
    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->PRESCALER = 8; // 16MHz / (2^prescaler); 16MHz/8=62.5k
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER0->CC[0] = 62500;
    NRF_TIMER0->CC[1] = 125000;
    NRF_TIMER0->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos) | 
        (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
    NVIC_SetPriority(TIMER0_IRQn, 8);
    NVIC_EnableIRQ(TIMER0_IRQn);
    NRF_TIMER0->TASKS_START = 1;

    NRF_SPIS0->ENABLE = SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos;
    NRF_SPI0->ENABLE = SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos;
    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;
    NRF_TWIS0->ENABLE = TWIS_ENABLE_ENABLE_Disabled << TWIS_ENABLE_ENABLE_Pos;
    NRF_TWI0->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;

    NRF_TWI1->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
    NRF_TWIM1->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;
    NRF_TWIS1->ENABLE = TWIS_ENABLE_ENABLE_Disabled << TWIS_ENABLE_ENABLE_Pos;
    NRF_SPI1->ENABLE = SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos;
    NRF_SPIM1->ENABLE = SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos;
    NRF_SPIS1->ENABLE = SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos;

    NRF_GPIO->PIN_CNF[PIN_LIS_INT1] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
        (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
    NRF_GPIO->DETECTMODE |= (1UL << PIN_LIS_INT1);

    NRF_GPIOTE->CONFIG[GPIOTE_INTENSET_IN0_Pos] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                                  (PIN_LIS_INT1 << GPIOTE_CONFIG_PSEL_Pos) |
                                                  (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

    NRF_GPIOTE->INTENSET |= (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos);
    NVIC_EnableIRQ(GPIOTE_IRQn);

    NRF_GPIO->PIN_CNF[PIN_VDD_PWD_CTRL] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    NRF_GPIO->OUTSET |= (1UL << PIN_VDD_PWD_CTRL);

    NRF_GPIO->PIN_CNF[PIN_SDA] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    NRF_GPIO->PIN_CNF[PIN_SCL] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    NRF_GPIO->PIN_CNF[PIN_SDA_EXT] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    NRF_GPIO->PIN_CNF[PIN_SCL_EXT] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    NRF_TWIM0->PSEL.SDA =
        (TWIM_PSEL_SDA_CONNECT_Connected << TWIM_PSEL_SDA_CONNECT_Pos) | (PIN_SDA << TWIM_PSEL_SDA_PIN_Pos);
    NRF_TWIM0->PSEL.SCL =
        (TWIM_PSEL_SCL_CONNECT_Connected << TWIM_PSEL_SCL_CONNECT_Pos) | (PIN_SCL << TWIM_PSEL_SCL_PIN_Pos);
    NRF_TWIM0->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K100 << TWIM_FREQUENCY_FREQUENCY_Pos;
    NRF_TWIM0->ADDRESS = SX1509B << TWIM_ADDRESS_ADDRESS_Pos;

    NRF_TWIM1->PSEL.SDA =
        (TWIM_PSEL_SDA_CONNECT_Connected << TWIM_PSEL_SDA_CONNECT_Pos) | (PIN_SDA_EXT << TWIM_PSEL_SDA_PIN_Pos);
    NRF_TWIM1->PSEL.SCL =
        (TWIM_PSEL_SCL_CONNECT_Connected << TWIM_PSEL_SCL_CONNECT_Pos) | (PIN_SCL_EXT << TWIM_PSEL_SCL_PIN_Pos);
    NRF_TWIM1->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K400 << TWIM_FREQUENCY_FREQUENCY_Pos;
    NRF_TWIM1->ADDRESS = LIS2DH12 << TWIM_ADDRESS_ADDRESS_Pos;

    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;
    NRF_TWIM1->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;

    for (int idx = 0; idx < sizeof(sx1509_init_seq); idx = idx + 2)
    {
        i2c_write(TWIM0_BUS, sx1509_init_seq[idx], sx1509_init_seq[idx + 1]);
    }

    for (int addr = 0; addr < sizeof(sx1509_init_seq)/2;addr++)
    {
        sx1509_read_buf[addr] = i2c_read(TWIM0_BUS, sx1509_init_seq[addr*2]);
    }
    
    for (int idx = 0; idx < sizeof(lis2dh12_init_seq); idx = idx + 2)
    {
        i2c_write(TWIM1_BUS, lis2dh12_init_seq[idx], lis2dh12_init_seq[idx + 1]);
    }
    
    for (int addr = 0; addr < sizeof(lis2dh12_init_seq)/2; addr++)
    {
        lis2dh12_read_buf[addr] = i2c_read(TWIM1_BUS, lis2dh12_init_seq[addr*2]);
    }
    i2c_write(TWIM1_BUS, LIS_REG_CTRL3_ADDR, (LIS_REG_CTRL3_I1_OVERRUN_En << LIS_REG_CTRL3_I1_OVERRUN_Pos) | (LIS_REG_CTRL3_I1_WTM_En << LIS_REG_CTRL3_I1_WTM_Pos));
    i2c_write(TWIM1_BUS, LIS_REG_FIFO_CTRL_ADDR, (LIS_REG_FIFO_CTRL_FM_STREAM << LIS_REG_FIFO_CTRL_FM_Pos) | (LIS2DH12_FIFO_LEN << LIS_REG_FIFO_CTRL_FTH_Pos));
    i2c_write(TWIM1_BUS, LIS_REG_FIFO_SRC_ADDR, (LIS_REG_FIFO_SRC_WTM_Enable << LIS_REG_FIFO_SRC_WTM_Pos) | (LIS_REG_FIFO_SRC_OVRN_FIFO_Enable << LIS_REG_FIFO_SRC_OVRN_FIFO_Pos) | (LIS_REG_FIFO_SRC_EMPTY_Enable << LIS_REG_FIFO_SRC_EMPTY_Pos));
    i2c_write(TWIM1_BUS, LIS_REG_CTRL4_ADDR, LIS_REG_CTRL4_HR_12B << LIS_REG_CTRL4_HR_Pos);
    i2c_write(TWIM1_BUS, LIS_REG_CTRL5_ADDR, LIS_REG_CTRL5_ADDR_FIFO_EN_Enable << LIS_REG_CTRL5_ADDR_FIFO_EN_Pos);
    i2c_write(TWIM1_BUS, LIS_REG_CTRL1_ADDR, (LIS_REG_CTRL1_HR_12B << LIS_REG_CTRL1_HR_Pos) | (LIS_CTRL1_ODR_5376HZ << LIS_CTRL1_ODR_FIELD_OFFSET) | LIS_CTRL1_X_EN | LIS_CTRL1_Y_EN);
    while (1);
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

void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void)
{
    if (NRF_TWIM1->EVENTS_STOPPED)
    {
        NRF_TWIM1->EVENTS_STOPPED = 0;
    }
    if (NRF_TWIM1->EVENTS_ERROR)
    {
        NRF_TWIM1->EVENTS_ERROR = 0;
    }
    if (NRF_TWIM1->EVENTS_SUSPENDED)
    {
        NRF_TWIM1->EVENTS_SUSPENDED = 0;
    }
    if (NRF_TWIM1->EVENTS_RXSTARTED)
    {
        NRF_TWIM1->EVENTS_RXSTARTED = 0;
    }
    if (NRF_TWIM1->EVENTS_TXSTARTED)
    {
        NRF_TWIM1->EVENTS_TXSTARTED = 0;
    }
    if (NRF_TWIM1->EVENTS_LASTRX)
    {
        NRF_TWIM1->EVENTS_LASTRX = 0;
    }
    if (NRF_TWIM1->EVENTS_LASTTX)
    {
        NRF_TWIM1->EVENTS_LASTTX = 0;
    }
}

void TIMER0_IRQHandler(void)
{
    if(NRF_TIMER0->EVENTS_COMPARE[0]) {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        i2c_write(TWIM0_BUS, REG_DATA_A, 0xFF);
    }
    if(NRF_TIMER0->EVENTS_COMPARE[1]) {
        NRF_TIMER0->EVENTS_COMPARE[1] = 0;
        NRF_TIMER0->TASKS_CLEAR = 1;
        i2c_write(TWIM0_BUS, REG_DATA_A, 0x0);
    }
}

void GPIOTE_IRQHandler(void)
{
    if (NRF_GPIOTE->EVENTS_IN[GPIOTE_INTENSET_IN0_Pos])
    {
        if((i2c_read(TWIM1_BUS, LIS_REG_FIFO_SRC_ADDR) >> LIS_REG_FIFO_SRC_WTM_Pos) & 1UL) {
            lis2dh12_wtm_handler();
        }
        if((i2c_read(TWIM1_BUS, LIS_REG_FIFO_SRC_ADDR) >> LIS_REG_FIFO_SRC_OVRN_FIFO_Pos) & 1UL) {
            __NOP();
        }
        if((i2c_read(TWIM1_BUS, LIS_REG_FIFO_SRC_ADDR) >> LIS_REG_FIFO_SRC_EMPTY_Pos) & 1UL) {
            __NOP();
        }
    }
}
