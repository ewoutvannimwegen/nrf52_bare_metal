#include "accel.h"
#define PIN_UART_TX (2UL)
#define PIN_UART_RX (4UL)
#define PIN_LIS_INT1 (12UL)
#define PIN_SDA_EXT (14UL)
#define PIN_SCL_EXT (15UL)
#define PIN_VDD_PWD_CTRL (30UL)
#define LIS2DH12 (0x19)

void i2c_write(uint8_t addr, uint8_t data)
{
    uint8_t tx_buf[2] = {addr, data};
    NRF_TWIM0->SHORTS = TWIM_SHORTS_LASTTX_STOP_Msk;
    NRF_TWIM0->TXD.MAXCNT = sizeof(tx_buf);
    NRF_TWIM0->TXD.PTR = (uint32_t)&tx_buf[0];
    NRF_TWIM0->TASKS_STARTTX = 1;
    while (NRF_TWIM0->EVENTS_STOPPED == 0);
    NRF_TWIM0->EVENTS_TXSTARTED = 0;
    NRF_TWIM0->EVENTS_RXSTARTED = 0;
    NRF_TWIM0->EVENTS_LASTTX = 0;
    NRF_TWIM0->EVENTS_LASTRX = 0;
    NRF_TWIM0->EVENTS_STOPPED = 0;
}

uint8_t i2c_read(uint8_t addr)
{
    uint8_t tx_buf[1] = {addr}, rx_buf[1] = {0};
    NRF_TWIM0->SHORTS = (TWIM_SHORTS_LASTTX_STARTRX_Enabled << TWIM_SHORTS_LASTTX_STARTRX_Pos) |
                        (TWIM_SHORTS_LASTRX_STOP_Enabled << TWIM_SHORTS_LASTRX_STOP_Pos);
    NRF_TWIM0->TXD.MAXCNT = sizeof(tx_buf);
    NRF_TWIM0->RXD.MAXCNT = sizeof(rx_buf);
    NRF_TWIM0->TXD.PTR = (uint32_t)&tx_buf[0];
    NRF_TWIM0->RXD.PTR = (uint32_t)&rx_buf[0];
    NRF_TWIM0->TASKS_STARTTX = 1;
    while (NRF_TWIM0->EVENTS_STOPPED == 0);
    NRF_TWIM0->EVENTS_TXSTARTED = 0;
    NRF_TWIM0->EVENTS_RXSTARTED = 0;
    NRF_TWIM0->EVENTS_LASTTX = 0;
    NRF_TWIM0->EVENTS_LASTRX = 0;
    NRF_TWIM0->EVENTS_STOPPED = 0;
    return rx_buf[0];
}

void lis2dh12_getSamples(void) {
//    uint8_t tx_buf[1] = {LIS_REG_OUT_X_L_ADDR};
//    NRF_TWIM0->INTENSET |= TWIM_INTENSET_STOPPED_Enabled << TWIM_INTENSET_STOPPED_Pos;
//    NRF_TWIM0->SHORTS = (TWIM_SHORTS_LASTTX_STARTRX_Enabled << TWIM_SHORTS_LASTTX_STARTRX_Pos) |
//                        (TWIM_SHORTS_LASTRX_STOP_Enabled << TWIM_SHORTS_LASTRX_STOP_Pos);
//    NRF_TWIM0->TXD.MAXCNT = sizeof(tx_buf);
//    NRF_TWIM0->RXD.MAXCNT = sizeof(lis2dh12_list);
//    NRF_TWIM0->TXD.PTR = (uint32_t)&tx_buf[0];
//    NRF_TWIM0->RXD.PTR = (uint32_t)&lis2dh12_list[0];
//    NRF_TWIM0->TASKS_STARTTX = 1;
    for(int sample = 0; sample < LIS2DH12_I2C_BURST_LEN; sample++) {
        lis2dh12_list[sample].buffer[0] = i2c_read(LIS_REG_OUT_X_L_ADDR);
        lis2dh12_list[sample].buffer[1] = i2c_read(LIS_REG_OUT_X_H_ADDR);
        lis2dh12_list[sample].buffer[2] = i2c_read(LIS_REG_OUT_Y_L_ADDR);
        lis2dh12_list[sample].buffer[3] = i2c_read(LIS_REG_OUT_Y_H_ADDR);
        lis2dh12_list[sample].buffer[4] = i2c_read(LIS_REG_OUT_Z_L_ADDR);
        lis2dh12_list[sample].buffer[5] = i2c_read(LIS_REG_OUT_Z_H_ADDR);
    }

    NRF_UARTE0->TXD.PTR = (uint32_t)&lis2dh12_list[0];
    NRF_UARTE0->TXD.MAXCNT = sizeof(lis2dh12_list);
    NRF_UARTE0->TASKS_STARTTX = 1;
}

int main(void)
{
    volatile static uint8_t lis2dh12_read_buf[32];

    lis2dh12_ptr = 0;
    for(int i = 0; i < sizeof(lis2dh12_read_buf); i++) {
        lis2dh12_read_buf[i] = 0;
    }
    for(int i = 0; i < sizeof(lis2dh12_list)/6; i++) {
        for(int j = 0; j < sizeof(lis2dh12_list[0].buffer); j++) {
            lis2dh12_list[i].buffer[j] = 0;
        }
    }

    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
        ;
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

    NRF_SPIS0->ENABLE = SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos;
    NRF_SPI0->ENABLE  = SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos;
    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;
    NRF_TWIS0->ENABLE = TWIS_ENABLE_ENABLE_Disabled << TWIS_ENABLE_ENABLE_Pos;
    NRF_TWI0->ENABLE  = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
    NRF_TWI1->ENABLE  = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
    NRF_TWIM1->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;
    NRF_TWIS1->ENABLE = TWIS_ENABLE_ENABLE_Disabled << TWIS_ENABLE_ENABLE_Pos;
    NRF_SPI1->ENABLE  = SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos;
    NRF_SPIM1->ENABLE = SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos;
    NRF_SPIS1->ENABLE = SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos;
    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Disabled << UART_ENABLE_ENABLE_Pos;
    NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Disabled << UARTE_ENABLE_ENABLE_Pos;

    NRF_GPIO->PIN_CNF[PIN_UART_RX] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    NRF_GPIO->PIN_CNF[PIN_UART_TX] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    NRF_GPIO->OUTSET |= (1UL << PIN_UART_TX);

    NRF_GPIO->PIN_CNF[PIN_SDA_EXT] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    NRF_GPIO->PIN_CNF[PIN_SCL_EXT] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    
    NRF_GPIO->PIN_CNF[PIN_LIS_INT1] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
        (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
    NRF_GPIO->DETECTMODE = (1UL << PIN_LIS_INT1);
    NRF_GPIOTE->CONFIG[GPIOTE_INTENSET_IN0_Pos] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                                  (PIN_LIS_INT1 << GPIOTE_CONFIG_PSEL_Pos) |
                                                  (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);
    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos);

    NRF_UARTE0->PSEL.RTS = UARTE_PSEL_RTS_CONNECT_Disconnected << UARTE_PSEL_RTS_CONNECT_Pos;
    NRF_UARTE0->PSEL.CTS = UARTE_PSEL_CTS_CONNECT_Disconnected << UARTE_PSEL_CTS_CONNECT_Pos;
    NRF_UARTE0->PSEL.RXD = UARTE_PSEL_RXD_CONNECT_Disconnected << UARTE_PSEL_RXD_CONNECT_Pos;
    NRF_UARTE0->PSEL.TXD = PIN_UART_TX;
    NRF_UARTE0->INTENSET = (UARTE_INTENSET_ERROR_Enabled << UARTE_INTENSET_ERROR_Pos) |
                           (UARTE_INTENSET_TXDRDY_Enabled << UARTE_INTENSET_TXDRDY_Pos) |
                           (UARTE_INTENSET_ENDTX_Enabled << UARTE_INTENSET_ENDTX_Pos) |
                           (UARTE_INTENSET_TXSTARTED_Enabled << UARTE_INTENSET_TXSTARTED_Pos) |
                           (UARTE_INTENSET_RXSTARTED_Enabled << UARTE_INTENSET_RXSTARTED_Pos);
    NRF_UARTE0->BAUDRATE = UARTE_BAUDRATE_BAUDRATE_Baud115200 << UARTE_BAUDRATE_BAUDRATE_Pos;
    NRF_UARTE0->CONFIG = (UARTE_CONFIG_HWFC_Disabled << UARTE_CONFIG_HWFC_Pos) |
                         (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos);
    NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos;

    NRF_TWIM0->PSEL.SDA =
        (TWIM_PSEL_SDA_CONNECT_Connected << TWIM_PSEL_SDA_CONNECT_Pos) | (PIN_SDA_EXT << TWIM_PSEL_SDA_PIN_Pos);
    NRF_TWIM0->PSEL.SCL =
        (TWIM_PSEL_SCL_CONNECT_Connected << TWIM_PSEL_SCL_CONNECT_Pos) | (PIN_SCL_EXT << TWIM_PSEL_SCL_PIN_Pos);
    NRF_TWIM0->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K400 << TWIM_FREQUENCY_FREQUENCY_Pos;
    NRF_TWIM0->ADDRESS = LIS2DH12 << TWIM_ADDRESS_ADDRESS_Pos;
    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;

    for (int idx = 0; idx < sizeof(lis2dh12_init_seq); idx = idx + 2)
    {
        i2c_write(lis2dh12_init_seq[idx], lis2dh12_init_seq[idx + 1]);
    }
    for (int addr = 0; addr < sizeof(lis2dh12_init_seq) / 2; addr++)
    {
        lis2dh12_read_buf[addr] = i2c_read(lis2dh12_init_seq[addr * 2]);
    }
    i2c_write(LIS_REG_CTRL2_ADDR,
              (LIS_REG_CTRL2_FDS_En << LIS_REG_CTRL2_FDS_Pos));
    i2c_write(LIS_REG_CTRL3_ADDR,
              (LIS_REG_CTRL3_I1_OVERRUN_En << LIS_REG_CTRL3_I1_OVERRUN_Pos));
    i2c_write(LIS_REG_FIFO_CTRL_ADDR, (LIS_REG_FIFO_CTRL_FM_STREAM << LIS_REG_FIFO_CTRL_FM_Pos));
    i2c_write(LIS_REG_FIFO_SRC_ADDR,
                  (LIS_REG_FIFO_SRC_OVRN_FIFO_Enable << LIS_REG_FIFO_SRC_OVRN_FIFO_Pos));
    i2c_write(LIS_REG_CTRL4_ADDR, (LIS_REG_CTRL4_HR_12B << LIS_REG_CTRL4_HR_Pos) | (LIS_REG_CTRL4_FS_2G << LIS_REG_CTRL4_FS_Pos));
    i2c_write(LIS_REG_CTRL5_ADDR, LIS_REG_CTRL5_ADDR_FIFO_EN_Enable << LIS_REG_CTRL5_ADDR_FIFO_EN_Pos);
    i2c_write(LIS_REG_CTRL1_ADDR,
              (LIS_REG_CTRL1_HR_12B << LIS_REG_CTRL1_HR_Pos) | (LIS_CTRL1_ODR_5376HZ << LIS_CTRL1_ODR_FIELD_OFFSET) |
                  LIS_CTRL1_X_EN | LIS_CTRL1_Y_EN);
    lis2dh12_read_buf[0] = i2c_read(LIS_REG_CTRL3_ADDR);
    lis2dh12_read_buf[1] = i2c_read(LIS_REG_FIFO_CTRL_ADDR);
    lis2dh12_read_buf[2] = i2c_read(LIS_REG_FIFO_SRC_ADDR);
    lis2dh12_read_buf[3] = i2c_read(LIS_REG_CTRL4_ADDR);
    lis2dh12_read_buf[4] = i2c_read(LIS_REG_CTRL5_ADDR);
    lis2dh12_read_buf[5] = i2c_read(LIS_REG_CTRL1_ADDR); 

    NRF_TWIM0->INTENSET = (TWIM_INTENSET_TXSTARTED_Enabled << TWIM_INTENSET_TXSTARTED_Pos) | 
        (TWIM_INTENSET_RXSTARTED_Enabled << TWIM_INTENSET_RXSTARTED_Pos) | 
        (TWIM_INTENSET_LASTTX_Enabled << TWIM_INTENSET_LASTTX_Pos) |
        (TWIM_INTENSET_LASTRX_Enabled << TWIM_INTENSET_LASTRX_Pos);
    NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
    NVIC_EnableIRQ(GPIOTE_IRQn);
    NVIC_EnableIRQ(UARTE0_UART0_IRQn);

    while (1) {
        __WFI();
    }
    return 0;
}

void GPIOTE_IRQHandler(void)
{
    if (NRF_GPIOTE->EVENTS_IN[GPIOTE_INTENSET_IN0_Pos])
    {
        NRF_GPIOTE->EVENTS_IN[GPIOTE_INTENSET_IN0_Pos] = 0;
        NRF_GPIOTE->INTENCLR = (GPIOTE_INTENCLR_IN0_Enabled << GPIOTE_INTENCLR_IN0_Pos);
        lis2dh12_getSamples();
    }
}

void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void)
{
    if (NRF_TWIM0->EVENTS_TXSTARTED)
    {
        NRF_TWIM0->EVENTS_TXSTARTED = 0;
    }
    else if (NRF_TWIM0->EVENTS_RXSTARTED)
    {
        NRF_TWIM0->EVENTS_RXSTARTED = 0;
    }
    else if (NRF_TWIM0->EVENTS_LASTTX)
    {
        NRF_TWIM0->EVENTS_LASTTX = 0;
    }
    else if (NRF_TWIM0->EVENTS_LASTRX)
    {
        NRF_TWIM0->EVENTS_LASTRX = 0;
    }
    else if (NRF_TWIM0->EVENTS_STOPPED)
    {
        NRF_TWIM0->EVENTS_STOPPED = 0;
        NRF_TWIM0->INTENCLR = TWIM_INTENCLR_STOPPED_Enabled << TWIM_INTENCLR_STOPPED_Pos;
        NRF_GPIOTE->INTENSET |= GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos;
    }
}

void UARTE0_UART0_IRQHandler(void)
{
    if (NRF_UARTE0->EVENTS_ERROR)
    {
        NRF_UARTE0->EVENTS_ERROR = 0;
    }
    if (NRF_UARTE0->EVENTS_TXDRDY)
    {
        NRF_UARTE0->EVENTS_TXDRDY = 0;
    }
    if (NRF_UARTE0->EVENTS_ENDTX)
    {
        NRF_GPIOTE->INTENSET |= GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos;
        NRF_UARTE0->EVENTS_ENDTX = 0;
        NRF_UARTE0->TASKS_STOPTX = 1;
    }
    if (NRF_UARTE0->EVENTS_TXSTARTED)
    {
        NRF_UARTE0->EVENTS_TXSTARTED = 0;
    }
    if (NRF_UARTE0->EVENTS_RXSTARTED)
    {
        NRF_UARTE0->EVENTS_RXSTARTED = 0;
    }
}
