#define LIS2DH12_I2C_ADDRESS 0x30	// Dev board is 0x32, production board is 0x30
#define LIS2DH12_REG_ADDR_SIZE 1
#define LIS2DH12_REG_COUNT 34

#define LIS_REG_STATUS_AUX_ADDR                                    0x07 /* n/a */
#define LIS_REG_OUT_TEMP_ADDR                                      0x0C /* Temperature sensor data */
#define LIS_REG_WHO_AM_I_ADDR                                      0x0F /* Device identification register */
#define LIS_REG_CTRL0_ADDR                                         0x1E /* Control Register 0 */
#define LIS_REG_TEMP_CFG_ADDR                                      0x1F /* n/a */
#define LIS_REG_CTRL1_ADDR                                         0x20 /* Control Register 1 */
#define LIS_REG_CTRL2_ADDR                                         0x21 /* Control Register 2 */
#define LIS_REG_CTRL3_ADDR                                         0x22 /* Control Register 3 */
#define LIS_REG_CTRL4_ADDR                                         0x23 /* Control Register 4 */
#define LIS_REG_CTRL5_ADDR                                         0x24 /* Control Register 5 */
#define LIS_REG_CTRL6_ADDR                                         0x25 /* Control Register 6 */
#define LIS_REG_REFERENCE_ADDR                                     0x26 /* Reference value for interrupt generation */
#define LIS_REG_STATUS_ADDR                                        0x27 /* n/a */
#define LIS_REG_OUT_X_L_ADDR                                       0x28 /* X-axis acceleration data */
#define LIS_REG_OUT_X_H_ADDR                                       0x29 /* X-axis acceleration data */
#define LIS_REG_OUT_Y_L_ADDR                                       0x2A /* Y-axis acceleration data */
#define LIS_REG_OUT_Y_H_ADDR                                       0x2B /* Y-axis acceleration data */
#define LIS_REG_OUT_Z_L_ADDR                                       0x2C /* Z-axis acceleration data */
#define LIS_REG_OUT_Z_H_ADDR                                       0x2D /* Z-axis acceleration data */
#define LIS_REG_FIFO_CTRL_ADDR                                     0x2E /* Fifo Control register */
#define LIS_REG_FIFO_SRC_ADDR                                      0x2F /* Fifo status register */
#define LIS_REG_INT1_CFG_ADDR                                      0x30 /* Interrupt 1 config register */
#define LIS_REG_INT1_SRC_ADDR                                      0x31 /* Interrupt 1 source register */
#define LIS_REG_INT1_THS_ADDR                                      0x32 /* Interrupt 1 threshold register */
#define LIS_REG_INT1_DURATION_ADDR                                 0x33 /* Interrupt 1 duration register */
#define LIS_REG_INT2_CFG_ADDR                                      0x34 /* Interrupt 2 config register */
#define LIS_REG_INT2_SRC_ADDR                                      0x35 /* Interrupt 2 source register */
#define LIS_REG_INT2_THS_ADDR                                      0x36 /* Interrupt 2 threshold register */
#define LIS_REG_INT2_DURATION_ADDR                                 0x37 /* Interrupt 2 duration register */
#define LIS_REG_CLICK_CFG_ADDR                                     0x38 /* Click config */
#define LIS_REG_CLICK_SRC_ADDR                                     0x39 /* Click source */
#define LIS_REG_CLICK_THS_ADDR                                     0x3A /* Click Threshold */
#define LIS_REG_TIME_LIMIT_ADDR                                    0x3B /* Click time limit */
#define LIS_REG_TIME_LATENCY_ADDR                                  0x3C /* Click time latency */
#define LIS_REG_TIME_WINDOW_ADDR                                   0x3D /* Click time window */
#define LIS_REG_ACT_THS_ADDR                                       0x3E /* Activity threshold */
#define LIS_REG_ACT_DUR_ADDR                                       0x3F /* Activity duration */


#define LIS_REG_STATUS_ZDA_Pos 2UL
#define LIS_REG_STATUS_YDA_Pos 1UL
#define LIS_REG_STATUS_XDA_Pos 0UL

#define LIS_REG_FIFO_CTRL_FM_Pos         (6UL)
#define LIS_REG_FIFO_CTRL_FM_BYPASS      0b00
#define LIS_REG_FIFO_CTRL_FM_FIFO        0b01
#define LIS_REG_FIFO_CTRL_FM_STREAM      0b10
#define LIS_REG_FIFO_CTRL_FM_STREAM2FIFO 0b11
#define LIS_REG_FIFO_CTRL_TR_Pos         (6UL)
#define LIS_REG_FIFO_CTRL_INT1           (0UL)
#define LIS_REG_FIFO_CTRL_FTH_Pos        (0UL)

#define LIS_REG_FIFO_SRC_WTM_Pos          (7UL)
#define LIS_REG_FIFO_SRC_WTM_Enable       (1UL)
#define LIS_REG_FIFO_SRC_OVRN_FIFO_Pos    (6UL)
#define LIS_REG_FIFO_SRC_OVRN_FIFO_Enable (1UL)
#define LIS_REG_FIFO_SRC_EMPTY_Pos        (5UL)
#define LIS_REG_FIFO_SRC_EMPTY_Enable     (1UL)

#define LIS_REG_CTRL5_ADDR_FIFO_EN_Disable (0UL)
#define LIS_REG_CTRL5_ADDR_FIFO_EN_Enable  (1UL)
#define LIS_REG_CTRL5_ADDR_FIFO_EN_Pos     (6UL)

#define LIS_REG_CTRL3_I1_ZYXDA_Pos   (4UL)
#define LIS_REG_CTRL3_I1_ZYXDA_En    (1UL) 
#define LIS_REG_CTRL3_I1_WTM_Pos     (2UL)
#define LIS_REG_CTRL3_I1_WTM_En      (1UL) 
#define LIS_REG_CTRL3_I1_OVERRUN_Pos (1UL)
#define LIS_REG_CTRL3_I1_OVERRUN_En  (1UL) 

#define LIS_REG_CTRL1_HR_Pos (0UL)
#define LIS_REG_CTRL1_HR_12B (1UL)
#define LIS_REG_CTRL4_HR_Pos (3UL)
#define LIS_REG_CTRL4_HR_12B (1UL)


/* WHO_AM_I Register Fields */
    /* WHO_AM_I -> WHO_AM_I */
    #define LIS_WHO_AM_I_WHO_AM_I_FIELD_MASK                       0xFF /* Device identification register */
    #define LIS_WHO_AM_I_WHO_AM_I_FIELD_OFFSET                     0x00

/* CTRL0 Register Fields */
    /* CTRL0 -> CTRL0 */
    #define LIS_CTRL0_CTRL0_FIELD_MASK                             0xFF /* Control Register 0 */
    #define LIS_CTRL0_CTRL0_FIELD_OFFSET                           0x00

/* TEMP_CFG Register Fields */
    /* TEMP_CFG -> TEMP_CFG */
    #define LIS_TEMP_CFG_TEMP_CFG_FIELD_MASK                       0xFF /* n/a */
    #define LIS_TEMP_CFG_TEMP_CFG_FIELD_OFFSET                     0x00

/* CTRL1 Register Fields */
    /* CTRL1 -> Flags */
    #define LIS_CTRL1_X_EN                                         0x01 /* X-axis enable */
    #define LIS_CTRL1_Y_EN                                         0x02 /* Y-axis enable */
    #define LIS_CTRL1_Z_EN                                         0x04 /* Z-axis enable */
    #define LIS_CTRL1_LOW_PWR                                      0x08 /* Low-power mode enable */
    /* CTRL1 -> ODR */
    #define LIS_CTRL1_ODR_FIELD_MASK                               0xF0 /* Data rate selection */
    #define LIS_CTRL1_ODR_FIELD_OFFSET                             (4UL) 
      #define LIS_CTRL1_ODR_PWR_DWN                                0x00 /* Power-down mode */
      #define LIS_CTRL1_ODR_1HZ                                    0x01 /* HR/ Normal / Low-power mode (1 Hz) */
      #define LIS_CTRL1_ODR_10HZ                                   0x08 /* HR/ Normal / Low-power mode (10 Hz) */
      #define LIS_CTRL1_ODR_25HZ                                   0x09 /* HR/ Normal / Low-power mode (25 Hz) */
      #define LIS_CTRL1_ODR_50HZ                                   0x40 /* HR/ Normal / Low-power mode (50 Hz) */
      #define LIS_CTRL1_ODR_100HZ                                  0x41 /* HR/ Normal / Low-power mode (100 Hz) */
      #define LIS_CTRL1_ODR_200HZ                                  0x48 /* HR/ Normal / Low-power mode (200 Hz) */
      #define LIS_CTRL1_ODR_400HZ                                  0x49 /* HR/ Normal / Low-power mode (400 Hz) */
      #define LIS_CTRL1_ODR_1620HZ                                 0x07 /* Low-power mode (1.620 kHz) */
      #define LIS_CTRL1_ODR_5376HZ                                 0x09 /* HR/ Normal (1.344 kHz) / Low-power mode (5.376 kHz) */

/* CTRL2 Register Fields */
    /* CTRL2 -> Flags */
    #define LIS_CTRL2_HP_IA1                                       0x01 /* High-pass filter enabled for AOI function on Interrupt 1 */
    #define LIS_CTRL2_HP_IA2                                       0x02 /* High-pass filter enabled for AOI function on Interrupt 2 */
    #define LIS_CTRL2_HPCLICK                                      0x04 /* High-pass filter enabled for Click function */
    #define LIS_CTRL2_FDS                                          0x08 /* Filtered data selection */

/* CTRL3 Register Fields */
    /* CTRL3 -> CTRL3 */
    #define LIS_CTRL3_CTRL3_FIELD_MASK                             0xFF /* Control Register 3 */
    #define LIS_CTRL3_CTRL3_FIELD_OFFSET                           0x00

/* CTRL4 Register Fields */
    /* CTRL4 -> CTRL4 */
    #define LIS_CTRL4_CTRL4_FIELD_MASK                             0xFF /* Control Register 4 */
    #define LIS_CTRL4_CTRL4_FIELD_OFFSET                           0x00

/* CTRL5 Register Fields */
    /* CTRL5 -> CTRL5 */
    #define LIS_CTRL5_CTRL5_FIELD_MASK                             0xFF /* Control Register 5 */
    #define LIS_CTRL5_CTRL5_FIELD_OFFSET                           0x00

/* CTRL6 Register Fields */
    /* CTRL6 -> CTRL6 */
    #define LIS_CTRL6_CTRL6_FIELD_MASK                             0xFF /* Control Register 6 */
    #define LIS_CTRL6_CTRL6_FIELD_OFFSET                           0x00

/* REFERENCE Register Fields */
    /* REFERENCE -> REFERENCE */
    #define LIS_REFERENCE_REFERENCE_FIELD_MASK                     0xFF /* Reference value for interrupt generation */
    #define LIS_REFERENCE_REFERENCE_FIELD_OFFSET                   0x00

/* FIFO_CTRL Register Fields */
    /* FIFO_CTRL -> FIFO_CTRL */
    #define LIS_FIFO_CTRL_FIFO_CTRL_FIELD_MASK                     0xFF /* Fifo Control register */
    #define LIS_FIFO_CTRL_FIFO_CTRL_FIELD_OFFSET                   0x00

/* INT1_CFG Register Fields */
    /* INT1_CFG -> INT1_CFG */
    #define LIS_INT1_CFG_INT1_CFG_FIELD_MASK                       0xFF /* Interrupt 1 config register */
    #define LIS_INT1_CFG_INT1_CFG_FIELD_OFFSET                     0x00

/* INT1_THS Register Fields */
    /* INT1_THS -> INT1_THS */
    #define LIS_INT1_THS_INT1_THS_FIELD_MASK                       0xFF /* Interrupt 1 threshold register */
    #define LIS_INT1_THS_INT1_THS_FIELD_OFFSET                     0x00

/* INT1_DURATION Register Fields */
    /* INT1_DURATION -> INT1_DURATION */
    #define LIS_INT1_DURATION_INT1_DURATION_FIELD_MASK             0xFF /* Interrupt 1 duration register */
    #define LIS_INT1_DURATION_INT1_DURATION_FIELD_OFFSET           0x00

/* INT2_CFG Register Fields */
    /* INT2_CFG -> INT2_CFG */
    #define LIS_INT2_CFG_INT2_CFG_FIELD_MASK                       0xFF /* Interrupt 2 config register */
    #define LIS_INT2_CFG_INT2_CFG_FIELD_OFFSET                     0x00

/* INT2_THS Register Fields */
    /* INT2_THS -> INT2_THS */
    #define LIS_INT2_THS_INT2_THS_FIELD_MASK                       0xFF /* Interrupt 2 threshold register */
    #define LIS_INT2_THS_INT2_THS_FIELD_OFFSET                     0x00

/* INT2_DURATION Register Fields */
    /* INT2_DURATION -> INT2_DURATION */
    #define LIS_INT2_DURATION_INT2_DURATION_FIELD_MASK             0xFF /* Interrupt 2 duration register */
    #define LIS_INT2_DURATION_INT2_DURATION_FIELD_OFFSET           0x00

/* CLICK_CFG Register Fields */
    /* CLICK_CFG -> CLICK_CFG */
    #define LIS_CLICK_CFG_CLICK_CFG_FIELD_MASK                     0xFF /* Click config */
    #define LIS_CLICK_CFG_CLICK_CFG_FIELD_OFFSET                   0x00

/* CLICK_THS Register Fields */
    /* CLICK_THS -> CLICK_THS */
    #define LIS_CLICK_THS_CLICK_THS_FIELD_MASK                     0xFF /* Click Threshold */
    #define LIS_CLICK_THS_CLICK_THS_FIELD_OFFSET                   0x00

/* TIME_LIMIT Register Fields */
    /* TIME_LIMIT -> TIME_LIMIT */
    #define LIS_TIME_LIMIT_TIME_LIMIT_FIELD_MASK                   0xFF /* Click time limit */
    #define LIS_TIME_LIMIT_TIME_LIMIT_FIELD_OFFSET                 0x00

/* TIME_LATENCY Register Fields */
    /* TIME_LATENCY -> TIME_LATENCY */
    #define LIS_TIME_LATENCY_TIME_LATENCY_FIELD_MASK               0xFF /* Click time latency */
    #define LIS_TIME_LATENCY_TIME_LATENCY_FIELD_OFFSET             0x00

/* TIME_WINDOW Register Fields */
    /* TIME_WINDOW -> TIME_WINDOW */
    #define LIS_TIME_WINDOW_TIME_WINDOW_FIELD_MASK                 0xFF /* Click time window */
    #define LIS_TIME_WINDOW_TIME_WINDOW_FIELD_OFFSET               0x00

/* ACT_THS Register Fields */
    /* ACT_THS -> ACT_THS */
    #define LIS_ACT_THS_ACT_THS_FIELD_MASK                         0xFF /* Activity threshold */
    #define LIS_ACT_THS_ACT_THS_FIELD_OFFSET                       0x00

/* ACT_DUR Register Fields */
    /* ACT_DUR -> ACT_DUR */
    #define LIS_ACT_DUR_ACT_DUR_FIELD_MASK                         0xFF /* Activity duration */
    #define LIS_ACT_DUR_ACT_DUR_FIELD_OFFSET                       0x00


#define LIS_WHO_AM_I_DEFAULT                                       0x33
#define LIS_CTRL0_DEFAULT                                          0x10
#define LIS_TEMP_CFG_DEFAULT                                       0x07
#define LIS_CTRL1_DEFAULT                                          0x07
#define LIS_CTRL2_DEFAULT                                          0x00
#define LIS_CTRL3_DEFAULT                                          0x00
#define LIS_CTRL4_DEFAULT                                          0x00
#define LIS_CTRL5_DEFAULT                                          0x00
#define LIS_CTRL6_DEFAULT                                          0x00
#define LIS_REFERENCE_DEFAULT                                      0x00
#define LIS_FIFO_CTRL_DEFAULT                                      0x00
#define LIS_INT1_CFG_DEFAULT                                       0x00
#define LIS_INT1_THS_DEFAULT                                       0x00
#define LIS_INT1_DURATION_DEFAULT                                  0x00
#define LIS_INT2_CFG_DEFAULT                                       0x00
#define LIS_INT2_THS_DEFAULT                                       0x00
#define LIS_INT2_DURATION_DEFAULT                                  0x00
#define LIS_CLICK_CFG_DEFAULT                                      0x00
#define LIS_CLICK_THS_DEFAULT                                      0x00
#define LIS_TIME_LIMIT_DEFAULT                                     0x00
#define LIS_TIME_LATENCY_DEFAULT                                   0x00
#define LIS_TIME_WINDOW_DEFAULT                                    0x00
#define LIS_ACT_THS_DEFAULT                                        0x00
#define LIS_ACT_DUR_DEFAULT                                        0x00
