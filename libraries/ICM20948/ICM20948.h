#ifndef ICM20948_h
#define ICM20948_h
#include <ICM20948_DMP.h>

#include <Arduino.h>

#ifdef I2C
  #include <Wire.h> // I2C
#endif

//ICM20948 SPI
//   Data is delivered MSB first and LSB last => SPISettings.dataOrder = MSBFIRST
#define SPISETTINGS_DATAORDER       MSBFIRST  
 
//   Data is latched on the rising edge of SCLK
//   Data should be transitioned on the falling edge of SCLK => SPISettings.dataMode = SPI_MODE0
//            Mode	    Clock Polarity (CPOL)	Clock Phase (CPHA)	    Output Edge	        Data Capture
//            SPI_MODE0	        0	                         0	            Falling	            Rising
#define SPISETTINGS_DATAMODE       SPI_MODE0
   
//   The maximum frequency of SCLK is 7MHz => SPISettings.speedMaximum = 7000000 (7MHZ)
#define SPI_SPEEDMAXIMUM  7000000   
 
//ICM20948 I2C
#define ADDRESS_DEFAULT 0X69 //AD0 logic level set to high

#define WIRE_TRANSMIT_SUCESS          0x00 // Wire.endTransmission()- 0:success
#define WIRE_ERROR_TRANSMIT_TOOLONG   0x01 // Wire.endTransmission()- 1:data too long to fit in transmit buffer
#define WIRE_ERROR_TRANSMIT_ADR_NACK  0x02 // Wire.endTransmission()- 2:received NACK on transmit of address
#define WIRE_ERROR_TRANSMIT_DATA_NACK 0x03 // Wire.endTransmission()- 3:received NACK on transmit of data
#define WIRE_TRANSMIT_ERROR_OTHER     0x04 // Wire.endTransmission()- 4:other error
#define WIRE_REQUEST_ERROR            0x80 // Wire.requestFrom()- the number of bytes returned from the slave device != the number of bytes to request

//ICM20948 Errors
#define CHECK_DEVICE_ERROR            0x01
#define CHECK_STARTUP_ERROR           0x02

#define ICM20948_ID 0xEA                     // The value for ICM-20948 is 0xEA.

#define AK09916_I2C_ADDR 0x0C                // The slave address of AK09916 is 0Ch. The 8th bit (least significant bit) of the first byte is a R/W bit
                                             // When the R/W bit is set to “1”, READ instruction is executed. When the R/W bit is set to “0”, WRITE instruction is executed

#define AK09916_WHO_AM_I 0x09                // The value for ICM-20948 is 0x09.
#define MAX_MAGNETOMETER_STARTS 10 

/* Max size that can be read across SPI data lines */
#define INV_MAX_SERIAL_READ 16
/* Max size that can be written across SPI data lines */
#define INV_MAX_SERIAL_WRITE 16

/*****************************************************************************/
/* ICM20948 register banks                                                   */
/*****************************************************************************/
#define ICM20948_BANK_0                     (0 << 8)                    /**< Register bank 0    */
#define ICM20948_BANK_1                     (1 << 8)                    /**< Register bank 1    */
#define ICM20948_BANK_2                     (2 << 8)                    /**< Register bank 2    */
#define ICM20948_BANK_3                     (3 << 8)                    /**< Register bank 3    */

#define ICM20948_REG_WHO_AM_I               (ICM20948_BANK_0 | 0x00)    /**< Device ID register                                     */

#define ICM20948_REG_USER_CTRL              (ICM20948_BANK_0 | 0x03)    /**< User control register                                  */

#define ICM20948_BIT_DMP_EN                 0x80                        /**< DMP enable bit                                         */
#define ICM20948_BIT_FIFO_EN                0x40                        /**< FIFO enable bit                                        */
#define ICM20948_BIT_I2C_MST_EN             0x20                        /**< I2C master I/F enable bit                              */
#define ICM20948_BIT_I2C_IF_DIS             0x10                        /**< Disable I2C, enable SPI bit                            */
#define ICM20948_BIT_DMP_RST                0x08                        /**< DMP module reset bit                                   */
#define ICM20948_BIT_SRAM_RST               0x04                        /**< SRAM module reset bit                                  */
#define ICM20948_BIT_I2C_MST_RST            0x02                        /**< I2C Master reset bit                                   */

#define REG_USER_CTRL_VAL_S1                0x00                        /**< Value of the register at the end the step 1            */

#define ICM20948_REG_LP_CONFIG              (ICM20948_BANK_0 | 0x05)    /**< Low Power mode config register                         */
#define ICM20948_BIT_I2C_MST_CYCLE          0x40                        /**< I2C master cycle mode enable                           */
#define ICM20948_BIT_ACCEL_CYCLE            0x20                        /**< Accelerometer cycle mode enable bit                    */
#define ICM20948_BIT_GYRO_CYCLE             0x10                        /**< Gyroscope cycle mode enable bit                        */

#define REG_LP_CONFIG_VAL_S1                0x40                        /**< Value of the register at the end the step 1            */

#define ICM20948_REG_PWR_MGMT_1             (ICM20948_BANK_0 | 0x06)    /**< Power Management 1 register                            */
#define ICM20948_BIT_RESET                  0x80                        /**< Device reset bit                                       */
#define ICM20948_BIT_SLEEP                  0x40                        /**< Sleep mode enable bit                                  */
#define ICM20948_BIT_LP_EN                  0x20                        /**< Low Power feature enable bit                           */
#define ICM20948_BIT_TEMP_DIS               0x08                        /**< Temperature sensor disable bit                         */
#define ICM20948_BITS_CLKSEL                0x07                        /**< Clock Source 3 bits                                    */

#define ICM20948_CLOCK_AUTO                 0x01                        /**< Auto selects the best available clock source           */
#define REG_PWR_MGMT_1_VAL_STARTUP          0x01                        /**< Value of the register at the end the startup           */
#define REG_PWR_MGMT_1_VAL_S1               0x01                        /**< Value of the register at the end the step 1            */

#define ICM20948_REG_PWR_MGMT_2             (ICM20948_BANK_0 | 0x07)    /**< Power Management 2 register                            */
#define ICM20948_BITS_DISABLE_ACCEL         0x38                        /**< Disable accelerometer 3 bits                           */
#define ICM20948_BITS_DISABLE_GYRO          0x07                        /**< Disable gyroscope 3 bits                               */
#define REG_PWR_MGMT_2_VAL_S1               0x00                        /**< Value of the register at the end the step 1            */

#define ICM20948_REG_INT_PIN_CFG            (ICM20948_BANK_0 | 0x0F)    /**< Interrupt Pin Configuration register                   */
#define ICM20948_BIT_INT_ACTL               0x80                        /**< Active low setting bit                                 */
#define ICM20948_BIT_INT_OPEN               0x40                        /**< Open collector configuration bit                       */
#define ICM20948_BIT_INT_LATCH_EN           0x20                        /**< Latch enable bit                                       */
#define ICM20948_BIT_BYPASS_EN              0x02                        /**< Bypass enable bit                                      */

#define ICM20948_REG_INT_ENABLE             (ICM20948_BANK_0 | 0x10)    /**< Interrupt Enable register                              */
#define ICM20948_BIT_WOM_INT_EN             0x08                        /**< Wake-up On Motion enable bit                           */

#define ICM20948_REG_INT_ENABLE_1           (ICM20948_BANK_0 | 0x11)    /**< Interrupt Enable 1 register                            */
#define ICM20948_BIT_RAW_DATA_0_RDY_EN      0x01                        /**< Raw data ready interrupt enable bit                    */

#define REG_INT_ENABLE_1_VAL_S1             0x00                        /**< Value of the register at the end the step 1            */
#define REG_INT_ENABLE_1_VAL_STARTUP        0x00                        /**< Value of the register at the startup                   */

#define ICM20948_REG_INT_ENABLE_2           (ICM20948_BANK_0 | 0x12)    /**< Interrupt Enable 2 register                            */
#define ICM20948_BITS_FIFO_OVERFLOW_EN      0x1F                        /**< FIFO overflow interrupt enable bits                    */

#define REG_INT_ENABLE_2_VAL_S1             0x0F                        /**< Value of the register at the end the step 1            */
#define REG_INT_ENABLE_2_VAL_STARTUP        0x00                        /**< Value of the register at the startup                   */

#define ICM20948_REG_INT_ENABLE_3           (ICM20948_BANK_0 | 0x13)    /**< Interrupt Enable 2 register                            */

#define ICM20948_REG_I2C_MST_STATUS         (ICM20948_BANK_0 | 0x17)
#define ICM20948_BIT_I2C_SLV4_DONE          0x40                        /**< SLV4_DONE_INT_EN bit                                   */

#define ICM20948_REG_INT_STATUS             (ICM20948_BANK_0 | 0x19)    /**< Interrupt Status register                              */
#define ICM20948_BIT_WOM_INT                0x08                        /**< Wake-up on motion interrupt bit                        */
#define ICM20948_BIT_PLL_RDY                0x04                        /**< PLL ready interrupt bit                                */

#define ICM20948_REG_INT_STATUS_1           (ICM20948_BANK_0 | 0x1A)    /**< Interrupt Status 1 register                            */
#define ICM20948_BIT_RAW_DATA_0_RDY_INT     0x01                        /**< Raw data ready interrupt bit                           */

#define ICM20948_REG_INT_STATUS_2           (ICM20948_BANK_0 | 0x1B)    /**< Interrupt Status 2 register                            */

#define ICM20948_REG_INT_STATUS_3           (ICM20948_BANK_0 | 0x1C)    /**< Interrupt Status 3 register                            */

#define DMP_REG_SINGLE_FIFO_PRIORITY_SEL   (ICM20948_BANK_0 | 0x26)    /* Single FIFO Priority Select register                      */

#define ICM20948_REG_DELAY_TIMEH           (ICM20948_BANK_0 | 0x28)    /**< Interrupt Time High byte register                       */
#define ICM20948_REG_DELAY_TIMEL           (ICM20948_BANK_0 | 0x29)    /**< Interrupt Time Low bytes register                       */

#define ICM20948_REG_ACCEL_XOUT_H_SH        (ICM20948_BANK_0 | 0x2D)    /**< Accelerometer X-axis data high byte                    */
#define ICM20948_REG_ACCEL_XOUT_L_SH        (ICM20948_BANK_0 | 0x2E)    /**< Accelerometer X-axis data low byte                     */
#define ICM20948_REG_ACCEL_YOUT_H_SH        (ICM20948_BANK_0 | 0x2F)    /**< Accelerometer Y-axis data high byte                    */
#define ICM20948_REG_ACCEL_YOUT_L_SH        (ICM20948_BANK_0 | 0x30)    /**< Accelerometer Y-axis data low byte                     */
#define ICM20948_REG_ACCEL_ZOUT_H_SH        (ICM20948_BANK_0 | 0x31)    /**< Accelerometer Z-axis data high byte                    */
#define ICM20948_REG_ACCEL_ZOUT_L_SH        (ICM20948_BANK_0 | 0x32)    /**< Accelerometer Z-axis data low byte                     */

#define ICM20948_REG_GYRO_XOUT_H_SH         (ICM20948_BANK_0 | 0x33)    /**< Gyroscope X-axis data high byte                        */
#define ICM20948_REG_GYRO_XOUT_L_SH         (ICM20948_BANK_0 | 0x34)    /**< Gyroscope X-axis data low byte                         */
#define ICM20948_REG_GYRO_YOUT_H_SH         (ICM20948_BANK_0 | 0x35)    /**< Gyroscope Y-axis data high byte                        */
#define ICM20948_REG_GYRO_YOUT_L_SH         (ICM20948_BANK_0 | 0x36)    /**< Gyroscope Y-axis data low byte                         */
#define ICM20948_REG_GYRO_ZOUT_H_SH         (ICM20948_BANK_0 | 0x37)    /**< Gyroscope Z-axis data high byte                        */
#define ICM20948_REG_GYRO_ZOUT_L_SH         (ICM20948_BANK_0 | 0x38)    /**< Gyroscope Z-axis data low byte                         */

#define ICM20948_REG_TEMPERATURE_H          (ICM20948_BANK_0 | 0x39)    /**< Temperature data high byte                             */
#define ICM20948_REG_TEMPERATURE_L          (ICM20948_BANK_0 | 0x3A)    /**< Temperature data low byte                              */

#define ICM20948_REG_FIFO_EN_1              (ICM20948_BANK_0 | 0x66)    /**< FIFO Enable 1 register                                 */
#define ICM20948_BIT_SLV_3_FIFO_EN          0x08                        /**< Write EXT_SENS_DATA registers associated to SLV_3 bit  */
#define ICM20948_BIT_SLV_2_FIFO_EN          0x04                        /**< Write EXT_SENS_DATA registers associated to SLV_2 bit  */
#define ICM20948_BIT_SLV_1_FIFO_EN          0x02                        /**< Write EXT_SENS_DATA registers associated to SLV_1 bit  */
#define ICM20948_BIT_SLV_0_FIFO_EN          0x01                        /**< Write EXT_SENS_DATA registers associated to SLV_0 bit  */

#define REG_FIFO_EN_1_VAL_S1                0x00                        /**< Value of the register at the end the step 1            */
#define REG_FIFO_EN_1_VAL_STARTUP           0x00                        /**< Value of the register at the startup                   */

#define ICM20948_REG_FIFO_EN_2              (ICM20948_BANK_0 | 0x67)    /**< FIFO Enable 2 register                                 */
#define ICM20948_BIT_ACCEL_FIFO_EN          0x10                        /**< Enable writing acceleration data to FIFO bit           */
#define ICM20948_BITS_GYRO_FIFO_EN          0x0E                        /**< Enable writing gyroscope data to FIFO bitd             */
#define ICM20948_BITS_TEMP_FIFO_EN          0x01                        /**< Enable writing temprature data to FIFO bit             */

#define REG_FIFO_EN_2_VAL_S1                0x00                        /**< Value of the register at the end the step 1            */
#define REG_FIFO_EN_2_VAL_STARTUP           0x00                        /**< Value of the register at the startup                   */

#define ICM20948_REG_FIFO_RST               (ICM20948_BANK_0 | 0x68)    /**< FIFO Reset register                                    */
#define ICM20948_BITS_FIFO_RESET            0x1F                        /**< FIFO reset bits                                        */

#define REG_FIFO_RST _VAL_S1                0x0F                        /**< Value of the register at the end the step 1            */

#define ICM20948_REG_FIFO_MODE              (ICM20948_BANK_0 | 0x69)    /**< FIFO Mode register                                     */
#define ICM20948_BITS_FIFO_MODE             0x1F                        /**< FIFO Mode bits                                         */

#define ICM20948_REG_FIFO_COUNT_H           (ICM20948_BANK_0 | 0x70)    /**< FIFO data count high byte                              */
#define ICM20948_REG_FIFO_COUNT_L           (ICM20948_BANK_0 | 0x71)    /**< FIFO data count low byte                               */
#define ICM20948_REG_FIFO_R_W               (ICM20948_BANK_0 | 0x72)    /**< FIFO Read/Write register                               */

#define ICM20948_REG_DATA_RDY_STATUS        (ICM20948_BANK_0 | 0x74)    /**< Data Ready Status register                             */
#define ICM20948_BIT_RAW_DATA_0_RDY         0x01                        /**< Raw Data Ready bit                                     */

#define DMP_REG_HW_FIX_DISABLE              (ICM20948_BANK_0 | 0x75)    /**< Hardware Fix Disable register                          */

#define ICM20948_REG_FIFO_CFG               (ICM20948_BANK_0 | 0x76)    /**< FIFO Configuration register                            */
#define ICM20948_BIT_MULTI_FIFO_CFG         0x01                        /**< Interrupt status for each sensor is required           */
#define ICM20948_BIT_SINGLE_FIFO_CFG        0x00                        /**< Interrupt status for only a single sensor is required  */

#define DMP_REG_MEM_START_ADDR             (ICM20948_BANK_0 |  0x7C)    // Hmm, Invensense thought they were sneaky not listing these locations on the datasheet...
#define DMP_REG_MEM_R_W                    (ICM20948_BANK_0 |  0x7D)    // These three locations seem to be able to access some memory within the device
#define DMP_REG_MEM_BANK_SEL               (ICM20948_BANK_0 |  0x7E)    // And that location is also where the DMP image gets loaded


/***********************/
/* Bank 1 register map */
/***********************/
#define ICM20948_REG_XA_OFFSET_H            (ICM20948_BANK_1 | 0x14)    /**< Acceleration sensor X-axis offset cancellation high byte   */
#define ICM20948_REG_XA_OFFSET_L            (ICM20948_BANK_1 | 0x15)    /**< Acceleration sensor X-axis offset cancellation low byte    */
#define ICM20948_REG_YA_OFFSET_H            (ICM20948_BANK_1 | 0x17)    /**< Acceleration sensor Y-axis offset cancellation high byte   */
#define ICM20948_REG_YA_OFFSET_L            (ICM20948_BANK_1 | 0x18)    /**< Acceleration sensor Y-axis offset cancellation low byte    */
#define ICM20948_REG_ZA_OFFSET_H            (ICM20948_BANK_1 | 0x1A)    /**< Acceleration sensor Z-axis offset cancellation high byte   */
#define ICM20948_REG_ZA_OFFSET_L            (ICM20948_BANK_1 | 0x1B)    /**< Acceleration sensor Z-axis offset cancellation low byte    */

#define ICM20948_REG_TIMEBASE_CORR_PLL      (ICM20948_BANK_1 | 0x28)    /**< PLL Timebase Correction register                           */

/***********************/
/* Bank 2 register map */
/***********************/
#define ICM20948_REG_GYRO_SMPLRT_DIV        (ICM20948_BANK_2 | 0x00)    /**< Gyroscope Sample Rate Divider register     */

#define REG_GYRO_SMPLRT_DIV_VAL_S1          0x13                        /**< Value of the register at the end the step 1            */

#define ICM20948_REG_GYRO_CONFIG_1          (ICM20948_BANK_2 | 0x01)    /**< Gyroscope Configuration 1 register         */
#define ICM20948_BIT_GYRO_FCHOICE           0x01                        /**< Gyro Digital Low-Pass Filter enable bit    */
#define ICM20948_BITS_GYRO_FS_SEL           0x06                        /**< Gyro Full Scale Select bits                */
#define ICM20948_BITS_GYRO_DLPCFG           0x38                        /**< Gyro DLPF Config bits                      */

#define ICM20948_GYRO_FULLSCALE_250DPS      0x00                        /**< Gyro Full Scale = 250 deg/s                */
#define ICM20948_GYRO_FULLSCALE_500DPS      0x01                        /**< Gyro Full Scale = 500 deg/s                */
#define ICM20948_GYRO_FULLSCALE_1000DPS     0x02                        /**< Gyro Full Scale = 1000 deg/s               */
#define ICM20948_GYRO_FULLSCALE_2000DPS     0x03                        /**< Gyro Full Scale = 2000 deg/s               */

#define ICM20948_GYRO_BW_200HZ              0x00                        /**< Gyro Bandwidth = 200 Hz                    */
#define ICM20948_GYRO_BW_150HZ              0x01                        /**< Gyro Bandwidth = 150 Hz                    */
#define ICM20948_GYRO_BW_120HZ              0x02                        /**< Gyro Bandwidth = 120 Hz                    */
#define ICM20948_GYRO_BW_51HZ               0x03                        /**< Gyro Bandwidth = 51 Hz                     */
#define ICM20948_GYRO_BW_24HZ               0x04                        /**< Gyro Bandwidth = 24 Hz                     */
#define ICM20948_GYRO_BW_12HZ               0x05                        /**< Gyro Bandwidth = 12 Hz                     */
#define ICM20948_GYRO_BW_6HZ                0x06                        /**< Gyro Bandwidth = 6 Hz                      */
#define ICM20948_GYRO_BW_360HZ              0x07                        /**< Gyro Bandwidth = 360 Hz                    */

#define REG_GYRO_CONFIG_1_VAL_S1            0x07                        /**< Value of the register at the end the step 1            */

#define ICM20948_REG_GYRO_CONFIG_2          (ICM20948_BANK_2 | 0x02)    /**< Gyroscope Configuration 2 register                     */
#define ICM20948_BIT_GYRO_CTEN              0x38                        /**< Gyroscope Self-Test Enable bits                        */

#define ICM20948_REG_XG_OFFS_USRH           (ICM20948_BANK_2 | 0x03)    /**< Gyroscope sensor X-axis offset cancellation high byte  */
#define ICM20948_REG_XG_OFFS_USRL           (ICM20948_BANK_2 | 0x04)    /**< Gyroscope sensor X-axis offset cancellation low byte   */
#define ICM20948_REG_YG_OFFS_USRH           (ICM20948_BANK_2 | 0x05)    /**< Gyroscope sensor Y-axis offset cancellation high byte  */
#define ICM20948_REG_YG_OFFS_USRL           (ICM20948_BANK_2 | 0x06)    /**< Gyroscope sensor Y-axis offset cancellation low byte   */
#define ICM20948_REG_ZG_OFFS_USRH           (ICM20948_BANK_2 | 0x07)    /**< Gyroscope sensor Z-axis offset cancellation high byte  */
#define ICM20948_REG_ZG_OFFS_USRL           (ICM20948_BANK_2 | 0x08)    /**< Gyroscope sensor Z-axis offset cancellation low byte   */

#define ICM20948_REG_ODR_ALIGN_EN           (ICM20948_BANK_2 | 0x09)    /**< Output Data Rate start time alignment                  */

#define ICM20948_REG_ACCEL_SMPLRT_DIV_1     (ICM20948_BANK_2 | 0x10)    /**< Acceleration Sensor Sample Rate Divider 1 register     */
#define ICM20948_BITS_ACCEL_SMPLRT_DIV_1     0x0F                       /**< Acceleration Sensor Sample Rate Divider 1 bits         */

#define REG_ACCEL_SMPLRT_DIV_1_S1            0x0 0                       /**< Value of the register at the end the step 1            */

#define ICM20948_REG_ACCEL_SMPLRT_DIV_2     (ICM20948_BANK_2 | 0x11)    /**< Acceleration Sensor Sample Rate Divider 2 register     */

#define REG_ACCEL_SMPLRT_DIV_2_VAL_S1       0x13                        /**< Value of the register at the end the step 1            */

#define ICM20948_REG_ACCEL_INTEL_CTRL       (ICM20948_BANK_2 | 0x12)    /**< Accelerometer Hardware Intelligence Control register   */
#define ICM20948_BIT_ACCEL_INTEL_EN         0x02                        /**< Wake-up On Motion enable bit                           */
#define ICM20948_BIT_ACCEL_INTEL_MODE       0x01                        /**< WOM algorithm selection bit                            */

#define ICM20948_REG_ACCEL_WOM_THR          (ICM20948_BANK_2 | 0x13)    /**< Wake-up On Motion Threshold register                   */

#define ICM20948_REG_ACCEL_CONFIG           (ICM20948_BANK_2 | 0x14)    /**< Accelerometer Configuration register                   */
#define ICM20948_BIT_ACCEL_FCHOICE          0x01                        /**< Accel Digital Low-Pass Filter enable bit               */
#define ICM20948_BITS_ACCEL_FS              0x06                        /**< Accel Full Scale Select bits                           */
#define ICM20948_BITS_ACCEL_DLPCFG          0x38                        /**< Accel DLPF Config bits                                 */

#define ICM20948_ACCEL_FULLSCALE_2G         0x00                        /**< Accel Full Scale = 2 g                                 */
#define ICM20948_ACCEL_FULLSCALE_4G         0x01                        /**< Accel Full Scale = 4 g                                 */
#define ICM20948_ACCEL_FULLSCALE_8G         0x02                        /**< Accel Full Scale = 8 g                                 */
#define ICM20948_ACCEL_FULLSCALE_16G        0x03                        /**< Accel Full Scale = 16 g                                */

#define ICM20948_ACCEL_BW_246HZ             0x00                        /**< Accel Bandwidth = 246 Hz                               */
#define ICM20948_ACCEL_BW_111HZ             0x02                        /**< Accel Bandwidth = 111 Hz                               */
#define ICM20948_ACCEL_BW_50HZ              0x03                        /**< Accel Bandwidth = 50 Hz                                */
#define ICM20948_ACCEL_BW_24HZ              0x04                        /**< Accel Bandwidth = 24 Hz                                */
#define ICM20948_ACCEL_BW_12HZ              0x05                        /**< Accel Bandwidth = 12 Hz                                */
#define ICM20948_ACCEL_BW_6HZ               0x06                        /**< Accel Bandwidth = 6 Hz                                 */
#define ICM20948_ACCEL_BW_470HZ             0x07                        /**< Accel Bandwidth = 470 Hz                               */

#define REG_ACCEL_CONFIG_VAL_S1             0x03                        /**< Value of the register at the end the step 1            */

#define ICM20948_REG_ACCEL_CONFIG_2         (ICM20948_BANK_2 | 0x15)    /**< Accelerometer Configuration 2 register             */
#define ICM20948_BIT_ACCEL_CTEN             0x1C                        /**< Accelerometer Self-Test Enable bits                */


#define DMP_REG_PRS_ODR_CONFIG              (ICM20948_BANK_2 | 0x20)

#define DMP_REG_PRGM_START_ADDRH            (ICM20948_BANK_2 | 0x50)
#define DMP_REG_PRGM_START_ADDRL            (ICM20948_BANK_2 | 0x51)
#define ICM20948_REG_FSYNC_CONFIG           (ICM20948_BANK_2 | 0x52)
#define ICM20948_REG_TEMP_CONFIG            (ICM20948_BANK_2 | 0x53)
#define ICM20948_REG_MOD_CTRL_USR           (ICM20948_BANK_2 | 0x54)


/***********************/
/* Bank 3 register map */
/***********************/
#define ICM20948_REG_I2C_MST_ODR_CONFIG     (ICM20948_BANK_3 | 0x00)    /**< I2C Master Output Data Rate Configuration register */

#define ICM20948_REG_I2C_MST_CTRL           (ICM20948_BANK_3 | 0x01)    /**< I2C Master Control register                        */

#define ICM20948_BIT_I2C_MST_P_NSR          0x10                        /**< Stop between reads enabling bit                    */
#define ICM20948_BITS_I2C_MST_CLK           0x0F                        /**< I2C_MST_CLK bits                                   */
#define ICM20948_I2C_MST_CTRL_CLK_400KHZ    0x07                        /**< I2C_MST_CLK = 345.6 kHz (for 400 kHz Max)          */

#define ICM20948_REG_I2C_MST_DELAY_CTRL     (ICM20948_BANK_3 | 0x02)    /**< I2C Master Delay Control register                  */
#define ICM20948_BIT_SLV0_DLY_EN            0x01                        /**< I2C Slave0 Delay Enable bit                        */
#define ICM20948_BIT_SLV1_DLY_EN            0x02                        /**< I2C Slave1 Delay Enable bit                        */
#define ICM20948_BIT_SLV2_DLY_EN            0x04                        /**< I2C Slave2 Delay Enable bit                        */
#define ICM20948_BIT_SLV3_DLY_EN            0x08                        /**< I2C Slave3 Delay Enable bit                        */

#define ICM20948_REG_I2C_SLV0_ADDR          (ICM20948_BANK_3 | 0x03)    /**< I2C Slave0 Physical Address register               */
#define ICM20948_REG_I2C_SLV0_REG           (ICM20948_BANK_3 | 0x04)    /**< I2C Slave0 Register Address register               */
#define ICM20948_REG_I2C_SLV0_CTRL          (ICM20948_BANK_3 | 0x05)    /**< I2C Slave0 Control register                        */
#define ICM20948_REG_I2C_SLV0_DO            (ICM20948_BANK_3 | 0x06)    /**< I2C Slave0 Data Out register                       */

#define ICM20948_REG_I2C_SLV1_ADDR          (ICM20948_BANK_3 | 0x07)    /**< I2C Slave1 Physical Address register               */
#define ICM20948_REG_I2C_SLV1_REG           (ICM20948_BANK_3 | 0x08)    /**< I2C Slave1 Register Address register               */
#define ICM20948_REG_I2C_SLV1_CTRL          (ICM20948_BANK_3 | 0x09)    /**< I2C Slave1 Control register                        */
#define ICM20948_REG_I2C_SLV1_DO            (ICM20948_BANK_3 | 0x0A)    /**< I2C Slave1 Data Out register                       */

#define ICM20948_REG_I2C_SLV2_ADDR          (ICM20948_BANK_3 | 0x0B)    /**< I2C Slave2 Physical Address register               */
#define ICM20948_REG_I2C_SLV2_REG           (ICM20948_BANK_3 | 0x0C)    /**< I2C Slave2 Register Address register               */
#define ICM20948_REG_I2C_SLV2_CTRL          (ICM20948_BANK_3 | 0x0D)    /**< I2C Slave2 Control register                        */
#define ICM20948_REG_I2C_SLV2_DO            (ICM20948_BANK_3 | 0x0E)    /**< I2C Slave2 Data Out register                       */

#define ICM20948_REG_I2C_SLV3_ADDR          (ICM20948_BANK_3 | 0x0F)    /**< I2C Slave3 Physical Address register               */
#define ICM20948_REG_I2C_SLV3_REG           (ICM20948_BANK_3 | 0x10)    /**< I2C Slave3 Register Address register               */
#define ICM20948_REG_I2C_SLV3_CTRL          (ICM20948_BANK_3 | 0x11)    /**< I2C Slave3 Control register                        */
#define ICM20948_REG_I2C_SLV3_DO            (ICM20948_BANK_3 | 0x12)    /**< I2C Slave3 Data Out register                       */

#define ICM20948_REG_I2C_SLV4_ADDR          (ICM20948_BANK_3 | 0x13)    /**< I2C Slave4 Physical Address register               */
#define ICM20948_REG_I2C_SLV4_REG           (ICM20948_BANK_3 | 0x14)    /**< I2C Slave4 Register Address register               */

#define ICM20948_REG_I2C_SLV4_CTRL          (ICM20948_BANK_3 | 0x15)    /**< I2C Slave4 Control register                        */
#define ICM20948_I2C_SLV4_CTRL_MAG_VAL      0x80                        /* EN = 1, INT_EN = 0, REG_DIS = 0,  DLY[4:0] = 0       */

#define ICM20948_REG_I2C_SLV4_DO            (ICM20948_BANK_3 | 0x16)    /**< I2C Slave4 Data Out register                       */
#define ICM20948_REG_I2C_SLV4_DI            (ICM20948_BANK_3 | 0x17)    /**< I2C Slave4 Data In register                        */

#define ICM20948_BIT_I2C_SLV_READ           0x80                        /**< I2C Slave Read bit                                 */
#define ICM20948_BIT_I2C_SLV_EN             0x80                        /**< I2C Slave Enable bit                               */
#define ICM20948_BIT_I2C_BYTE_SW            0x40                        /**< I2C Slave Byte Swap enable bit                     */
#define ICM20948_BIT_I2C_REG_DIS            0x20                        /**< I2C Slave Do Not Write Register Value bit          */
#define ICM20948_BIT_I2C_GRP                0x10                        /**< I2C Slave Group bit                                */
#define ICM20948_BIT_I2C_READ               0x80                        /**< I2C Slave R/W bit                                  */

/* Register common for all banks */
#define ICM20948_REG_BANK_SEL               0x7F                        /**< Bank Select register                               */


/* Register common for Magnetometer AK09916 */               
#define AK09916_REG_WIA2                    0x01                        // Device ID
#define AK09916_REG_RSV1                    0x02                        // Reserved register. 
#define AK09916_REG_RSV2                    0x03                        // Reserved register. We start reading here when using the DMP. Secret sauce...
                                                                        // discontinuity - containing another nine reserved registers? Secret sauce...
#define AK09916_REG_ST1                     0x10                        // Data status 1
#define AK09916_REG_HXL                     0x11                        // Measurement data X
#define AK09916_REG_HXH                     0x12
#define AK09916_REG_HYL                     0x13                        // Measurement data Y
#define AK09916_REG_HYH                     0x14
#define AK09916_REG_HZL                     0x15                        // Measurement data Z
#define AK09916_REG_HZH                     0x16
                                                                        // discontinuity
#define AK09916_REG_ST2                     0x18                        // Data status 2
                                                                        // discontinuity
#define AK09916_REG_CNTL2                   0x31                        // Control settings 2
#define AK09916_REG_CNTL3                   0x32                        // Controls setting 3

 
class ICM20948Class
{
  public:

    ICM20948Class();
    
    void    printBinary(uint8_t inByte);
    
    void    ICM20948_initializeSPI(uint8_t slaveSelectPin);
    void    ICM20948_initializeI2C();
    void    ICM20948_writeRegI2C(uint8_t regaddr, const uint8_t regval);
    void    ICM20948_readRegI2C(uint8_t regaddr, uint8_t *rdData);
     
    void    ICM20948_writeReg(uint8_t regaddr, const uint8_t regval);
    void    ICM20948_writeData(uint8_t regaddr, const uint8_t *pwrData, uint32_t len);   
    void    ICM20948_readData(uint8_t regaddr, uint8_t *rdData, uint32_t len);
    void    ICM20948_read_mems(uint16_t regaddr, uint32_t len, uint8_t *data); 
    void    ICM20948_write_mems(uint16_t regaddr, uint32_t len, const uint8_t *data);
    
    void    ICM20948_switchBank(uint8_t newBank);
    void    ICM20948_switchDMPBank(uint8_t newDMPBank);           
    void    ICM20948_writeBankReg(uint16_t bankreg, const uint8_t regval);
    void    ICM20948_writeBankData(uint16_t bankreg, const uint8_t *pwrData, uint32_t len);
    void    ICM20948_readBankReg(uint16_t bankreg, uint8_t *regval);
    void    ICM20948_readBankData(uint16_t bankreg, uint8_t *prdData, uint32_t len);

    void    ICM20948_reset(void);
    void    ICM20948_sleep(bool enable);
    void    ICM20948_lowPower(bool enable);
    
    void    ICM20948_setClockSource(uint8_t val);
    void    ICM20948_enableAccel(bool enable);
    void    ICM20948_enableGyro(bool enable);
    void    ICM20948_enableAccelCycledMode(bool enable);
    void    ICM20948_enableGyroCycledMode(bool enable);
    void    ICM20948_enableDMP(bool enable);
    void    ICM20948_enableFIFO(bool enable);
    void    ICM20948_enableFIFOSnapshot(bool enable);
    void    ICM20948_enableIntFIFOOverflow(bool enable);
    void    ICM20948_enableWriteFIFO(bool enable);
    void    ICM20948_enableIntRawdataReady(bool enable);

    void    ICM20948_resetFIFO(void);   
    void    ICM20948_resetDMP(void);
    void    ICM20948_resetI2CMaster(void);
    
    void    ICM20948_setFullScaleGyro(uint8_t val);
    void    ICM20948_enableDLPFGyro(uint8_t val, bool enable);
    void    ICM20948_setSampleRateGyro(uint8_t val);
    void    ICM20948_setFullScaleAccel(uint8_t val);
    void    ICM20948_enableDLPFAccel(uint8_t val, bool enable);
    void    ICM20948_setSampleRateAccel(uint16_t val);
    void    ICM20948_enable_I2CMasterPassthrough( bool enable);
    void    ICM20948_enableI2CMaster(uint8_t val, bool enable);
    
    void    ICM20948_setDMPstartAddress(uint16_t address);
    void    ICM20948_loadDMPFirmware(void);
    void    ICM20948_setGyroSF(uint8_t div, uint32_t gyro_level);
    void    ICM20948_enableDMPSensor(uint16_t DMP_sensors);
    void    ICM20948_getFIFOcount(uint16_t *count);
    void    ICM20948_readFIFO(uint8_t *data, uint8_t len);
    uint8_t ICM20948_readDMPdataFromFIFO(icm20948_DMP_data_t *data);
    void    ICM20948_i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut);

        
    uint8_t ICM20948_initializeDMP(bool Magnetometer);
    void    ICM20948_setDMPDsensorPeriod(enum DMP_ODR_Registers odr_reg, uint16_t interval);
    uint8_t ICM20948_startupDefault(bool Magnetometer);
    void    ICM20948_configureMagnetometer(void);
    uint8_t ICM20948_checkDeviceID(void);
    uint8_t ICM20948_checkStartup(void);
    uint8_t ICM20948_checkStep1(void);
    uint8_t ICM20948_getLaststatus(void);
    uint8_t ICM20948_startupMagnetometer(void);
    uint8_t ICM20948_magWhoIAm(void);
    uint8_t ICM20948_i2c_controller_periph4_txn(uint8_t addr, uint8_t reg, uint8_t data, bool Rw);


    
  private:
    uint8_t _slaveSelectPin; //SPI
    uint32_t _SPIfreq; //SPI
    uint8_t _address;        //I2C
    uint8_t _last_nb_receive; //I2C
    uint8_t _last_status;
    uint8_t _currentBank;
    uint8_t _currentDMPBank;
    bool _firmware_loaded;
    uint8_t _gyroSFpll;
    uint32_t _gyroSF;
 
}; 
  
#endif


