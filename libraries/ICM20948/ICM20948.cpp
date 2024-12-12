#include <Arduino.h>
#include <SPI.h>
#include <ICM20948.h>
#include <ICM20948_DMP.h>

#include <avr/pgmspace.h>
const uint8_t dmp3_image[] PROGMEM = {
#include "icm20948_img.dmp3a.h"
};


ICM20948Class::ICM20948Class(void)
{
  _last_status = 0;
  _currentBank = 0xFF;
  _currentDMPBank = 0xFF;
  _firmware_loaded = 0;
  _gyroSFpll = 0;
  _gyroSF = 0;  
}

void ICM20948Class::printBinary(uint8_t inByte)
{
  for (int8_t b = 7; b >= 0; b--)
  {
    Serial.print(bitRead(inByte, b));
  }
}

uint8_t ICM20948Class::ICM20948_getLaststatus(void)
{
    return _last_status;
}

#ifdef I2C
void ICM20948Class::ICM20948_initializeI2C()
{ 
    Serial.println("Start ICM20948_initializeI2C");
    // Setup I2C
    _address =ADDRESS_DEFAULT;  

}
#endif

void ICM20948Class::ICM20948_initializeSPI(uint8_t slaveSelectPin)
{ 
  Serial.println("Start ICM20948_initializeSPI");
  
   _SPIfreq= F_CPU/4;
  if (_SPIfreq > SPI_SPEEDMAXIMUM) _SPIfreq = SPI_SPEEDMAXIMUM;
  Serial.print("_SPIfreq: "); Serial.println(_SPIfreq);
  Serial.print("SPISETTINGS_DATAORDER: "); Serial.println(SPISETTINGS_DATAORDER);
  Serial.print("SPISETTINGS_DATAMODE: "); Serial.println(SPISETTINGS_DATAMODE,HEX);
  Serial.print("Slave Select Pin: "); Serial.println(slaveSelectPin);
  
  // set the slaveSelectPin as an output
  _slaveSelectPin = slaveSelectPin;
  pinMode(_slaveSelectPin, OUTPUT);
  digitalWrite(_slaveSelectPin, HIGH);

  // initialize SPI
  SPI.begin();
  SPI.beginTransaction(SPISettings(_SPIfreq, SPISETTINGS_DATAORDER, SPISETTINGS_DATAMODE));
  SPI.transfer(0x00);
  SPI.endTransaction();

  Serial.println("End ICM20948_initializeSPI");
}

uint8_t ICM20948Class::ICM20948_checkDeviceID()
{
    uint8_t deviceID = 0;
    
    ICM20948_readBankReg(ICM20948_REG_WHO_AM_I, &deviceID);
    if (_last_status > 0) return _last_status;
   
    //Serial.print(F("ICM20948_checkDeviceID - deviceID: "));Serial.println(deviceID,HEX);     
     
    if (deviceID != ICM20948_ID) _last_status = CHECK_DEVICE_ERROR; 
    
    return _last_status;      
}

uint8_t ICM20948Class::ICM20948_checkStartup()
{
    uint8_t regval = 0;
    
    ICM20948_readBankReg(ICM20948_REG_PWR_MGMT_1, &regval);
    if (_last_status > 0) return _last_status;
           
    if (regval == REG_PWR_MGMT_1_VAL_STARTUP) return 0;
    else return regval;
        
    ICM20948_readBankReg(ICM20948_REG_INT_ENABLE_1, &regval);
    if (_last_status > 0) return _last_status;
           
    if (regval == REG_INT_ENABLE_1_VAL_STARTUP) return 0;
    else return regval;
    
    ICM20948_readBankReg(ICM20948_REG_INT_ENABLE_2, &regval);
    if (_last_status > 0) return _last_status;
           
    if (regval == REG_INT_ENABLE_2_VAL_STARTUP) return 0;
    else return regval;       
    
    ICM20948_readBankReg(ICM20948_REG_FIFO_EN_1, &regval);
    if (_last_status > 0) return _last_status;
           
    if (regval == REG_FIFO_EN_1_VAL_STARTUP) return 0;
    else return regval;
    
    ICM20948_readBankReg(ICM20948_REG_FIFO_EN_2, &regval);
    if (_last_status > 0) return _last_status;
           
    if (regval == REG_FIFO_EN_2_VAL_STARTUP) return 0;
    else return regval;   
        
              
}

uint8_t ICM20948Class::ICM20948_checkStep1()
{
    uint8_t regval = 0;
    
    ICM20948_readBankReg(ICM20948_REG_PWR_MGMT_1, &regval);
    if (_last_status > 0) return _last_status;    
    if (regval != REG_PWR_MGMT_1_VAL_S1)
    {   
         Serial.print(F("ICM20948_checkStep1 - ICM20948_REG_PWR_MGMT_1]: "));printBinary(regval);Serial.print(F(" <> "));printBinary(REG_PWR_MGMT_1_VAL_S1);Serial.println(F(" "));  
         return regval;
    }
    
    ICM20948_readBankReg(ICM20948_REG_PWR_MGMT_2, &regval);
    if (_last_status > 0) return _last_status;    
    if (regval != REG_PWR_MGMT_2_VAL_S1)
    {   
         Serial.print(F("ICM20948_checkStep1 - ICM20948_REG_PWR_MGMT_2]: "));printBinary(regval);Serial.print(F(" <> "));printBinary(REG_PWR_MGMT_2_VAL_S1);Serial.println(F(" "));  
         return regval;
    }
    
    ICM20948_readBankReg(ICM20948_REG_LP_CONFIG, &regval);
    if (_last_status > 0) return _last_status;    
    if (regval != REG_LP_CONFIG_VAL_S1)
    {   
         Serial.print(F("ICM20948_checkStep1 - ICM20948_REG_LP_CONFIG]: "));printBinary(regval);Serial.print(F(" <> "));printBinary(REG_LP_CONFIG_VAL_S1);Serial.println(F(" "));  
         return regval;
    }
           
    ICM20948_readBankReg(ICM20948_REG_USER_CTRL, &regval);
    if (_last_status > 0) return _last_status;    
    if (regval != REG_USER_CTRL_VAL_S1)
    {   
         Serial.print(F("ICM20948_checkStep1 - ICM20948_REG_USER_CTRL]: "));printBinary(regval);Serial.print(F(" <> "));printBinary(REG_USER_CTRL_VAL_S1);Serial.println(F(" "));  
        //FIXME return regval;
    }
               
    ICM20948_readBankReg(ICM20948_REG_GYRO_CONFIG_1, &regval);
    if (_last_status > 0) return _last_status;    
    if (regval != REG_GYRO_CONFIG_1_VAL_S1)
    {   
         Serial.print(F("ICM20948_checkStep1 - ICM20948_REG_GYRO_CONFIG_1]: "));printBinary(regval);Serial.print(F(" <> "));printBinary(REG_GYRO_CONFIG_1_VAL_S1);Serial.println(F(" "));  
         return regval;
    }
    return 0;
}

uint8_t ICM20948Class::ICM20948_startupDefault(bool Magnetometer)
{
  int8_t result = 0;
  
  ICM20948_reset();
  if (_last_status > 0) return _last_status;
    
  delay(50);

  ICM20948_sleep(false);
  if (_last_status > 0) return _last_status;
  
  ICM20948_lowPower(false);
  if (_last_status > 0) return _last_status;

  if (Magnetometer) {
     result = ICM20948_startupMagnetometer(); // startupMagnetometer
     if (_last_status > 0) {   
        Serial.println(F("ICM20948_checkStep1 - ICM20948_startupMagnetometer KO"));
        Serial.print(F("result: "));Serial.print(result);Serial.print(F("  _last_status: "));Serial.println(_last_status);
        return _last_status;
     }
  }
   
  result = ICM20948_checkDeviceID();
  if (result > 0) 
  {
     _last_status = CHECK_DEVICE_ERROR;
     return result;
  }
   
  result = ICM20948_checkStartup();
  if (result > 0)
  {
     _last_status = CHECK_STARTUP_ERROR;
     return result;
  } 

  return 0;
}

uint8_t ICM20948Class::ICM20948_initializeDMP(bool Magnetometer)
{
    //PRINTs(">>>Start  DMP_init") 
    Serial.println("Start ICM20948_initializeDMP"); 
    
    if (Magnetometer)
    {
      ICM20948_configureMagnetometer();
      if (_last_status > 0) return _last_status;
    } 
             
    /*
    3.1 Initializing the ICM
        Configure clock source through PWR_MGMT_1.
        Enable accel and gyro sensors through PWR_MGMT_2.
        Configure Gyro/Accel in Low power mode with LP_CONFIG. ??
        Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1.
        Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG.
    */
 
    // Auto selects the best available clock source, i.e. the PLL will be selected whenever gyroscope is on.
    // The internal relaxation oscillator is trimmed to have a consistent operating frequency at room temperature, while the PLL clock frequency varies from part to part
    // But PLL has better frequency stability and lower frequency variation over temperature than the internal relaxation oscillator.
    // The PLL frequency deviation from the nominal value in percentage is captured in register TIMEBASE_CORRECTION_PLL,
    ICM20948_setClockSource(ICM20948_CLOCK_AUTO);
    if (_last_status > 0) return _last_status;  
            
    // Enable Accelerometer and Gyroscope(all axes)on.
    ICM20948_enableAccel(true);
    if (_last_status > 0) return _last_status; 
    
    ICM20948_enableGyro(true);
    if (_last_status > 0) return _last_status;    
     
    //Configure Gyro/Accel in Low power mode.
    ICM20948_enableAccelCycledMode(false);
    if (_last_status > 0) return _last_status; 
    
    ICM20948_enableGyroCycledMode(false);  
    if (_last_status > 0) return _last_status;
                
    delay(1); // Give the ICM20948 time to change the sample mode (see issue #8)                    
 
    // Disable FIFO  
    ICM20948_enableFIFO(false);
    if (_last_status > 0) return _last_status;
    
    // Disable DMP  
    ICM20948_enableDMP(false);
    if (_last_status > 0) return _last_status; 
             
    // Config Gyro 2000dps, value from DMP doc
    ICM20948_setFullScaleGyro(ICM20948_GYRO_FULLSCALE_2000DPS);   
    if (_last_status > 0) return _last_status;       
         
    // Enable Gyro DLPF with bandwith 196.6HZ - 229.8HZ , value from DMP doc          
    ICM20948_enableDLPFGyro(ICM20948_GYRO_BW_200HZ, true);    
    if (_last_status > 0) return _last_status;
        
    // Config Accel 4g, value from DMP doc
    ICM20948_setFullScaleAccel(ICM20948_ACCEL_FULLSCALE_4G);   
    if (_last_status > 0) return _last_status; 
          
    // Enable Accel DLPF with bandwith 246.0HZ - 265.0HZ ???   
    ICM20948_enableDLPFAccel(ICM20948_ACCEL_BW_246HZ, true);
    if (_last_status > 0) return _last_status;   

 /*
 3.2 Configuring FIFO and Interrupts
    Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE
    Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2.
    Turn off data ready interrupt through INT_ENABLE_1.  
   */ 
       
    //Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2.     
    //ICM20948_enableIntFIFOOverflow(true);
    
    //Enable snapshot for FIFO, wwhen the FIFO is full, additional writes will be written to the FIFO, replacing the oldest data..
    ICM20948_enableFIFOSnapshot(false);
    if (_last_status > 0) return _last_status; 
        
    // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2   
    ICM20948_enableWriteFIFO(false);
    if (_last_status > 0) return _last_status;
        
    // Turn off data ready interrupt through INT_ENABLE_1    
    ICM20948_enableIntRawdataReady(false);
    if (_last_status > 0) return _last_status;             

  /*
  3.3 Reset the FIFO
    Reset FIFO through FIFO_RST.
  */
  
    // Reset FIFO through FIFO_RST 
    ICM20948_resetFIFO();
    if (_last_status > 0) return _last_status;    
 
 /*
 3.4 Configuring Sensor Sample Rate
    Set gyro sample rate divider with GYRO_SMPLRT_DIV.
    Set accel sample rate divider with ACCEL_SMPLRT_DIV_2.       
 */      
    // Config Gyro sample rate divider = 19 ODR = 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]) = 55HZ InvenSense Nucleo example uses 19 (0x13).
    ICM20948_setSampleRateGyro(0x13);
    if (_last_status > 0) return _last_status;   
     
    // Config Accel sample rate divider = 19 ODR = 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]) = 56.25HZ InvenSense Nucleo example uses 19 (0x13).
    ICM20948_setSampleRateAccel(0x13);
    if (_last_status > 0) return _last_status;  
   
    uint8_t buf;
    ICM20948_readBankReg(ICM20948_REG_ACCEL_CONFIG, &buf);
    Serial.print("ICM20948_REG_ACCEL_CONFIG = ");Serial.println(buf,BIN);


    ICM20948_readBankReg(ICM20948_REG_FIFO_RST, &buf);
    Serial.print("ICM20948_REG_FIFO_RST = ");Serial.println(buf,BIN);
 
   
    ICM20948_readBankReg(ICM20948_REG_GYRO_SMPLRT_DIV, &buf);
    Serial.print("ICM20948_REG_GYRO_SMPLRT_DIV = ");Serial.println(buf,BIN);   
    ICM20948_readBankReg(ICM20948_REG_ACCEL_SMPLRT_DIV_1, &buf);
    Serial.print("ICM20948_REG_ACCEL_SMPLRT_DIV_1 = ");Serial.println(buf,BIN);  
    ICM20948_readBankReg(ICM20948_REG_ACCEL_SMPLRT_DIV_2, &buf);
    Serial.print("ICM20948_REG_ACCEL_SMPLRT_DIV_2 = ");Serial.println(buf,BIN);  
    
     uint8_t result = ICM20948_checkStep1();
     if (result > 0)
     {
        _last_status = 3;
        return result;
    }
 
/*
3.5 Configuring the DMP start address
    Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL.  
*/
  Serial.print("DMP_START_ADDRESS: ");Serial.println(DMP_START_ADDRESS,HEX);  
  ICM20948_setDMPstartAddress(DMP_START_ADDRESS); 
  if (_last_status > 0) return _last_status;    
              
/*
3.6 Loading DMP Firmware
    The DMP firmware size is >12Kbytes. DMP program code may be loaded one byte at a time or in burst mode as explained below.
    3.6.1 Load Firmware One Byte at a Time
        Starting from byte [0] of the firmware image provided, perform the following steps to load each byte of image to Nth byte of DMP address. N must start from address (0x90) and increase accordingly:
            Write the value (N >> 8) to MEM_BANK_SEL
            Write the value (N & 0xFF) to MEM_START_ADDR
            Write the data to MEM_R_W starting from the 0x90th byte of DMP firmware image.
        Repeat the steps above for each byte of the DMP firmware image.
    3.6.2 Load Firmware up to 256 Bytes at a Time
        Up to 256 bytes can be burst written. Please follow the steps outlined in previous section. However, the maximum number of bytes that can be written at once is not always 256, but is given by (256 – (N & 0xFF)).

    3.6.3 Load Firmware Start Value and start DMP
    Please perform the following once the DMP firmware has been loaded. 
        Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
        The Firmware Start Value is provided by InvenSense. If you do not have this value, please contact your field representative.   
*/

  // Load the DMP firmware
  ICM20948_loadDMPFirmware();
  if (_last_status > 0) return _last_status;  
  
  // Set the Single FIFO Priority Select register to 0xE4 
  ICM20948_writeBankReg(DMP_REG_SINGLE_FIFO_PRIORITY_SEL, 0xE4); 
  if (_last_status > 0) return _last_status; 

/*
3.7 Configure Accel scaling to DMP
    The DMP scales accel raw data internally to align 1g as 2^25.
    To do this and output hardware unit again as configured FSR, write 0x4000000 to ACC_SCALE DMP register, and write 0x40000 to ACC_SCALE2 DMP register.
*/
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  const uint8_t accScale[4] = {0x04, 0x00, 0x00, 0x00};
  ICM20948_write_mems(ACC_SCALE, 4, &accScale[0]);  // Write accScale to ACC_SCALE DMP register
  if (_last_status > 0) return _last_status; 
  
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
  const uint8_t accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  ICM20948_write_mems(ACC_SCALE2, 4, &accScale2[0]);  // Write accScale2 to ACC_SCALE2 DMP register
  if (_last_status > 0) return _last_status; 
    
/*
3.8 Configure Compass mount matrix and scale to DMP
The mount matrix write to DMP register is used to align the compass axes with accel/gyro. This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
*/
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
  // 2^30 / 6.66666 = 161061273 = 0x9999999
  const uint8_t mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const uint8_t mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
  const uint8_t mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
  ICM20948_write_mems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]); if (_last_status > 0) return _last_status; 
  ICM20948_write_mems(CPASS_MTX_01, 4, &mountMultiplierZero[0]); if (_last_status > 0) return _last_status; 
  ICM20948_write_mems(CPASS_MTX_02, 4, &mountMultiplierZero[0]); if (_last_status > 0) return _last_status; 
  ICM20948_write_mems(CPASS_MTX_10, 4, &mountMultiplierZero[0]); if (_last_status > 0) return _last_status; 
  ICM20948_write_mems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]);if (_last_status > 0) return _last_status; 
  ICM20948_write_mems(CPASS_MTX_12, 4, &mountMultiplierZero[0]); if (_last_status > 0) return _last_status; 
  ICM20948_write_mems(CPASS_MTX_20, 4, &mountMultiplierZero[0]); if (_last_status > 0) return _last_status; 
  ICM20948_write_mems(CPASS_MTX_21, 4, &mountMultiplierZero[0]); if (_last_status > 0) return _last_status; 
  ICM20948_write_mems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]);if (_last_status > 0) return _last_status; 
    
  // Configure the B2S Mounting Matrix
  const uint8_t b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const uint8_t b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  ICM20948_write_mems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]); if (_last_status > 0) return _last_status;
  ICM20948_write_mems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]); if (_last_status > 0) return _last_status;
  ICM20948_write_mems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]); if (_last_status > 0) return _last_status;
  ICM20948_write_mems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]); if (_last_status > 0) return _last_status;
  ICM20948_write_mems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]); if (_last_status > 0) return _last_status;
  ICM20948_write_mems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]); if (_last_status > 0) return _last_status;
  ICM20948_write_mems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]); if (_last_status > 0) return _last_status;
  ICM20948_write_mems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]); if (_last_status > 0) return _last_status;
  ICM20948_write_mems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]); if (_last_status > 0) return _last_status;

  // Configure the DMP Gyro Scaling Factor
  // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
  //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
  //            10=102.2727Hz sample rate, ... etc.
  // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
  ICM20948_setGyroSF(19, 3);  // 19 = 55Hz (see above), 3 = 2000dps (see above)
  if (_last_status > 0) return _last_status;
    
  // Configure the Gyro full scale
  // 2000dps : 2^28
  // 1000dps : 2^27
  //  500dps : 2^26
  //  250dps : 2^25
  const uint8_t gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  ICM20948_write_mems(GYRO_FULLSCALE, 4, &gyroFullScale[0]);
  if (_last_status > 0) return _last_status;
    
  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  const uint8_t accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  //const uint8_t accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
  //const uint8_t accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
  ICM20948_write_mems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]); 
  if (_last_status > 0) return _last_status;
    
  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  const uint8_t accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  //const uint8_t accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
  //const uint8_t accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
  ICM20948_write_mems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]); 
  if (_last_status > 0) return _last_status;
    
  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  const uint8_t accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  //const uint8_t accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
  //const uint8_t accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
  ICM20948_write_mems(ACCEL_A_VAR, 4, &accelAVar[0]);
  if (_last_status > 0) return _last_status;
    
  // Configure the Accel Cal Rate
  const uint8_t accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
  ICM20948_write_mems(ACCEL_CAL_RATE, 2, &accelCalRate[0]);
  if (_last_status > 0) return _last_status;
    
  // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
  // Let's set the Compass Time Buffer to 69 (Hz).
  const uint8_t compassRate[2] = {0x00, 0x45}; // 69Hz
  ICM20948_write_mems(CPASS_TIME_BUFFER, 2, &compassRate[0]);
  if (_last_status > 0) return _last_status;

  //Reset FIFO
  //ICM20948_resetFIFO();
  if (_last_status > 0) return _last_status;
    
  //Reset DMP
  //ICM20948_resetDMP();
  if (_last_status > 0) return _last_status; 
    
     
  //PRINTs("<<<End OK DMP_init")

    return 0;
}

#ifdef I2C
// Write an 8-bit register
void ICM20948Class::ICM20948_writeRegI2C(uint8_t regaddr, const uint8_t regval)
{
  Wire.beginTransmission(_address);
  Wire.write(regaddr);
  Wire.write(regval);
  _last_status = Wire.endTransmission();
}

// Read an 8-bit register
void ICM20948Class::ICM20948_readRegI2C(uint8_t regaddr, uint8_t *rdData)
{
  Wire.beginTransmission(_address);
  Wire.write(regaddr);
  _last_status = Wire.endTransmission();
  if (_last_status > 0) return;

  _last_nb_receive = Wire.requestFrom(_address, (uint8_t)1);
  if (_last_nb_receive != 1) {
     _last_status=WIRE_REQUEST_ERROR;
     return;
  }
  *rdData = Wire.read();

  return;
}
#endif

void ICM20948Class::ICM20948_writeReg(uint8_t regaddr, const uint8_t regval)
{
  //Serial.print("ICM20948_writeReg - regaddr: ");Serial.println(regaddr,HEX);   
  //Serial.print("ICM20948_writeReg - regval: ");printBinary(regval);Serial.println(" ");   
  // 'Kickstart' the SPI hardware. This is a fairly high amount of overhead, but it guarantees that the lines will start in the correct states even when sharing the SPI bus with devices that use other modes
  SPI.beginTransaction(SPISettings(_SPIfreq, SPISETTINGS_DATAORDER, SPISETTINGS_DATAMODE));
  SPI.transfer(0x00);
  SPI.endTransaction();
 
  // take the SS pin low to select the chip
  digitalWrite(_slaveSelectPin, LOW);
  
  // Send the address
  // The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
  // The following 7 bits contain the Register Address.
  SPI.beginTransaction(SPISettings(_SPIfreq, SPISETTINGS_DATAORDER, SPISETTINGS_DATAMODE));
  SPI.transfer((((regaddr & 0x7F) | 0x00)));
  
  // Send the value
  SPI.transfer(regval);
  SPI.endTransaction(); 
  
  // take the SS pin high to de-select the chip
  digitalWrite(_slaveSelectPin, HIGH);


  return;
}

void ICM20948Class::ICM20948_writeData(uint8_t regaddr, const uint8_t *pwrData, uint32_t len)
{
  uint32_t i = len;          // writing byte to the SPI
  const uint8_t *pbuf;
  
  // take the SS pin low to select the chip
  digitalWrite(_slaveSelectPin, LOW);
  
  // Send the address
  // The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
  // The following 7 bits contain the Register Address.
  SPI.transfer((((regaddr & 0x7F) | 0x00)));
  
  // Send the value
  //SPI.transfer((uint8_t*)pwrData, len);
  for (pbuf = pwrData; i > 0; i--, pbuf++) {   
    // Write one byte
    SPI.transfer(*pbuf);   
  } // end while
  
  // take the SS pin high to de-select the chip
  digitalWrite(_slaveSelectPin, HIGH);
 
 return;
}

void ICM20948Class::ICM20948_readData(uint8_t regaddr, uint8_t *prdData, uint32_t len)
{
  uint32_t i = len;          // incoming byte from the SPI
  uint8_t *pbuf;
  //Serial.print("ICM20948_readData - regaddr: ");Serial.println(regaddr,HEX);   

  // 'Kickstart' the SPI hardware. This is a fairly high amount of overhead, but it guarantees that the lines will start in the correct states even when sharing the SPI bus with devices that use other modes
  SPI.beginTransaction(SPISettings(_SPIfreq, SPISETTINGS_DATAORDER, SPISETTINGS_DATAMODE));
  SPI.transfer(0x00);
  SPI.endTransaction();
  
  // take the SS pin low to select the chip
  digitalWrite(_slaveSelectPin, LOW);

  // Send in the address
  // The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
  // The following 7 bits contain the Register Address.
  SPI.beginTransaction(SPISettings(_SPIfreq, SPISETTINGS_DATAORDER, SPISETTINGS_DATAMODE));
  SPI.transfer((((regaddr & 0x7F) | 0x80)));  
  
  for (pbuf = prdData; i > 0; i--, pbuf++) {   
    // Read and store one byte
   *pbuf = SPI.transfer(0x00);
  }  // end while
  SPI.endTransaction();
    
  // take the SS pin high to de-select the chip
  digitalWrite(_slaveSelectPin, HIGH);
  //Serial.print("ICM20948_readData - *prdData: ");printBinary(*prdData);Serial.println(" ");   
  return;
}

void ICM20948Class::ICM20948_write_mems(uint16_t reg, uint32_t length, const uint8_t *data)
{
  uint32_t bytesWritten = 0;
  uint32_t thisLen;
  uint8_t lStartAddrSelected;
  
/*  To write to advanced hardware register 0xABC with data {0x1, 0x2, 0x3, 0x4}, please follow the procedure below:
   1.   Write 0xA to MEM_BANK_SEL
   2.   Write 0xBC to MEM_START_ADDR
   3. Write {0x1, 0x2, 0x3, 0x4} to MEM_R_W
*/   
  // Set DMPbank = reg >> 8, Write to MEM_BANK_SEL
  ICM20948_switchDMPBank((uint8_t)(reg >> 8));

  while (bytesWritten < length)
  {
    lStartAddrSelected = (reg & 0xff);

    /* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
           Contents are changed after read or write of the selected memory.
           This register must be written prior to each access to initialize the register to the proper starting address.
           The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */
 
    ICM20948_writeBankReg(DMP_REG_MEM_START_ADDR, lStartAddrSelected);
    if (_last_status > 0) return; 

    if (length - bytesWritten <= INV_MAX_SERIAL_WRITE)
      thisLen = length - bytesWritten;
    else
      thisLen = INV_MAX_SERIAL_WRITE;

    /* Write data */
    ICM20948_writeBankData(DMP_REG_MEM_R_W, (uint8_t *)&data[bytesWritten], thisLen);
    if (_last_status > 0) return; 

    bytesWritten += thisLen;
    reg += thisLen;
  }

  return;
}
   
void ICM20948Class::ICM20948_read_mems(uint16_t reg, uint32_t length, uint8_t *data)
   
{
  uint32_t bytesRead = 0;
  uint32_t thisLen;
  uint8_t lStartAddrSelected;

  // Set DMPbank = reg >> 8
  ICM20948_switchDMPBank((uint8_t)(reg >> 8));

  while (bytesRead < length)
  {
    lStartAddrSelected = (reg & 0xff);

    /* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
		   Contents are changed after read or write of the selected memory.
		   This register must be written prior to each access to initialize the register to the proper starting address.
		   The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */

    ICM20948_writeBankReg(DMP_REG_MEM_START_ADDR, lStartAddrSelected);
    if (_last_status > 0) return; 

    if (length - bytesRead <= INV_MAX_SERIAL_READ)
      thisLen = length - bytesRead;
    else
      thisLen = INV_MAX_SERIAL_READ;

    /* Read data */
    ICM20948_readBankData(DMP_REG_MEM_R_W, &data[bytesRead], thisLen);
    if (_last_status > 0) return; 

    bytesRead += thisLen;
    reg += thisLen;
  }

  return;
}

void ICM20948Class::ICM20948_switchBank(uint8_t newBank)
{
    if(newBank != _currentBank)
    {
        _currentBank = newBank;

#ifdef I2C 
        ICM20948_writeRegI2C(ICM20948_REG_BANK_SEL, (_currentBank<<4)); // USER_BANK[5:4] 
#else        
        ICM20948_writeReg(ICM20948_REG_BANK_SEL, (_currentBank<<4)); // USER_BANK[5:4]
#endif

    }
    return;
}

void ICM20948Class::ICM20948_switchDMPBank(uint8_t newBank)
{
    if(newBank != _currentDMPBank)
    {
        _currentDMPBank = newBank;      
        ICM20948_writeBankReg(DMP_REG_MEM_BANK_SEL, _currentDMPBank); 
    }
    return;
}

void ICM20948Class::ICM20948_writeBankReg(uint16_t bankreg, const uint8_t regval)
{
 ICM20948_switchBank((uint8_t)(bankreg >> 8));  

#ifdef I2C 
 ICM20948_writeRegI2C((uint8_t)(bankreg), regval);
#else 
 ICM20948_writeReg((uint8_t)(bankreg), regval);
#endif

 
 return;
}

void ICM20948Class::ICM20948_writeBankData(uint16_t bankreg, const uint8_t *pwrData, uint32_t len)
{
 
 ICM20948_switchBank((uint8_t)(bankreg >> 8));
 
 ICM20948_writeData((uint8_t)(bankreg), pwrData, len); 
 
 return;
}


void ICM20948Class::ICM20948_readBankReg(uint16_t bankreg, uint8_t *regval)
{
 ICM20948_switchBank((uint8_t)(bankreg >> 8));
 
#ifdef I2C
 ICM20948_readRegI2C((uint8_t)(bankreg), regval); // 1 byte to read 
#else
 ICM20948_readData((uint8_t)(bankreg), regval, 1); // 1 byte to read
#endif
      

 return;
}

void ICM20948Class::ICM20948_readBankData(uint16_t bankreg, uint8_t *prdData, uint32_t len)
{
 
 ICM20948_switchBank((uint8_t)(bankreg >> 8));
 
 ICM20948_readData((uint8_t)(bankreg), prdData, len); // 1 byte to read
 
 return;
}


void ICM20948Class::ICM20948_reset()
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_PWR_MGMT_1, &regval);
  if (_last_status > 0) return;  
  
  // DEVICE_RESET [7:7]   Autoclear
  regval |= ICM20948_BIT_RESET;  // set DEVICE_RESET=1
        
  ICM20948_writeBankReg(ICM20948_REG_PWR_MGMT_1, regval);
  return; 
}

void ICM20948Class::ICM20948_resetI2CMaster()
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_USER_CTRL, &regval);
  if (_last_status > 0) return;  
  
  // I2C_MST_RST [1:1]   Autoclear
  regval |= ICM20948_BIT_I2C_MST_RST;  // set I2C_MST_RST [1:1]=1
        
  ICM20948_writeBankReg(ICM20948_REG_USER_CTRL, regval);
  return; 
}


void ICM20948Class::ICM20948_sleep(bool enable)
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_PWR_MGMT_1, &regval);
  if (_last_status > 0) return;  

  // SLEEP [6:6]   
  if (enable)
     regval |= ICM20948_BIT_SLEEP;  // set SLEEP=1
  else
     regval &= ~ICM20948_BIT_SLEEP; // set SLEEP=0
        
  ICM20948_writeBankReg(ICM20948_REG_PWR_MGMT_1, regval);
  return; 
}

void ICM20948Class::ICM20948_lowPower(bool enable)
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_PWR_MGMT_1, &regval);
  if (_last_status > 0) return;  
  
  // LP _EN [5:5]  
  if (enable)
     regval |= ICM20948_BIT_LP_EN;  // set LP_EN=1
  else
     regval &= ~ICM20948_BIT_LP_EN; // set LP_EN=0
   
  ICM20948_writeBankReg(ICM20948_REG_PWR_MGMT_1, regval);
  return; 
}

void ICM20948Class::ICM20948_setClockSource(uint8_t val)
{
  uint8_t regval = 0;
  ICM20948_readBankReg(ICM20948_REG_PWR_MGMT_1, &regval);
  
  if (_last_status > 0) return;  
  
  // CLKSEL[2:0]
  regval &= ~ICM20948_BITS_CLKSEL; // set CLKSEL= 000
  regval |= (val<<0);              // set CLKSEL= val
    
  ICM20948_writeBankReg(ICM20948_REG_PWR_MGMT_1, regval);
  return; 
}

void ICM20948Class::ICM20948_enableDMP(bool enable)
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_USER_CTRL, &regval);
  if (_last_status > 0) return;  
  
  // DMP_EN [7:7]   
  if (enable)
     regval |= ICM20948_BIT_DMP_EN;  // set DMP=1
  else
     regval &= ~ICM20948_BIT_DMP_EN; // set DMP=0
     
  ICM20948_writeBankReg(ICM20948_REG_USER_CTRL, regval);
  return; 
}

void ICM20948Class::ICM20948_enableFIFO(bool enable)
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_USER_CTRL, &regval);
  if (_last_status > 0) return;  
  
  // FIFO_EN [6:6]   
  if (enable)
     regval |= ICM20948_BIT_FIFO_EN;  // set FIFO_EN=1
  else
     regval &= ~ICM20948_BIT_FIFO_EN; // set FIFO_EN=0
 
  ICM20948_writeBankReg(ICM20948_REG_USER_CTRL, regval);
  
  return; 
}

void ICM20948Class::ICM20948_enableWriteFIFO(bool enable)
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_FIFO_EN_1, &regval);
  if (_last_status > 0) return;  
  
  // SLV_3_FIFO_EN[3:3]+ SLV_2_FIFO_EN[2:2]+SLV_1_FIFO_EN[1:1]+SLV_0_FIFO_EN[0:0]  
  if (enable) {
     regval |= ICM20948_BIT_SLV_3_FIFO_EN;  // set SLV_3_FIFO_EN[3:3]=SLV_2_FIFO_EN[2:2]=SLV_1_FIFO_EN[1:1]=SLV_0_FIFO_EN[0:0] = 1
     regval |= ICM20948_BIT_SLV_2_FIFO_EN;
     regval |= ICM20948_BIT_SLV_1_FIFO_EN;  
     regval |= ICM20948_BIT_SLV_0_FIFO_EN;
  }       
  else {
     regval &= ~ICM20948_BIT_SLV_3_FIFO_EN; // set SLV_3_FIFO_EN[3:3]=SLV_2_FIFO_EN[2:2]=SLV_1_FIFO_EN[1:1]=SLV_0_FIFO_EN[0:0] = 0
     regval &= ~ICM20948_BIT_SLV_2_FIFO_EN;
     regval &= ~ICM20948_BIT_SLV_1_FIFO_EN; 
     regval &= ~ICM20948_BIT_SLV_0_FIFO_EN;  
  }

  ICM20948_writeBankReg(ICM20948_REG_FIFO_EN_1, regval);
   
  regval = 0;  
  ICM20948_readBankReg(ICM20948_REG_FIFO_EN_2, &regval);
  if (_last_status > 0) return;  
  
  // ACCEL_FIFO_EN[4:4]+ GYRO_FIFO_EN[3:1]+TEMP_FIFO_EN[0:0]
  if (enable) {
     regval |= ICM20948_BIT_ACCEL_FIFO_EN;  // set ACCEL_FIFO_EN[4:4]+ GYRO_FIFO_EN[3:1]+TEMP_FIFO_EN[0:0] = 1
     regval |= ICM20948_BITS_GYRO_FIFO_EN;
     regval |= ICM20948_BITS_TEMP_FIFO_EN;  
  }       
  else {
     regval &= ~ICM20948_BIT_ACCEL_FIFO_EN; // set ACCEL_FIFO_EN[4:4]+ GYRO_FIFO_EN[3:1]+TEMP_FIFO_EN[0:0] = 0
     regval &= ~ICM20948_BITS_GYRO_FIFO_EN;
     regval &= ~ICM20948_BITS_TEMP_FIFO_EN;  
  }

  ICM20948_writeBankReg(ICM20948_REG_FIFO_EN_2, regval);
  return; 
}

void ICM20948Class::ICM20948_enableFIFOSnapshot(bool enable)
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_FIFO_MODE, &regval);
  if (_last_status > 0) return;  
  
  // FIFO_MODE[4:0]   
  if (enable)
     regval |= ICM20948_BITS_FIFO_OVERFLOW_EN;  // set FIFO_MODE[4:0]=11111
  else
     regval &= ~ICM20948_BITS_FIFO_OVERFLOW_EN; // set FIFO_MODE[4:0]=00000
 
  ICM20948_writeBankReg(ICM20948_REG_FIFO_MODE, regval);
  return; 
}

void ICM20948Class::ICM20948_enableIntFIFOOverflow(bool enable)
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_INT_ENABLE_2, &regval);
  if (_last_status > 0) return;  
  
  // FIFO_OVERFLOW_EN[4:0]   
  if (enable)
     regval |= ICM20948_BITS_FIFO_OVERFLOW_EN;  // set FIFO_OVERFLOW_EN[4:0]=11111
  else
     regval &= ~ICM20948_BITS_FIFO_OVERFLOW_EN; // set FIFO_OVERFLOW_EN[4:0]=00000
 
  ICM20948_writeBankReg(ICM20948_REG_INT_ENABLE_2, regval);
  return; 
}

void ICM20948Class::ICM20948_enableIntRawdataReady(bool enable)
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_INT_ENABLE_1, &regval);
  if (_last_status > 0) return;  
  
  // RAW_DATA_0_RDY_EN[0:0]   
  if (enable)
     regval |= ICM20948_BIT_RAW_DATA_0_RDY_EN;  // set RAW_DATA_0_RDY_EN[0:0]=1
  else
     regval &= ~ICM20948_BIT_RAW_DATA_0_RDY_EN; // set RAW_DATA_0_RDY_EN[0:0]=0

  ICM20948_writeBankReg(ICM20948_REG_INT_ENABLE_1, regval);
  return; 
}

void ICM20948Class::ICM20948_enableI2CMaster(uint8_t val, bool enable)
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_I2C_MST_CTRL, &regval);
  if (_last_status > 0) return;  
  
  // I2C_MST_P_NSR[4:4] + I2C_MST_CLK[3:0]
  if (enable) {
     regval |= ICM20948_BIT_I2C_MST_P_NSR;  // set I2C_MST_P_NSR[4:4]=1
     regval &= ~ICM20948_BITS_I2C_MST_CLK;  // set I2C_MST_CLK[3:0]= 0000
     regval |= (val<<0);                    // set I2C_MST_CLK[3:0]= val
  }   
  else
     regval &= ~ICM20948_BIT_I2C_MST_P_NSR; // set I2C_MST_P_NSR[0:0]=0

  ICM20948_writeBankReg(ICM20948_REG_I2C_MST_CTRL, regval);
  if (_last_status > 0) return;
      
  ICM20948_readBankReg(ICM20948_REG_USER_CTRL, &regval);
  if (_last_status > 0) return; 
    
  // I2C_MST_EN[5:5] + I2C_MST_RST[1:1] Autoclear
  if (enable) {
     regval |= ICM20948_BIT_I2C_MST_EN;  // set I2C_MST_EN[5:5]=1
     regval |= ICM20948_BIT_I2C_MST_RST;  // set I2C_MST_RST[1:1]=1
  } 
  else
     regval &= ~ICM20948_BIT_I2C_MST_EN; // set I2C_MST_EN[5:5]=0    
 
  ICM20948_writeBankReg(ICM20948_REG_USER_CTRL, regval);
  if (_last_status > 0) return;
       
  return; 
}

void ICM20948Class::ICM20948_enable_I2CMasterPassthrough(bool enable)
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_INT_PIN_CFG, &regval);
  if (_last_status > 0) return;  
  
  // BYPASS_EN[1:1]  
  if (enable)
     regval |= ICM20948_BIT_BYPASS_EN;  // set  BYPASS_EN[1:1]=1
  else
     regval &= ~ICM20948_BIT_BYPASS_EN; // set  BYPASS_EN[1:1]=0

  ICM20948_writeBankReg(ICM20948_REG_INT_PIN_CFG, regval);
  if (_last_status > 0) return;
       
  return; 
}
 
void ICM20948Class::ICM20948_resetFIFO()
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_FIFO_RST, &regval);
  if (_last_status > 0) return;  
      
  // Assert and hold to set FIFO size to 0. Assert and de-assert to reset FIFO.
  // FIFO_RESET[4:0]
  regval |= ICM20948_BITS_FIFO_RESET; // setFIFO_RESET [4:0]= 11111  (0x1F)
  ICM20948_writeBankReg(ICM20948_REG_FIFO_RST, regval);
  delay(10);  

  // The InvenSense Nucleo examples write 0x1F followed by 0x1E
  // FIFO_RESET[4:0]
  regval &= ~ICM20948_BITS_FIFO_RESET; // set FIFO_RESET [4:0]= 00000 
  regval |= 0x1E; // setFIFO_RESET [4:0]= 11110  (0x1E) 
  ICM20948_writeBankReg(ICM20948_REG_FIFO_RST, regval);
  delay(10);

  
  return; 
}

void ICM20948Class::ICM20948_resetDMP()
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_USER_CTRL, &regval);
  if (_last_status > 0) return;  
  
  // DMP_RESET[3:3]
  regval |= ICM20948_BIT_DMP_RST; // set DMP_RESET [3:3]= 1
   
  ICM20948_writeBankReg(ICM20948_REG_USER_CTRL, regval);
  return; 
}
    
void ICM20948Class::ICM20948_setFullScaleGyro(uint8_t val)
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_GYRO_CONFIG_1, &regval);
  if (_last_status > 0) return;  
  
  // GYRO_FS_SEL[2:1]  
  regval &= ~ICM20948_BITS_GYRO_FS_SEL; // set GYRO_FS_SEL= 00
  regval |= (val<<1);                  // set GYRO_FS_SEL= val
           
  ICM20948_writeBankReg(ICM20948_REG_GYRO_CONFIG_1, regval);
  return; 
}

void ICM20948Class::ICM20948_enableDLPFGyro(uint8_t val, bool enable) 
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_GYRO_CONFIG_1, &regval);
  if (_last_status > 0) return;  
  
  // GYRO_FCHOICE [0:0] + GYRO_DLPFCFG[5:3]  
  if (enable)
  {
     regval |=  ICM20948_BIT_GYRO_FCHOICE;    // set GYRO_FCHOICE= 1
     regval &= ~ICM20948_BITS_GYRO_DLPCFG;    // set GYRO_DLPFCFG= 000
     regval |= (val<<3);                      // set GYRO_DLPFCFG= val
  }
  else
     regval &= ~ICM20948_BIT_GYRO_FCHOICE;   // set GYRO_FCHOICE= 0
  
  ICM20948_writeBankReg(ICM20948_REG_GYRO_CONFIG_1, regval);
  return; 
}   
    
void ICM20948Class::ICM20948_setSampleRateGyro(uint8_t val)
{
  uint8_t regval = 0;

  // GYRO_SMPLRT_DIV[7:0] 
  regval = val;    
  
  ICM20948_writeBankReg(ICM20948_REG_GYRO_SMPLRT_DIV, regval);
  return; 
}
    
void ICM20948Class::ICM20948_setFullScaleAccel(uint8_t val)
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_ACCEL_CONFIG, &regval);
  if (_last_status > 0) return;  
  
  // ACCEL_FS_SEL[2:1]  
  regval &= ~ICM20948_BITS_ACCEL_FS; // set ACCEL_FS_SEL= 00
  regval |= (val<<1);                // set ACCEL_FS_SEL= val
    
  ICM20948_writeBankReg(ICM20948_REG_ACCEL_CONFIG, regval);
  return; 
}

void ICM20948Class::ICM20948_enableAccel(bool enable) 
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_PWR_MGMT_2, &regval);
  if (_last_status > 0) return;  
  
  // DISABLE_ACCEL [5:3] all 3 axes
  if (enable)
  {
     regval &= ~ICM20948_BITS_DISABLE_ACCEL;  // set DISABLE_ACCEL= 000
  }
  else
     regval |= ICM20948_BITS_DISABLE_ACCEL;   // set DISABLE_ACCEL = 111
  
  ICM20948_writeBankReg(ICM20948_REG_PWR_MGMT_2, regval);
  return; 
} 

void ICM20948Class::ICM20948_enableGyro(bool enable) 
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_PWR_MGMT_2, &regval);
  if (_last_status > 0) return;  
  
  // DISABLE_GYRO [2:0] all 3 axes
  if (enable)
  {
     regval &= ~ICM20948_BITS_DISABLE_GYRO;  // set DISABLE_GYRO= 000
  }
  else
     regval |= ICM20948_BITS_DISABLE_GYRO;   // set DISABLE_GYRO = 111
   
  ICM20948_writeBankReg(ICM20948_REG_PWR_MGMT_2, regval);
  return; 
} 

void ICM20948Class::ICM20948_enableAccelCycledMode(bool enable) 
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_LP_CONFIG, &regval);
  if (_last_status > 0) return;  
  
  // ACCEL_CYCLE [5:5]
  if (enable)
  {
     regval |= ICM20948_BIT_ACCEL_CYCLE;  // set ACCEL_CYCLE= 1
  }
  else
     regval &= ~ICM20948_BIT_ACCEL_CYCLE;   // set ACCEL_CYCLE= 0
   
  ICM20948_writeBankReg(ICM20948_REG_LP_CONFIG, regval);
  return; 
} 

void ICM20948Class::ICM20948_enableGyroCycledMode(bool enable) 
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_LP_CONFIG, &regval);
  if (_last_status > 0) return;  
  
  // ACCEL_CYCLE [4:4]
  if (enable)
  {
     regval |= ICM20948_BIT_GYRO_CYCLE;   // set GYRO_CYCLE= 1
  }
  else
     regval &= ~ICM20948_BIT_GYRO_CYCLE;   // set GYRO_CYCLE= 0
  
  ICM20948_writeBankReg(ICM20948_REG_LP_CONFIG, regval);
  return; 
} 

void ICM20948Class::ICM20948_enableDLPFAccel(uint8_t val, bool enable) 
{
  uint8_t regval = 0;
  
  ICM20948_readBankReg(ICM20948_REG_ACCEL_CONFIG, &regval);
  if (_last_status > 0) return;  
  
  // ACCEL_FCHOICE [0:0] + ACCEL_DLPFCFG[5:3]  
  if (enable)
  {
     regval |=  ICM20948_BIT_ACCEL_FCHOICE; // set ACCEL_FCHOICE= 1 
     regval &= ~ICM20948_BITS_ACCEL_DLPCFG; // set ACCEL_DLPFCFG= 000
     regval |= (val<<3);                    // set ACCEL_DLPFCFG= val
  }
  else
     regval &= ~ICM20948_BIT_ACCEL_FCHOICE;   // set ACCEL_FCHOICE= 0
  
  ICM20948_writeBankReg(ICM20948_REG_ACCEL_CONFIG, regval);
  return; 
} 

void ICM20948Class::ICM20948_setSampleRateAccel(uint16_t val)
{
  uint8_t valMSB = (uint8_t)(val>>8);     // MSB for ACCEL sample rate div.
  uint8_t valLSB = (uint8_t)(val & 0xFF); // LSB for ACCEL sample rate div.
  uint8_t regval = 0;

  
  ICM20948_readBankReg(ICM20948_REG_ACCEL_SMPLRT_DIV_1, &regval);
  if (_last_status > 0) return;  
  
  // ACCEL_SMPLRT_DIV1[3:0] 
  regval &= ~ICM20948_BITS_ACCEL_SMPLRT_DIV_1;  // set ACCEL_SMPLRT_DIV1= 0000
  regval |= valMSB;                             // set ACCEL_SMPLRT_DIV1= valMSB 
 
  
  ICM20948_writeBankReg(ICM20948_REG_ACCEL_SMPLRT_DIV_1, regval);
  if (_last_status > 0) return;
   
  // ACCEL_SMPLRT_DIV2[7:0]
  regval = valLSB;   
  
  ICM20948_writeBankReg(ICM20948_REG_ACCEL_SMPLRT_DIV_2, regval); 
  
  return; 
}  

void ICM20948Class::ICM20948_setDMPstartAddress(uint16_t address) 
{
  uint8_t start_addressMSB = (uint8_t)(address>>8);     // MSB for start_address
  uint8_t start_addressLSB = (uint8_t)(address & 0xFF); // LSB for start_address
   
  ICM20948_writeBankReg(DMP_REG_PRGM_START_ADDRH, start_addressMSB);
  if (_last_status > 0) return;    
  
  ICM20948_writeBankReg(DMP_REG_PRGM_START_ADDRL, start_addressLSB); 
   
  return; 
}

void ICM20948Class::ICM20948_loadDMPFirmware() 
{
  uint16_t write_size;
  uint16_t memaddr;
  const uint8_t *data;
  uint16_t size;
  uint8_t data_cmp[INV_MAX_SERIAL_READ];

  if (_firmware_loaded) return;

  ICM20948_sleep(false); // Make sure chip is awake
  if (_last_status > 0) return; 

  ICM20948_lowPower(false); // Make sure chip is not in low power state
  if (_last_status > 0) return; 
  
  // Write DMP memory
  data = dmp3_image;
  size = sizeof(dmp3_image);
  memaddr = DMP_LOAD_START;
  
  Serial.print(F("ICM20948_loadDMPFirmware - size: "));Serial.print(size);Serial.print(F(" - memaddr: "));Serial.println(memaddr,HEX);         

  uint8_t data_not_pg[INV_MAX_SERIAL_READ]; // Suggested by @HyperKokichi in Issue #63

  while (size > 0)
  {
    write_size = min(size, INV_MAX_SERIAL_WRITE); // Write in chunks of INV_MAX_SERIAL_WRITE
    if (size <= INV_MAX_SERIAL_WRITE) // Write in chunks of INV_MAX_SERIAL_WRITE
       write_size = size;
    else
       write_size = INV_MAX_SERIAL_WRITE;
    
    if ((memaddr & 0xff) + write_size > 0x100) write_size = (memaddr & 0xff) + write_size - 0x100;  // Moved across a bank

    memcpy_P(data_not_pg, data, write_size);  // Suggested by @HyperKokichi in Issue #63
    ICM20948_write_mems(memaddr, write_size, (uint8_t *)data_not_pg);
    if (_last_status > 0) return; 

    data += write_size;
    size -= write_size;
    memaddr += write_size;
  }

  // Verify DMP memory

  data = dmp3_image;
  size = sizeof(dmp3_image);
  memaddr = DMP_LOAD_START;
  while (size > 0)
  {
    write_size = min(size, INV_MAX_SERIAL_READ); // Read in chunks of INV_MAX_SERIAL_READ
    if (size <= INV_MAX_SERIAL_READ) // Read in chunks of INV_MAX_SERIAL_READ
       write_size = size;
    else
       write_size = INV_MAX_SERIAL_READ;
    
    if ((memaddr & 0xff) + write_size > 0x100) write_size = (memaddr & 0xff) + write_size - 0x100; // Moved across a bank
    
    ICM20948_read_mems(memaddr, write_size, data_cmp);
    if (_last_status > 0) return; 

    memcpy_P(data_not_pg, data, write_size);  // Suggested by @HyperKokichi in Issue #63
    if (memcmp(data_cmp, data_not_pg, write_size))
    {
         _last_status = 9;
          Serial.println(F("ICM20948_loadDMPFirmware - error memcmp "));         
         return;
    }
    data += write_size;
    size -= write_size;
    memaddr += write_size;
  }
  Serial.println(F("ICM20948_loadDMPFirmware - memcmp OK"));
  _firmware_loaded = true;        

  //Enable LP_EN since we disabled it at begining of this function.
  ICM20948_lowPower(true); // Put chip into low power state
  if (_last_status > 0) return;    
    
  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  ICM20948_setDMPstartAddress(DMP_START_ADDRESS);
  if (_last_status > 0) return; 
    
  // Set the Hardware Fix Disable register to 0x48
  ICM20948_writeBankReg(DMP_REG_HW_FIX_DISABLE, 0x48); 
  if (_last_status > 0) return;

  return;
}

void ICM20948Class::ICM20948_setGyroSF(uint8_t div, uint32_t gyro_level)
{
  // Read the TIMEBASE_CORRECTION_PLL register
  uint8_t pll = 0; // Signed. Typical value is 0x18
    
  ICM20948_readBankReg(ICM20948_REG_TIMEBASE_CORR_PLL, &pll);
  if (_last_status > 0) return; 
 
  _gyroSFpll = pll; // Record the PLL value so we can debug print it

  // Now calculate the Gyro SF using code taken from the InvenSense example (inv_icm20948_set_gyro_sf)

  uint32_t gyro_sf;

  uint64_t const MagicConstant = 264446880937391LL;
  uint64_t const MagicConstantScale = 100000LL;
  uint64_t ResultLL;

  if (pll & 0x80)
  {
    ResultLL = (MagicConstant * (uint64_t)(1ULL << gyro_level) * (1 + div) / (1270 - (pll & 0x7F)) / MagicConstantScale);
  }
  else
  {
    ResultLL = (MagicConstant * (uint64_t)(1ULL << gyro_level) * (1 + div) / (1270 + pll) / MagicConstantScale);
  }
  /*
	    In above deprecated FP version, worst case arguments can produce a result that overflows a signed long.
	    Here, for such cases, we emulate the FP behavior of setting the result to the maximum positive value, as
	    the compiler's conversion of a u64 to an s32 is simple truncation of the u64's high half, sadly....
	*/
  if (ResultLL > 0x7FFFFFFF)
     gyro_sf = 0x7FFFFFFF;
  else
     gyro_sf = (uint32_t)ResultLL;

  _gyroSF = gyro_sf; // Record value so we can debug print it

  // Finally, write the value to the DMP GYRO_SF register
  uint8_t gyro_sf_reg[4];
  gyro_sf_reg[0] = (uint8_t)(gyro_sf >> 24);
  gyro_sf_reg[1] = (uint8_t)(gyro_sf >> 16);
  gyro_sf_reg[2] = (uint8_t)(gyro_sf >> 8);
  gyro_sf_reg[3] = (uint8_t)(gyro_sf & 0xff);
  ICM20948_write_mems(GYRO_SF, 4, &gyro_sf_reg[0]);
  
  return;
}

void ICM20948Class::ICM20948_enableDMPSensor(uint16_t DMP_sensors)
{    
  uint8_t buf2[2]; 
  
  ICM20948_sleep(false); // Make sure chip is awake
  if (_last_status > 0) return; 

  ICM20948_lowPower(false); // Make sure chip is not in low power state
  if (_last_status > 0) return; 
  
  // Check if Accel, Gyro/Gyro_Calibr or Compass_Calibr/Quat9/GeoMag/Compass are to be enabled.
  // If they are then we need to request the accuracy data via header2.
  // Also check which bits need to be set in the Data Ready Status register.
  uint16_t delta2 = 0;
  uint16_t data_rdy_status = 0;
  
  if ((DMP_sensors & DMP_Data_Output_Control_1_Accel) > 0)
  {
    delta2 |= DMP_Data_Output_Control_2_Accel_Accuracy;
    data_rdy_status |= DMP_Data_ready_Accel;
  }
  if (((DMP_sensors & DMP_Data_Output_Control_1_Gyro_Calibr) > 0) || ((DMP_sensors & DMP_Data_Output_Control_1_Gyro) > 0))
  {
    delta2 |= DMP_Data_Output_Control_2_Gyro_Accuracy;
    data_rdy_status |= DMP_Data_ready_Gyro;
  }
  if (((DMP_sensors & DMP_Data_Output_Control_1_Compass_Calibr) > 0) || ((DMP_sensors & DMP_Data_Output_Control_1_Compass) > 0) || ((DMP_sensors & DMP_Data_Output_Control_1_Quat9) > 0) || ((DMP_sensors & DMP_Data_Output_Control_1_Quat6) > 0) || ((DMP_sensors & DMP_Data_Output_Control_1_Geomag) > 0))
  {
    delta2 |= DMP_Data_Output_Control_2_Compass_Accuracy;
    data_rdy_status |= DMP_Data_ready_Secondary_Compass | DMP_Data_ready_Accel |DMP_Data_ready_Gyro;
  }
  
  buf2[0] = (uint8_t)(DMP_sensors >> 8);   // MSB 
  buf2[1] = (uint8_t)(DMP_sensors & 0xFF); // LSB
  //Serial.print(F("ICM20948_enableDMPSensor - DMP_sensors = "));printBinary(buf2[0]);Serial.print(" - ");printBinary(buf2[1]);Serial.println(" ");   
 
  // Write the sensor control bits into memory address DATA_OUT_CTL1
  ICM20948_write_mems(DATA_OUT_CTL1, 2, &buf2[0]);
  if (_last_status > 0) return; 
  
  // Write the 'header2' sensor control bits into memory address  
  buf2[0] = (uint8_t)(delta2 >> 8);   // MSB 
  buf2[1] = (uint8_t)(delta2 & 0xFF); // LSB
  //Serial.print(F("ICM20948_enableDMPSensor - delta2 = "));printBinary(buf2[0]);Serial.print(" - ");printBinary(buf2[1]);Serial.println(" ");   

  ICM20948_write_mems(DATA_OUT_CTL2, 2, &buf2[0]);
  if (_last_status > 0) return; 

  // Set the DATA_RDY_STATUS register
  buf2[0] = (uint8_t)(data_rdy_status >> 8);   // MSB 
  buf2[1] = (uint8_t)(data_rdy_status & 0xFF); // LSB
  //Serial.print(F("ICM20948_enableDMPSensor - data_rdy_status = "));printBinary(buf2[0]);Serial.print(" - ");printBinary(buf2[1]);Serial.println(" ");   

  ICM20948_write_mems(DATA_RDY_STATUS, 2, &buf2[0]);
  if (_last_status > 0) return; 

  // Set the MOTION_EVENT_CTL register to NO Motion event
  const uint8_t zero[2] = {0x00, 0x00};
  ICM20948_write_mems(MOTION_EVENT_CTL, 2, &zero[0]);
  if (_last_status > 0) return; 

  ICM20948_lowPower(true); // Put chip into low power state
  if (_last_status > 0) return;    
    
 
  return;
}

void ICM20948Class::ICM20948_getFIFOcount(uint16_t *count)
{
  uint8_t ctrlh;
  uint8_t ctrll;
  
  ICM20948_readBankReg(ICM20948_REG_FIFO_COUNT_H, &ctrlh);
  if (_last_status > 0) return; 
  
  ctrlh &= 0x1F; // Datasheet says "FIFO_CNT[12:8]"
  
  ICM20948_readBankReg(ICM20948_REG_FIFO_COUNT_L, &ctrll);
  if (_last_status > 0) return;

  *count = (((uint16_t)ctrlh) << 8) | (uint16_t)ctrll;
 
  return;
}

void ICM20948Class::ICM20948_readFIFO(uint8_t *data, uint8_t len)
{
  ICM20948_readBankData(ICM20948_REG_FIFO_R_W, data, (uint32_t)len);
  if (_last_status > 0) return; 
  
  return;
}
    
uint8_t ICM20948Class::ICM20948_readDMPdataFromFIFO(icm20948_DMP_data_t *data)
{
  uint8_t fifoBytes[icm_20948_DMP_Maximum_Bytes]; // Interim storage for the FIFO data
   
  // Check how much data is in the FIFO
  uint16_t fifo_count = 0;
  ICM20948_getFIFOcount(&fifo_count);
  if (_last_status > 0) return DMP_ERR_OTHER;
  //Serial.print("fifo_count: ");  Serial.println(fifo_count);
  if (fifo_count < icm_20948_DMP_Header_Bytes) return DMP_ERR_NO_HEADER; // Has a 2-byte header arrived?

  // Read the header (2 bytes)
  data->header = 0; // Clear the existing header
  uint16_t aShort = 0;
  ICM20948_readFIFO(&fifoBytes[0], icm_20948_DMP_Header_Bytes);
  if (_last_status > 0) return DMP_ERR_OTHER;
  
  for (int i = 0; i < icm_20948_DMP_Header_Bytes; i++)
  {
    aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8)); // MSB first
  }
  data->header = aShort;                    // Store the header in data->header
  //Serial.print("data->head: ");  Serial.println(aShort,HEX);

  fifo_count -= icm_20948_DMP_Header_Bytes; // Decrement the count
  // If the header indicates a header2 is present then read that now
  data->header2 = 0;                                  // Clear the existing header2
  if ((data->header & DMP_header_bitmap_Header2) > 0) // If the header2 bit is set
  {
    if (fifo_count < icm_20948_DMP_Header2_Bytes) // Check if we need to read the FIFO count again
    {
      ICM20948_getFIFOcount(&fifo_count);
      if (_last_status > 0) return DMP_ERR_OTHER;
    }  
    if (fifo_count < icm_20948_DMP_Header2_Bytes) return DMP_ERR_NO_HEADER2; // Has a 2-byte header2 arrived?

    // Read the header2 (2 bytes)
    aShort = 0;
    ICM20948_readFIFO(&fifoBytes[0], icm_20948_DMP_Header2_Bytes);
    if (_last_status > 0) return DMP_ERR_OTHER;
        
    for (int i = 0; i < icm_20948_DMP_Header2_Bytes; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->header2 = aShort;                    // Store the header2 in data->header2
    fifo_count -= icm_20948_DMP_Header2_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Accel) > 0) // case DMP_header_bitmap_Accel:
  {
    if (fifo_count < icm_20948_DMP_Raw_Accel_Bytes) // Check if we need to read the FIFO count again
    {
      ICM20948_getFIFOcount(&fifo_count);
      if (_last_status > 0) return DMP_ERR_OTHER;
    }
    if (fifo_count < icm_20948_DMP_Raw_Accel_Bytes) return DMP_ERR_NO_ENOUGH_DATA; // not enough data is available
    
    ICM20948_readFIFO(&fifoBytes[0], icm_20948_DMP_Raw_Accel_Bytes);
    if (_last_status > 0) return DMP_ERR_OTHER;
    
    for (int i = 0; i < icm_20948_DMP_Raw_Accel_Bytes; i++)
    {
      data->Raw_Accel.Bytes[DMP_PQuat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Raw_Accel_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Gyro) > 0) // case DMP_header_bitmap_Gyro:
  {
    if (fifo_count < (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes)) // Check if we need to read the FIFO count again
    {
       ICM20948_getFIFOcount(&fifo_count);
       if (_last_status > 0) return DMP_ERR_OTHER;
    }
    if (fifo_count < (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes)) return DMP_ERR_NO_ENOUGH_DATA; // not enough data is available
    
    ICM20948_readFIFO(&fifoBytes[0], (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes));
    if (_last_status > 0) return DMP_ERR_OTHER;
    
    for (int i = 0; i < (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes); i++)
    {
      data->Raw_Gyro.Bytes[DMP_Raw_Gyro_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes); // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Compass) > 0) // case DMP_header_bitmap_Compass:
  {
    if (fifo_count < icm_20948_DMP_Compass_Bytes) // Check if we need to read the FIFO count again
    {
       ICM20948_getFIFOcount(&fifo_count);
       if (_last_status > 0) return DMP_ERR_OTHER;
    }
    if (fifo_count < icm_20948_DMP_Compass_Bytes) return DMP_ERR_NO_ENOUGH_DATA; // not enough data is available

    ICM20948_readFIFO(&fifoBytes[0], icm_20948_DMP_Compass_Bytes);
    if (_last_status > 0) return DMP_ERR_OTHER;
        
    for (int i = 0; i < icm_20948_DMP_Compass_Bytes; i++)
    {
      data->Compass.Bytes[DMP_PQuat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Compass_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Quat6) > 0) // case DMP_header_bitmap_Quat6:
  {
    if (fifo_count < icm_20948_DMP_Quat6_Bytes) // Check if we need to read the FIFO count again
    {
       ICM20948_getFIFOcount( &fifo_count);
       if (_last_status > 0) return DMP_ERR_OTHER;
    }
    if (fifo_count < icm_20948_DMP_Quat6_Bytes) return DMP_ERR_NO_ENOUGH_DATA; // not enough data is available

    ICM20948_readFIFO(&fifoBytes[0], icm_20948_DMP_Quat6_Bytes);
    if (_last_status > 0) return DMP_ERR_OTHER;
    
    for (int i = 0; i < icm_20948_DMP_Quat6_Bytes; i++)
    {
      data->Quat6.Bytes[DMP_Quat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Quat6_Bytes; // Decrement the count
    //Serial.print("fifo_countQuat6: ");  Serial.println(fifo_count);
  }

  if ((data->header & DMP_header_bitmap_Quat9) > 0) // case DMP_header_bitmap_Quat9:
  {
    if (fifo_count < icm_20948_DMP_Quat9_Bytes) // Check if we need to read the FIFO count again
    {
       ICM20948_getFIFOcount(&fifo_count);
       if (_last_status > 0) return DMP_ERR_OTHER;
    }
    if (fifo_count < icm_20948_DMP_Quat9_Bytes)  return DMP_ERR_NO_ENOUGH_DATA; // not enough data is available
   
    ICM20948_readFIFO(&fifoBytes[0], icm_20948_DMP_Quat9_Bytes);
    if (_last_status > 0) return DMP_ERR_OTHER;
    
    for (int i = 0; i < icm_20948_DMP_Quat9_Bytes; i++)
    {
      data->Quat9.Bytes[DMP_Quat9_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Quat9_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Geomag) > 0) // case DMP_header_bitmap_Geomag:
  {
    if (fifo_count < icm_20948_DMP_Geomag_Bytes) // Check if we need to read the FIFO count again
    {
       ICM20948_getFIFOcount(&fifo_count);
       if (_last_status > 0) return DMP_ERR_OTHER;
    }
    if (fifo_count < icm_20948_DMP_Geomag_Bytes) return DMP_ERR_NO_ENOUGH_DATA; // not enough data is available
 
    ICM20948_readFIFO(&fifoBytes[0], icm_20948_DMP_Geomag_Bytes);
    if (_last_status > 0) return DMP_ERR_OTHER;
    
    for (int i = 0; i < icm_20948_DMP_Geomag_Bytes; i++)
    {
      data->Geomag.Bytes[DMP_Quat9_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Geomag_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Gyro_Calibr) > 0) // case DMP_header_bitmap_Gyro_Calibr:
  {
    // lcm20948MPUFifoControl.c suggests icm_20948_DMP_Gyro_Calibr_Bytes is not supported
    // and looking at DMP frames which have the Gyro_Calibr bit set, that certainly seems to be true.
    // So, we'll skip it.
  }

  if ((data->header & DMP_header_bitmap_Compass_Calibr) > 0) // case DMP_header_bitmap_Compass_Calibr:
  {
    if (fifo_count < icm_20948_DMP_Compass_Calibr_Bytes) // Check if we need to read the FIFO count again
    {
       ICM20948_getFIFOcount(&fifo_count);
       if (_last_status > 0) return DMP_ERR_OTHER;
    }
    if (fifo_count < icm_20948_DMP_Compass_Calibr_Bytes) return DMP_ERR_NO_ENOUGH_DATA; // not enough data is available
   
    ICM20948_readFIFO(&fifoBytes[0], icm_20948_DMP_Compass_Calibr_Bytes);
    if (_last_status > 0) return DMP_ERR_OTHER;
    
    for (int i = 0; i < icm_20948_DMP_Compass_Calibr_Bytes; i++)
    {
      data->Compass_Calibr.Bytes[DMP_Quat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Compass_Calibr_Bytes; // Decrement the count
  }
 
  // Now check for header2 features

  if ((data->header2 & DMP_header2_bitmap_Accel_Accuracy) > 0) // case DMP_header2_bitmap_Accel_Accuracy:
  {
    if (fifo_count < icm_20948_DMP_Accel_Accuracy_Bytes) // Check if we need to read the FIFO count again
    {
       ICM20948_getFIFOcount(&fifo_count);
       if (_last_status > 0) return DMP_ERR_OTHER;
    }
    if (fifo_count < icm_20948_DMP_Accel_Accuracy_Bytes)  return DMP_ERR_NO_ENOUGH_DATA; // not enough data is available

    aShort = 0;
    ICM20948_readFIFO(&fifoBytes[0], icm_20948_DMP_Accel_Accuracy_Bytes);
    if (_last_status > 0) return DMP_ERR_OTHER;
    
    for (int i = 0; i < icm_20948_DMP_Accel_Accuracy_Bytes; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Accel_Accuracy = aShort;
    fifo_count -= icm_20948_DMP_Accel_Accuracy_Bytes; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Gyro_Accuracy) > 0) // case DMP_header2_bitmap_Gyro_Accuracy:
  {
    if (fifo_count < icm_20948_DMP_Gyro_Accuracy_Bytes) // Check if we need to read the FIFO count again
    {
       ICM20948_getFIFOcount(&fifo_count);
       if (_last_status > 0) return DMP_ERR_OTHER;
    }
    if (fifo_count < icm_20948_DMP_Gyro_Accuracy_Bytes) return DMP_ERR_NO_ENOUGH_DATA; // not enough data is available
   
    aShort = 0;
    ICM20948_readFIFO(&fifoBytes[0], icm_20948_DMP_Gyro_Accuracy_Bytes);
    if (_last_status > 0) return DMP_ERR_OTHER;
    
    for (int i = 0; i < icm_20948_DMP_Gyro_Accuracy_Bytes; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Gyro_Accuracy = aShort;
    fifo_count -= icm_20948_DMP_Gyro_Accuracy_Bytes; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Compass_Accuracy) > 0) // case DMP_header2_bitmap_Compass_Accuracy:
  {
    if (fifo_count < icm_20948_DMP_Compass_Accuracy_Bytes) // Check if we need to read the FIFO count again
    {
       ICM20948_getFIFOcount(&fifo_count);
       if (_last_status > 0) return DMP_ERR_OTHER;
    }
    if (fifo_count < icm_20948_DMP_Compass_Accuracy_Bytes) return DMP_ERR_NO_ENOUGH_DATA; // not enough data is available

    aShort = 0;
    ICM20948_readFIFO(&fifoBytes[0], icm_20948_DMP_Compass_Accuracy_Bytes);
    if (_last_status > 0) return DMP_ERR_OTHER;
    
    for (int i = 0; i < icm_20948_DMP_Compass_Accuracy_Bytes; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Compass_Accuracy = aShort;
    fifo_count -= icm_20948_DMP_Compass_Accuracy_Bytes; // Decrement the count
  }
  
   // Finally, extract the footer (gyro count)
  if (fifo_count < icm_20948_DMP_Footer_Bytes) // Check if we need to read the FIFO count again
  {
       ICM20948_getFIFOcount(&fifo_count);
       if (_last_status > 0) return DMP_ERR_OTHER;
  }
  if (fifo_count < icm_20948_DMP_Footer_Bytes) return DMP_ERR_NO_ENOUGH_DATA; // not enough data is available
  
  aShort = 0;
  ICM20948_readFIFO(&fifoBytes[0], icm_20948_DMP_Footer_Bytes);
  if (_last_status > 0) return DMP_ERR_OTHER;
  
  for (int i = 0; i < icm_20948_DMP_Footer_Bytes; i++)
  {
    aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
  }
  data->Footer = aShort;
  fifo_count -= icm_20948_DMP_Footer_Bytes; // Decrement the count
  //Serial.print("fifo_countfooter: ");  Serial.println(fifo_count);

  if (fifo_count > 0) return DMP_FIFO_NOT_EMPTY; // Check if there is still data waiting to be read
 
  return DMP_NO_ERR;
}

void ICM20948Class::ICM20948_setDMPDsensorPeriod(enum DMP_ODR_Registers odr_reg, uint16_t interval)
{
  // Set the ODR registers and clear the ODR counter

  // In order to set an ODR for a given sensor data, write 2-byte value to DMP using key defined above for a particular sensor.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate (225Hz) / ODR ) - 1
  // E.g. For a 25Hz ODR rate, value= (225/25) -1 = 8.

  // During run-time, if an ODR is changed, the corresponding rate counter must be reset.
  // To reset, write 2-byte {0,0} to DMP using keys below for a particular sensor:

  uint8_t odr_reg_val[2];
  odr_reg_val[0] = (uint8_t)(interval >> 8);
  odr_reg_val[1] = (uint8_t)(interval & 0xff);

  uint8_t odr_count_zero[2] = {0x00, 0x00};
  
  ICM20948_sleep(false); // Make sure chip is awake
  if (_last_status > 0) return;
    
  ICM20948_lowPower(false); // Make sure chip is not in low power state
  if (_last_status > 0) return;
 
  switch (odr_reg)
  {
  case DMP_ODR_Reg_Cpass_Calibr:
  {
     ICM20948_write_mems(ODR_CPASS_CALIBR, 2, &odr_reg_val[0]);
     ICM20948_write_mems(ODR_CNTR_CPASS_CALIBR, 2, &odr_count_zero[0]);
  }
  break;
  case DMP_ODR_Reg_Gyro_Calibr:
  {
    ICM20948_write_mems(ODR_GYRO_CALIBR, 2, &odr_reg_val[0]);
    ICM20948_write_mems(ODR_CNTR_GYRO_CALIBR, 2, &odr_count_zero[0]);
  }
  break;
  case DMP_ODR_Reg_Pressure:
  {
    ICM20948_write_mems(ODR_PRESSURE, 2, &odr_reg_val[0]);
    ICM20948_write_mems(ODR_CNTR_PRESSURE, 2, &odr_count_zero[0]);
  }
  break;
  case DMP_ODR_Reg_Geomag:
  {
    ICM20948_write_mems(ODR_GEOMAG, 2, &odr_reg_val[0]);
    ICM20948_write_mems(ODR_CNTR_GEOMAG, 2, &odr_count_zero[0]);
  }
  break;
  case DMP_ODR_Reg_PQuat6:
  {
    ICM20948_write_mems(ODR_PQUAT6, 2, &odr_reg_val[0]);
    ICM20948_write_mems(ODR_CNTR_PQUAT6, 2, &odr_count_zero[0]);
  }
  break;
  case DMP_ODR_Reg_Quat9:
  {
    ICM20948_write_mems(ODR_QUAT9, 2, &odr_reg_val[0]);
    ICM20948_write_mems(ODR_CNTR_QUAT9, 2, &odr_count_zero[0]);
  }
  break;
  case DMP_ODR_Reg_Quat6:
  {
    ICM20948_write_mems(ODR_QUAT6, 2, &odr_reg_val[0]);
    ICM20948_write_mems(ODR_CNTR_QUAT6, 2, &odr_count_zero[0]);
  }
  break;
  case DMP_ODR_Reg_ALS:
  {
    ICM20948_write_mems(ODR_ALS, 2, &odr_reg_val[0]);
    ICM20948_write_mems(ODR_CNTR_ALS, 2, &odr_count_zero[0]);
  }
  break;
  case DMP_ODR_Reg_Cpass:
  {
    ICM20948_write_mems(ODR_CPASS, 2, &odr_reg_val[0]);
    ICM20948_write_mems(ODR_CNTR_CPASS, 2, &odr_count_zero[0]);
  }
  break;
  case DMP_ODR_Reg_Gyro:
  {
    ICM20948_write_mems(ODR_GYRO, 2, &odr_reg_val[0]);
    ICM20948_write_mems(ODR_CNTR_GYRO, 2, &odr_count_zero[0]);
  }
  break;
  case DMP_ODR_Reg_Accel:
  {
    ICM20948_write_mems(ODR_ACCEL, 2, &odr_reg_val[0]);
    ICM20948_write_mems(ODR_CNTR_ACCEL, 2, &odr_count_zero[0]);
  }
  break;
  default:
    _last_status = 5;
     Serial.println(F("ICM20948_setDMPDsensorPeriod- error value odr_reg")); 
    break;
  }

  ICM20948_lowPower(true); // Put chip into low power state
  if (_last_status > 0) return;

  return;
}


void ICM20948Class::ICM20948_i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
{
  uint16_t periph_addr_reg;
  uint16_t periph_reg_reg;
  uint16_t periph_ctrl_reg;
  uint16_t periph_do_reg;

  switch (peripheral)
  {
  case 0:
    periph_addr_reg = ICM20948_REG_I2C_SLV0_ADDR;
    periph_reg_reg = ICM20948_REG_I2C_SLV0_REG;
    periph_ctrl_reg = ICM20948_REG_I2C_SLV0_CTRL;
    periph_do_reg = ICM20948_REG_I2C_SLV0_DO;
    break;
  case 1:
    periph_addr_reg = ICM20948_REG_I2C_SLV1_ADDR;
    periph_reg_reg = ICM20948_REG_I2C_SLV1_REG;
    periph_ctrl_reg = ICM20948_REG_I2C_SLV1_CTRL;
    periph_do_reg = ICM20948_REG_I2C_SLV1_DO;
    break;
  case 2:
    periph_addr_reg = ICM20948_REG_I2C_SLV2_ADDR;
    periph_reg_reg = ICM20948_REG_I2C_SLV2_REG;
    periph_ctrl_reg = ICM20948_REG_I2C_SLV2_CTRL;
    periph_do_reg = ICM20948_REG_I2C_SLV2_DO;
    break;
  case 3:
    periph_addr_reg = ICM20948_REG_I2C_SLV3_ADDR;
    periph_reg_reg = ICM20948_REG_I2C_SLV3_REG;
    periph_ctrl_reg = ICM20948_REG_I2C_SLV3_CTRL;
    periph_do_reg = ICM20948_REG_I2C_SLV3_DO;
    break;
  default:
    _last_status = 100;
     Serial.println(F("ICM20948_i2cControllerConfigurePeripheral- error value peripheral")); 
    return;
  }

  // Set the peripheral address and the Rw flag
  ICM_20948_I2C_PERIPHX_ADDR_t address;
  address.ID = addr;
  if (Rw)
  {
    address.RNW = 1;
  }
  else
  {
    address.RNW = 0; // Make sure bit is clear (just in case there is any garbage in that RAM location)
  }
  
  Serial.print(F("ICM20948_i2cControllerConfigurePeripheral- periph_addr_reg:")); Serial.println(periph_addr_reg,HEX);
  Serial.print(F("ICM20948_i2cControllerConfigurePeripheral- sizeof(ICM_20948_I2C_PERIPHX_ADDR_t:")); Serial.println(sizeof(ICM_20948_I2C_PERIPHX_ADDR_t));
  
  ICM20948_writeBankData(periph_addr_reg, (uint8_t *)&address, sizeof(ICM_20948_I2C_PERIPHX_ADDR_t));
  if (_last_status > 0) return;

  // If we are setting up a write, configure the Data Out register too
  if (!Rw)
  {
    ICM_20948_I2C_PERIPHX_DO_t dataOutByte;
    dataOutByte.DO = dataOut;
    
    ICM20948_writeBankData(periph_do_reg, (uint8_t *)&dataOutByte, sizeof(ICM_20948_I2C_PERIPHX_DO_t));
    if (_last_status > 0) return;
  }

  // Set the peripheral sub-address (register address)
  ICM_20948_I2C_PERIPHX_REG_t subaddress;
  subaddress.REG = reg;
  
  ICM20948_writeBankData(periph_reg_reg, (uint8_t *)&subaddress, sizeof(ICM_20948_I2C_PERIPHX_REG_t));
  if (_last_status > 0) return;
    
  // Set up the control info
  ICM_20948_I2C_PERIPHX_CTRL_t ctrl;
  ctrl.LENG = len;
  ctrl.EN = enable;
  ctrl.REG_DIS = data_only;
  ctrl.GRP = grp;
  ctrl.BYTE_SW = swap;
  
  ICM20948_writeBankData(periph_ctrl_reg, (uint8_t *)&ctrl, sizeof(ICM_20948_I2C_PERIPHX_CTRL_t));
  if (_last_status > 0) return;

  return;
}

void ICM20948Class::ICM20948_configureMagnetometer()
{
  // Normally, when the DMP is not enabled, startupMagnetometer (called by startupDefault, which is called by begin) configures the AK09916 magnetometer
  // to run at 100Hz by setting the CNTL2 register (0x31) to 0x08. Then the ICM20948's I2C_SLV0 is configured to read
  // nine bytes from the mag every sample, starting from the STATUS1 register (0x10). ST1 includes the DRDY (Data Ready) bit.
  // Next are the six magnetometer readings (little endian). After a dummy byte, the STATUS2 register (0x18) contains the HOFL (Overflow) bit.
  //
  // But looking very closely at the InvenSense example code, we can see in inv_icm20948_resume_akm (in Icm20948AuxCompassAkm.c) that,
  // when the DMP is running, the magnetometer is set to Single Measurement (SM) mode and that ten bytes are read, starting from the reserved
  // RSV2 register (0x03). The datasheet does not define what registers 0x04 to 0x0C contain. There is definitely some secret sauce in here...
  // The magnetometer data appears to be big endian (not little endian like the HX/Y/Z registers) and starts at register 0x04.
  // We had to examine the I2C traffic between the master and the AK09916 on the AUX_DA and AUX_CL pins to discover this...
  //
  // So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
  
  // 0: use I2C_SLV0
  // AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
  // 10: we read 10 bytes each cycle
  // true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)
  
  ICM20948_i2cControllerConfigurePeripheral(0, AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true, 0);
   //
  // We also need to set up I2C_SLV1 to do the Single Measurement triggering:
  // 1: use I2C_SLV1
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_CNTL2: we start writing here (0x31)
  // 1: not sure why, but the write does not happen if this is set to zero
  // false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
  // AK09916_mode_single: tell I2C_SLV1 to write the Single Measurement command each sample

   ICM20948_i2cControllerConfigurePeripheral(1, AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single);
   if (_last_status > 0) return; 

  // Set the I2C Master ODR configuration
  // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
  // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
  //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
  //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
  //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
  // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
  // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.  
    ICM20948_writeBankReg(ICM20948_REG_I2C_MST_ODR_CONFIG, 0x04);  // Write one byte to the I2C_MST_ODR_CONFIG register  
    if (_last_status > 0) return ;
        
    return;
}

uint8_t ICM20948Class::ICM20948_startupMagnetometer()
{
  Serial.println(F("ICM_20948::startupMagnetometer - Begin startup"));
  

  ICM20948_enable_I2CMasterPassthrough(false); //Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
  if (_last_status > 0) return _last_status;

  ICM20948_enableI2CMaster(ICM20948_I2C_MST_CTRL_CLK_400KHZ, true);  // corresponds to 345.6 kHz, good for up to 400 kHz
  if (_last_status > 0) return _last_status;

  uint8_t regval;
  ICM20948_readBankReg(ICM20948_REG_INT_PIN_CFG, &regval);
  Serial.print(F("ICM20948_startupMagnetometer - ICM20948_REG_INT_PIN_CFG: "));printBinary(regval);Serial.println(F(" "));
  ICM20948_readBankReg(ICM20948_REG_I2C_MST_CTRL, &regval);
  Serial.print(F("ICM20948_startupMagnetometer - ICM20948_REG_I2C_MST_CTRL: "));printBinary(regval);Serial.println(F(" "));
  ICM20948_readBankReg(ICM20948_REG_USER_CTRL, &regval);
  Serial.print(F("ICM20948_startupMagnetometer - ICM20948_REG_USER_CTRL: "));printBinary(regval);Serial.println(F(" "));

  // Reset mag
  uint8_t retval = 0;
  uint8_t SRST = 1;
  // Soft reset on bit 0 (SRST) of CNTL3: CONTROL 3 SRST= “0”: Normal,  SRST= “1”: Reset
  // When “1” is set, all registers are initialized. After reset, SRST bit turns to “0” automatically.
  retval = ICM20948_i2c_controller_periph4_txn(AK09916_I2C_ADDR, AK09916_REG_CNTL3, SRST, false); // Write 1 byte
  if (_last_status > 0) return _last_status;
  Serial.println(F("ICM_20948::startupMagnetometer - Reset mag OK"));
  
  //After a ICM reset the Mag sensor may stop responding over the I2C master
  //Reset the Master I2C until it responds
  uint8_t tries = 0;
  while (tries < MAX_MAGNETOMETER_STARTS)
  {
    tries++;

    //See if we can read the WhoIAm register correctly
    retval = ICM20948_magWhoIAm();
    if (retval == 0)
      break; //WIA matched!

    ICM20948_resetI2CMaster(); //Otherwise, reset the master I2C and try again

    delay(100);
  }

  if (tries == MAX_MAGNETOMETER_STARTS)
  {
    Serial.print(F("ICM_20948::startupMagnetometer: reached MAX_MAGNETOMETER_STARTS: "));
    Serial.print(MAX_MAGNETOMETER_STARTS);
    Serial.println(F(". Returning ICM_20948_Stat_WrongID"));
    return 1; 
  }
  else
  {
    Serial.print(F("ICM_20948::startupMagnetometer: successful magWhoIAm after "));
    Serial.print(tries);
    if (tries == 1)
      Serial.print(F(" try"));
    else
      Serial.println(F(" tries"));
  }

  Serial.println(F("ICM_20948::startupMagnetometer: startup complete!"));

  return 0;
}

uint8_t ICM20948Class::ICM20948_magWhoIAm()
{
  uint8_t whoiam = 0;
 
  whoiam = ICM20948_i2c_controller_periph4_txn(AK09916_I2C_ADDR, AK09916_REG_WIA2, 0x00, true);  // Read 1 byte
  if (_last_status > 0) return 2;

  if (whoiam == AK09916_WHO_AM_I) return 0;
 
  Serial.print(F("ICM_20948::magWhoIAm: "));Serial.print(whoiam);Serial.print(F(" should be: "));Serial.println(AK09916_WHO_AM_I);
  return 1;
}

//Transact directly with an I2C device, one byte at a time
//Used to configure a device before it is setup into a normal 0-3 peripheral slot
uint8_t ICM20948Class::ICM20948_i2c_controller_periph4_txn(uint8_t addr, uint8_t reg, uint8_t data, bool Rw)
{
  uint8_t readdata = 0;
  
  // Thanks MikeFair! // https://github.com/kriswiner/MPU9250/issues/86
   addr = (((Rw) ? 0x80 : 0x00) | addr); 

  ICM20948_writeBankReg(ICM20948_REG_I2C_SLV4_ADDR, addr);
  if (_last_status > 0) return 60;

  ICM20948_writeBankReg(ICM20948_REG_I2C_SLV4_REG, reg);
  if (_last_status > 0) return 60;
 
  if (!Rw)
  {
      ICM20948_writeBankReg(ICM20948_REG_I2C_SLV4_DO, data);
      if (_last_status > 0) return 60;
  }
  // Kick off txn
  ICM20948_writeBankReg(ICM20948_REG_I2C_SLV4_CTRL, ICM20948_I2C_SLV4_CTRL_MAG_VAL);
  if (_last_status > 0) return 60;

  const uint32_t max_cycles = 1000;
  uint32_t count = 0; 
  uint8_t i2c_mst_status = 0;

  uint8_t peripheral4Done = 0;
  while (( !peripheral4Done) && (count < max_cycles))
  {
      delay(10);
      ICM20948_readBankReg(ICM20948_REG_I2C_MST_STATUS, &i2c_mst_status);
      if (_last_status > 0) return 60;

      peripheral4Done = i2c_mst_status & ICM20948_BIT_I2C_SLV4_DONE; 
      count++;
  }
  if (count >= max_cycles)
  {
    //We often fail here if mag is stuck
    _last_status = 50;
    Serial.println(F("ICM20948_i2c_controller_periph4_txn- error Timeout")); 
    return _last_status;
  }  
 
  if (Rw)
  {
       ICM20948_readBankReg(ICM20948_REG_I2C_SLV4_DI, &readdata);
  }

 
  Serial.print(F("ICM20948_i2c_controller_periph4_txn- OK, readdata: ")); Serial.println(readdata);
  return readdata;
}


