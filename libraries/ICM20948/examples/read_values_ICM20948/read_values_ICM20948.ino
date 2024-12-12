#include "arduino.h"
#include "ICM20948.h"

#define SPI_SLAVESELECTED_PIN  53

ICM20948Class ICM20948;     
      


void setup()
{

  Serial.begin(115200); // initialize serial port
  Serial.println("Start setup");
 
  ICM20948.ICM20948_initializeSPI(SPI_SLAVESELECTED_PIN);
  
#ifdef I2C
  ICM20948.ICM20948_initializeI2C();
#endif
  uint8_t result = 0;
  uint8_t status = 0;
  
  status = ICM20948.ICM20948_startupDefault(false); // don't start Magnetometer
  if (status > 0)
  {
    switch (status) {
        case CHECK_DEVICE_ERROR: Serial.println("Init ICM20948 KO, startupDefault error: CHECK_DEVICE_ERROR");
             break;
        case CHECK_STARTUP_ERROR: Serial.println("Init ICM20948 KO, startupDefault error: CHECK_STARTUP_ERROR");
             break;
        default: Serial.print("Init ICM20948 KO, startupDefault failed, status: "); Serial.println(status);
    }  
  }
  else Serial.println("Startup OK" );  


  result = ICM20948.ICM20948_initializeDMP(false); // don't config Magnetometer
  if (result > 0)
  {
    Serial.println("initializeDMP failed");
    status = ICM20948.ICM20948_getLaststatus(); 
    Serial.print("status: ");Serial.println(status);
    Serial.print("result: ");Serial.println(result,HEX);  
  }
  else Serial.println("initializeDMP OK");

  // Enable the DMP Game Rotation Vector sensor (Quat6)
  ICM20948.ICM20948_enableDMPSensor(ICM20948_SENSOR_GAME_ROTATION_VECTOR);
  
   // Configuring DMP to output data at multiple ODRs:
  ICM20948.ICM20948_setDMPDsensorPeriod(DMP_ODR_Reg_Quat6, 0); // Set to the maximum 225 HZ.

  // Enable the FIFO
  ICM20948.ICM20948_enableFIFO(true);

  // Enable the DMP
  ICM20948.ICM20948_enableDMP(true);

  // Reset DMP
  ICM20948.ICM20948_resetDMP(); 
  
  // Reset FIFO
  ICM20948.ICM20948_resetFIFO();
  
  Serial.println("End Init" );
  Serial.println("********" ); 

}


void loop()
{
  unsigned long starttime = 0;
  unsigned long totelapsedtime = 0;
  unsigned long count = 0;
  uint8_t status;

  while (1) {
  // Read any DMP data waiting in the FIFO
  icm20948_DMP_data_t data;
  status = ICM20948.ICM20948_readDMPdataFromFIFO(&data);
  if ((status == DMP_NO_ERR) || (status == DMP_FIFO_NOT_EMPTY)) // Was valid data available?
  { 
    //Serial.print(F("Received data! Header: "));Serial.println( data.header, HEX ); // Print the header in HEX so we can see what data is arriving in the FIFO
    
    if ((data.header & DMP_header_bitmap_Quat6) > 0)
    {
      
      if (starttime > 0) {
         totelapsedtime += (micros() - starttime);
         count++;
      }
      if (count > 1000) {
         Serial.print(F("Elapsed time(micros): "));Serial.println(totelapsedtime/count);
         count = 0;
      }
      starttime = micros();
      
      Serial.print(F("Quat6 data"));
      Serial.print(F("  Q1: "));Serial.print(data.Quat6.Data.Q1);
      Serial.print(F("  Q2: "));Serial.print(data.Quat6.Data.Q2);
      Serial.print(F("  Q3: "));Serial.println(data.Quat6.Data.Q3);
    }   // end data.header 
 }
 else
 {
   switch (status) {
      case DMP_ERR_NO_HEADER:
          Serial.print ("DMP_ERR_NO_HEADER: no header is available");
          break;
      case DMP_ERR_NO_HEADER2:
          Serial.print ("DMP_ERR_NO_HEADER2: no header2 is available");
          break;
      case DMP_ERR_NO_ENOUGH_DATA:
          Serial.print ("DMP_ERR_NO_ENOUGH_DATA: not enough data is available");
          break;
      case DMP_ERR_NO_HEADER_QUAT6:
          Serial.print ("DMP_ERR_NO_HEADER_QUAT6: no header QUAT6 is available");
          break;  
      case DMP_ERR_OTHER:
          Serial.print ("DMP_ERR_OTHER: other error");
          break; 
      default:
          Serial.print ("Unknow error");
          break;                                                  
   }
   delay(10000);
 }
} // end while(1)
}

