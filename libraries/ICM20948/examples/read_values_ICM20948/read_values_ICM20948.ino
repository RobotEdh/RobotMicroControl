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

  uint8_t status = 0;
  
  status = ICM20948.ICM20948_startupDefault(false); // don't start Magnetometer
  if (status > 0)
  {
    switch (status) {
        case CHECK_DEVICE_ICM20948_ERROR: Serial.println("ICM20948_startupDefault error: CHECK_DEVICE_ICM20948_ERROR");
             break;
        case CHECK_DEVICE_MAGNETOMETER_ERROR: Serial.println("ICM20948_startupDefault error: CHECK_DEVICE_MAGNETOMETER_ERROR");
             break;
        case CHECK_REGISTER_ERROR: Serial.println("ICM20948_startupDefault error: CHECK_REGISTER_ERROR");
             break;
        default: Serial.print("ICM20948_startupDefault failed, unknown error, status: "); Serial.println(status);
    }  
  }
  else Serial.println("ICM20948_startupDefault OK" );  


  status = ICM20948.ICM20948_initializeDMP(false); // don't config Magnetometer
  if (status > 0)
  {
    switch (status) {
        case DMP_ERR_FIRMWARE_LOADED: Serial.println("ICM20948_initializeDMP error: DMP_ERR_FIRMWARE_LOADED");
             break;
        case CHECK_REGISTER_ERROR: Serial.println("ICM20948_initializeDMP error: CHECK_REGISTER_ERROR");
             break;
        default: Serial.print("ICM20948_initializeDMP failed, unknown error, status: "); Serial.println(status);
    }  
  }
  else Serial.println("ICM20948_initializeDMP OK" );

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

void get_angles()
{
  icm20948_DMP_data_t data;
  uint8_t status = 0;
  double angle [3];
  static uint32_t count = 0;

  // Read any DMP data waiting in the FIFO
  status = ICM20948.ICM20948_readDMPdataFromFIFO(&data);
  if ((status == DMP_NO_ERR) || (status == DMP_FIFO_NOT_EMPTY)) // Was valid data available?
  {     
    if ((data.header & DMP_header_bitmap_Quat6) > 0)
    {
      
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      // Scale to +/- 1
      double qy = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double qx = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double qz = -((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double u = ((qx * qx) + (qy * qy) + (qz * qz));
      double qw;
      if (u < 1.0) qw = sqrt(1.0 - u);
      else         qw = 0.0;      
      
      // The ICM 20948 chip has axes y-forward, x-right and Z-up - see Figure 12:
      // Orientation of Axes of Sensitivity and Polarity of Rotation
      // in DS-000189-ICM-20948-v1.6.pdf  These are the axes for gyro and accel and quat
      //
      // For conversion to roll, pitch and yaw for the equations below, the coordinate frame
      // must be in aircraft reference frame.
      //  
      // We use the Tait Bryan angles (in terms of flight dynamics):
      // ref: https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles
      //
      // Heading – ψ : rotation about the Z-axis (+/- 180 deg.)
      // Pitch – θ : rotation about the new Y-axis (+/- 90 deg.)
      // Bank – ϕ : rotation about the new X-axis  (+/- 180 deg.)
      //
      // where the X-axis points forward (pin 1 on chip), Y-axis to the right and Z-axis downward.
      // In the conversion example above the rotation occurs in the order heading, pitch, bank. 
      // To get the roll, pitch and yaw equations to work properly we need to exchange the axes

      // Note when pitch approaches +/- 90 deg. the heading and bank become less meaningfull because the
      // device is pointing up/down. (Gimbal lock)

      // roll (x-axis rotation)
      double t0 = +2.0 * (qw * qx + qy * qz);
      double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      angle[0] = atan2(t0, t1) * 180.0 / PI;


      // pitch (y-axis rotation)
      double t2 = +2.0 * (qw * qy - qx * qz);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      angle[1] = asin(t2) * 180.0 / PI;

      
      // yaw (z-axis rotation)
      double t3 = +2.0 * (qw * qz + qx * qy);
      double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      angle[2] = atan2(t3, t4) * 180.0 / PI;
     
      count++;
      if (count%100 == 0) {Serial.print("count: "); Serial.print(count);Serial.print("Roll: "); Serial.print(angle[0]);Serial.print(" - Pitch: "); Serial.print(angle[1]);Serial.print(" - Yaw: "); Serial.println(angle[2]);}
      return;
      
    }   // end data.header 
    else
    {   
      // Reset FIFO
      ICM20948.ICM20948_resetFIFO();
      
      return;
    }
  }      
  // Reset FIFO
  ICM20948.ICM20948_resetFIFO();    
    
  return; 
}

void loop()
{
 get_angles();
 delay(20);
}

