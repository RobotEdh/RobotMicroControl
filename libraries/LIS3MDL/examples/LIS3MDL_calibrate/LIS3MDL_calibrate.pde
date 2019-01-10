#include <Arduino.h>
#include <LIS3MDL.h>

#define NB_SAMPLE 1000


LIS3MDLClass LIS3MDL;

int calibrate = 0;
double max_mag_x = -99999;
double max_mag_y = -99999;
double max_mag_z = -99999;
double min_mag_x =  99999;
double min_mag_y =  99999;
double min_mag_z =  99999;

    
    double offset_mag_x = 0.0;
    double offset_mag_y = 0.0;
    double offset_mag_z = 0.0;
    
    double delta_mag_x = 0.0;
    double delta_mag_y = 0.0;
    double delta_mag_z = 0.0;
    double delta_mag   = 0.0; 

    double scale_mag_x = 0.0;
    double scale_mag_y = 0.0;
    double scale_mag_z = 0.0;
    
void setup()
{
  uint8_t status = 0;
  uint8_t devid= 0;
  
  Serial.begin(9600); // initialize serial port
  
  Wire.begin(); // initialize I2C

  devid = LIS3MDL.LIS3MDL_init();
  if (devid > 0)
  {
    status = LIS3MDL.LIS3MDL_getStatus();
    if (status > 0)
    {
       Serial.print("Failed to detect and initialize magnetometer, error:");Serial.println(status);
    } 
    else
    {
       Serial.print("Baddevice ID: 0x"); Serial.print(devid);Serial.print(" should be: 0x"); Serial.println(LIS3MDL_ID);
    } 
  }
  else
  {
    uint8_t address = LIS3MDL.LIS3MDL_getAddress();
    Serial.print("Init OK, Address: 0x"); Serial.println(address,HEX);
  }

}

void loop()
{
    uint8_t status = 0;
    double mag_x, mag_y, mag_z;
    double magnitude;

    
    if (calibrate == 0) {
    Serial.println("Start sample");
    for (int i = 0; i<NB_SAMPLE; i++) {
    
     
       mag_x = LIS3MDL.LIS3MDL_getMag_x();
       status = LIS3MDL.LIS3MDL_getStatus();
       if (status > 0)
       {
          Serial.print("Error LIS3MDL_getMag_x: ");Serial.println(status);
       }
       else
       {
           if      (mag_x > max_mag_x) max_mag_x = mag_x;
           else if (mag_x < min_mag_x) min_mag_x = mag_x;
        
       }
    
       mag_y = LIS3MDL.LIS3MDL_getMag_y();
       status = LIS3MDL.LIS3MDL_getStatus();
       if (status > 0)
       {
          Serial.print("Error LIS3MDL_getMag_y: ");Serial.println(status);
       }
       else
       {
           if      (mag_y > max_mag_y) max_mag_y = mag_y;
           else if (mag_y < min_mag_y) min_mag_y = mag_y;
        
      }
        
       mag_z = LIS3MDL.LIS3MDL_getMag_z();
       status = LIS3MDL.LIS3MDL_getStatus();
       if (status > 0)
       {
          Serial.print("Error LIS3MDL_getMag_z: ");Serial.println(status);
       }   
       else
       {
           if      (mag_z > max_mag_z) max_mag_z = mag_z;
           else if (mag_z < min_mag_z) min_mag_z = mag_z;
        
       }
       
       delay(20);
       
    }  // end for
   

    Serial.println("Stop sample");
 
    offset_mag_x = (min_mag_x + max_mag_x) / 2.0;
    offset_mag_y = (min_mag_y + max_mag_y) / 2.0;
    offset_mag_z = (min_mag_z + max_mag_z) / 2.0;
    
    delta_mag_x = (max_mag_x - min_mag_x) / 2.0;
    delta_mag_y = (max_mag_y - min_mag_y) / 2.0;
    delta_mag_z = (max_mag_z - min_mag_z) / 2.0;
    delta_mag   = (delta_mag_x + delta_mag_y + delta_mag_z) / 3.0; 

    scale_mag_x = delta_mag / delta_mag_x;
    scale_mag_y = delta_mag / delta_mag_y;
    scale_mag_z = delta_mag / delta_mag_z;
    
    calibrate = 1;

    Serial.print("offset_mag_x (gauss): ");Serial.println(offset_mag_x);   
    Serial.print("offset_mag_y (gauss): ");Serial.println(offset_mag_y);   
    Serial.print("offset_mag_z (gauss): ");Serial.println(offset_mag_z);   
    Serial.print("delta_mag_x (gauss): ");Serial.println(delta_mag_x); 
    Serial.print("delta_mag_y (gauss): ");Serial.println(delta_mag_y); 
    Serial.print("delta_mag_z (gauss): ");Serial.println(delta_mag_z);      
    Serial.print("scale_mag_x (gauss): ");Serial.println(scale_mag_x); 
    Serial.print("scale_mag_y (gauss): ");Serial.println(scale_mag_y); 
    Serial.print("scale_mag_z (gauss): ");Serial.println(scale_mag_z); 
    
    } // end if
           
       mag_x = LIS3MDL.LIS3MDL_getMag_x();
       status = LIS3MDL.LIS3MDL_getStatus();
       if (status > 0)
       {
          Serial.print("Error LIS3MDL_getMag_x: ");Serial.println(status);
       }
       else
       {
          double corrected_mag_x = (mag_x- offset_mag_x) * scale_mag_x;
          Serial.print("corrected_mag_x (gauss): ");Serial.println(corrected_mag_x);       
       }
    
       mag_y = LIS3MDL.LIS3MDL_getMag_y();
       status = LIS3MDL.LIS3MDL_getStatus();
       if (status > 0)
       {
          Serial.print("Error LIS3MDL_getMag_y: ");Serial.println(status);
       }
       else
       {
          double corrected_mag_y = (mag_y- offset_mag_y) * scale_mag_y;
          Serial.print("corrected_mag_y (gauss): ");Serial.println(corrected_mag_y);  ;
        
      }
        
       mag_z = LIS3MDL.LIS3MDL_getMag_z();
       status = LIS3MDL.LIS3MDL_getStatus();
       if (status > 0)
       {
          Serial.print("Error LIS3MDL_getMag_z: ");Serial.println(status);
       }   
       else
       {
          double corrected_mag_z = (mag_z- offset_mag_z) * scale_mag_z;
          Serial.print("corrected_mag_z (gauss): ");Serial.println(corrected_mag_z);  
        
       }


    delay(5000);
}
