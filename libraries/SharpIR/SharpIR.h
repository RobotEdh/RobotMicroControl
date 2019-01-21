/*
	SharpIR

	Arduino library for retrieving distance (in cm) from the analog GP2Y0A21Y and GP2Y0A02YK

	From an original version of Dr. Marcal Casas-Cartagena (marcal.casas@gmail.com)     
	
    Version : 1.0 : Guillaume Rico

	https://github.com/guillaume-rico/SharpIR

*/

#ifndef SharpIR_h
#define SharpIR_h

#include <Arduino.h> // used for pin definition


#define SHARP_IR_PIN_DEFAULT A0  // Default Sharp IR analogic pin  A0
/* Analogic interface is provided on pin A0 */
/* Power +5V is set on pin VCC              */
/* Ground    is set on pin GND              */

#define NB_SAMPLE 25

class SharpIRClass
{
  public:
    
    SharpIRClass ();
    
    void SharpIR_init (long sensorModel);
    void SharpIR_init (int pin, long sensorModel);
    double SharpIR_distance();

  private:

    void SharpIR_sort(int a[], int size);
    
    int _SharpIR_Pin;
    long _model;
};

#endif