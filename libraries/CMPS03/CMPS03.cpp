/*
  CMPS03.cpp - Library for commmunicating with compass CMPS03
  Created by EDH, June 12, 2012.
  Released into the public domain.
*/

#include <Arduino.h> // used for delay function (core lib)
#include <Wire.h>     // used for I2C protocol (lib)

#include <CMPS03.h>

// Constructor
CMPS03Class::CMPS03Class ()
{
}
 
void CMPS03Class::CMPS03_init ()
{

} 
 
    
int CMPS03Class::CMPS03_revision ()
{
    byte ret = 0;
    
    Wire.beginTransmission(CMPS03_ADDRESS); 
	Wire.write((uint8_t)0);                            // use register 0: revision
	
	ret = Wire.endTransmission();           
	    
	delay(1);
	
	ret = Wire.requestFrom(CMPS03_ADDRESS, (int)1);     // request 1 byte
	uint8_t value = Wire.read();
	return (int)value;
}

int CMPS03Class::CMPS03_read ()
{   
    byte ret = 0;
 
    Wire.beginTransmission(CMPS03_ADDRESS);  
	Wire.write((uint8_t)1);                            // use register 1 : direction
	
	ret = Wire.endTransmission();      
		
	delay(1);
	
	ret = Wire.requestFrom(CMPS03_ADDRESS, (int)1);     // request 1 byte
	uint8_t value = Wire.read();
	return (int)value;
}