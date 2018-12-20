/*
  Sound.cpp - Library for communicating with Sound Detector
  Created by EDH, December 20, 2018.
  Released into the public domain.
*/

#include <Sound.h>

// Constructor
SoundClass::SoundClass ()
{
}
 
void SoundClass::Sound_init ()
{

 Sound_init(SOUND_PIN_DEFAULT); 
}
 
void SoundClass::Sound_init (int pin)
{

 this->_Sound_Pin = pin;
 pinMode(this->_Sound_Pin, INPUT);   // define pin as input
 analogReference(DEFAULT);
 
 return;
}
  
int SoundClass::Sound_getNoise()
{
 if (!this->_Sound_Pin) return -1;            // pin is not initialized
 return (analogRead(this->_Sound_Pin));       // read analog input pin 
}