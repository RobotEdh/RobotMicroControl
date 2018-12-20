/*
  SOUND.h -  Library for communicating with Sound Detector
  Created by EDH, December 20, 2018.
  Released into the public domain.
*/


#ifndef Sound_h
#define Sound_h

#include <Arduino.h> // used for pin definition

#define SOUND_PIN_DEFAULT A3  // Default Sound Detector analogic pin  A3
/* Analogic interface is provided on pin A3 */
/* Power +5V is set on pin VCC              */
/* Ground    is set on pin GND              */



class SoundClass
{
  public:
    
  SoundClass();
 
  void Sound_init(); 
  void Sound_init(int pin);
  /* Description: Initialize the Sound Detector                                 */                                            
  /* input:       pin                                                           */ 
  /*                  = pin connected to the Sound Detector                     */                       
  /* output:      none                                                          */
  /* lib:         none                                                          */
  
  int Sound_getNoise();
  /* Description: Get noise using the Sound Detector                            */
  /* input:       none                                                          */ 
  /* output:      return                                                        */                            
  /*                  = value between 0 (no noise ) and 1023 (huge noise)       */
  /*                  = -1 if pin is not initialized                            */
  /* lib:         analogRead                                                    */

 private:
 int _Sound_Pin;
  
};

#endif
