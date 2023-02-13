#ifndef FONCTIONS
#define FONCTIONS

#include <Arduino.h>

float get_voltage();
  /* Read the battery voltage on the vbatt pin.
   *  If under USB power, but not battery, this function will still produce a value.
   *  return -- a floating point value representing the voltage at the vbatt pin.
   */

//void mesureEC();



#endif