#ifndef FONCTIONS
#define FONCTIONS

#include <Arduino.h>
#include <Wire.h>  // enable I2C
#include <SPI.h> 

#include <string.h> 
#include <iostream>
#include <bits/stdc++.h>

#include <Ezo_i2c.h> //include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <sequencer2.h> //imports a 2 function sequencer 
#include <sequencer3.h> //imports a 2 function sequencer 
#include <Ezo_i2c_util.h> //brings in common print statements


#define vbatt_pin A0 

//  mode de fonctionnement, valeur par défault en cas de de defaut de lecture du fichier config
extern int led_mode;        // utilise la Led pour controler ce qui se passe
extern int debug_mode;      // envoi les infos sur les liaison serie

// definition pour la carte Atlas
extern char ecData[48];                    // We make a 48 byte character array to hold incoming data from the EC circuit.
extern char salData[48]; 
extern char tdsData[48];
extern char sgData[48];

//extern Ezo_board EC = Ezo_board(100, "EC");      //create an EC circuit object who's address is 100 and name is "EC"



float get_voltage();
  /* Read the battery voltage on the vbatt pin.
   *  If under USB power, but not battery, this function will still produce a value.
   *  return -- a floating point value representing the voltage at the vbatt pin.
   */



void mesureEC();
void setting_ec_probe_type(double probeType);
void enable_ec_parameters(bool ec, bool tds, bool s, bool sg ); 


void mesureEClib();
void request_ec(char computerData[]);




#endif