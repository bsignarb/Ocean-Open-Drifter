#ifndef FONCTIONS
#define FONCTIONS

/* ---------- Appel des librairies ----------*/
#include <Arduino.h>
#include <Wire.h>  // Enable I2C
#include <SPI.h>   // Enable SPI

#include <string.h> 
#include <iostream>
#include <bits/stdc++.h>

#include <Ezo_i2c.h>      // include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <sequencer2.h>   // imports a 2 function sequencer 
#include <sequencer3.h>   // imports a 2 function sequencer 
#include <Ezo_i2c_util.h> // brings in common print statements

#include "TSYS01.h"  // Capteur température BlueRobotics
#include <SD.h>      // Carte SD
#include <DS3231.h>  // Horloge RTC

//#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

/* ---------- Définition des constantes ----------*/
#define VBATT_PIN A0 // Broche pour le contrôle de la tension de la batterie
#define DELAY 3000   // Délai du séquenceur 

// Définition du type de sonde | Voir fonction setting_ec_probe_type(double probeType)
#define PROBE_TYPE 1.0 
  
/* Définition des paramètres à renvoyer de la carte Atlas EC EZO (0 -> disabled / 1 -> enabled)
*  Voir fonction void enable_ec_parameters(bool ec, bool tds, bool s, bool sg )
*  Pour le moment 1 seul paramètre possible à la fois */
#define EC_ENABLED 0
#define TDS_ENABLED 0
#define SAL_ENABLED 1
#define SG_ENABLED 0

/*Mode de fonctionnement, valeur par défault en cas de de defaut de lecture du fichier config
extern int led_mode;        // Utilise la Led pour controler ce qui se passe
extern int debug_mode;      // Envoie les infos sur les liaison serie*/

/* Pas utilisé pour le moment
extern char ecData[48];                    // We make a 48 byte character array to hold incoming data from the EC circuit.
extern char salData[48]; 
extern char tdsData[48];
extern char sgData[48];
*/

//extern Ezo_board EC = Ezo_board(100, "EC");      //create an EC circuit object who's address is 100 and name is "EC"

/* ---------- Fonctions liées à la carte Atlas EC EZO ----------*/
void mesureEC();
  /* Envoie une demande de lecture à la car-te EC EZO et renvoie la mesure sur le moniteur série
  *  en fonction des paramètres autorisés à être renvoyés
  *  
  *  Mesures possibles (pour le moment 1 seul paramètre possible à la fois) :
  *   - Electrical conductivity
  *   - Total dissolved solids
  *   - Salinity
  *   - Specific gravity of seawater
  *  
  *  Pour activation ou non des paramètres, voir fonction void enable_ec_parameters(bool ec, bool tds, bool s, bool sg )
  */

void setting_ec_probe_type(); 
  /* Défini le type de sonde ec ezo Atlas
  *
  *   Constante à modifier définie plus haut (PROBE_TYPE)
  * 
  *   Choix possibles : "K0.1"
  *                     "K1.0"
  *                     "K10"
  */

void enable_ec_parameters(bool ec, bool tds, bool s, bool sg ); 
  /* Contôle les paramètres actifs en sortie 
  *  ec  => Electrical conductivity, 
  *  tds => Total dissolved solids, 
  *  sal => Salinity, 
  *  sg  => Specific gravity of seawater
  *
  *  Constantes à modifier définies plus haut (EC_ENABLED, TDS_ENABLED, SAL_ENABLED, SG_ENABLED)
  *
  *  Choix possibles : "1" = enabled
  *                    "0" = disabled
  */

void send_ec_cmd_and_response(char cmd[]);
  /* Envoie la commande entrée en paramètre et renvoie sur le port série la réponse du capteur ec
  *  
  *  Choix possibles :
  *  "I"       => Device firmware information | Trame retournée : "?I,EC,2.15"
  *  "Name,?"  => Device name information | Trame retournée : "?NAME,?r"
  *  "Status"  => Voltage at Vcc pin and reason for last restart | Trame retournée : "?STATUS,P,3.34"
  *              ("p" : powered off | "s" : software reset | "b" : brown out | "w" : watchdog | "u" : unknown)
  *  "Sleep"   => Enter sleep mode/low power (sous 3.3V : passe de 16.85mA à 0.4mA de conso)
  *               Any command waked up device
  *  "I2C,n"   => Sets I2C address and reboots into I2C mode 
  *               n = any number 1 – 127
  *               For example "I2C,100"
  *  "Factory" => Enable factory reset  
  *               Will not take the device out of I2C mode, I2C address will not change
  *  "Baud,n"  => Switch from I2C to UART, for example "Baud,115200"
  *  "I2C,n"   => Sets I2C address and reboots into I2C mode
  *               Default I2C address : 100 (0x64); n = any number 1 – 127
  *               For example "I2C,100"
  *  "L,n"     => "L,1" LED on ; "L,0"LED off 
  *  "Find"    => LED rapidly blinks white, used to help find device 
  */

/* Plus utilisées pour le moment Anciennes fonctions de mesure EC EZO
void mesureEClib(); 
void request_ec(char computerData[]);
*/

/* ---------- Fonctions liées au capteur de température BlueRobotics ----------*/
void mesure_temp();
  /* Envoie une demande de lecture au capteur et renvoie la température sur le moniteur série */


/* ---------- Fonctions liées au GPS Grove v1.2 ----------*/
void init_gps(); 
  /* Initialise la communication avec le GPS en liaison série UART à 9600 bauds */
void scanning_gps();
  /* Envoie une demande de lecture au GPS et renvoie la position GPS et la date sur le moniteur série */
void print_pos_gps();
  /* Utilisé dans scanning_gps() | Renvoie la position GPS si trame valide */
void print_date_gps();
  /* Utilisé dans scanning_gps() | Renvoie la date et heure si trame valide */

/* ---------- Fonctions annexes ----------*/
float get_voltage();
  /* Lit la tension sur la broche VBATT_PIN et retourne la valeur
  *  Constante à modifier définie plus haut (VBATT_PIN)
  */
void scanner_i2c_adress();
  /* Scanne les appareils présents sur le port I2C et renvoie leur adresse */

#endif