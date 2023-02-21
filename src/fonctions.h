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
#include <Ezo_i2c_util.h> // brings in common print statements
#include <sequencer2.h>   // imports a 2 function sequencer 

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

/* Pas utilisé pour le moment
extern char ecData[48];                    // We make a 48 byte character array to hold incoming data from the EC circuit.
extern char salData[48]; 
extern char tdsData[48];
extern char sgData[48];
*/

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

/* ---------- Fonctions liées au GPS Grove v1.2 + horloge atomique ----------*/
void init_gps(); 
  /* Initialise la communication avec le GPS en liaison série UART à 9600 bauds */
void scanning_gps_time();
  /* Envoie une demande de lecture au GPS et renvoie la date sur le moniteur série */
void scanning_gps_coord();
  /* Envoie une demande de lecture au GPS et renvoie la position sur le moniteur série */
void print_coord_gps();
  /* Utilisé dans scanning_gps() | Renvoie la position GPS si trame valide */
void print_date_gps();
  /* Utilisé dans scanning_gps() | Renvoie la date et heure si trame valide */

/* ---------- Fonctions liées à la carte SD et au fichier de config ----------*/
void test_sd();
  /* Initialise et teste la carte SD */
void errormessage_sd();
  /* Fait clignoter la led pour indiquer une erreur de carte SD */
void lecture_config();
  /* Lit le fichier de configuration */
void refresh_config_values();
  /* Réactualise les valeurs du programme en fonctions de celles lues dans le fichier config */
void mesure_cycle_to_datachain();
  /* Lance un cycle de mesure et stocke les paramètres mesurés dans datachain */
void save_datachain_to_sd();
  /* Vient écrire le contenu de datachain dans le fichier dataFilename sur la carte SD */

/* ---------- Fonctions liées à la RTC DS3231 Adafruit ----------*/
void reading_rtc();
  /* Vient lire l'horloge RTC et écrire dans les variables utiles */
void set_time_rtc(byte hour, byte min, byte sec);
  /* Initialise manuellement heure:minute:secondes de l'horloge RTC */
void set_date_rtc(byte day_of_month, byte day_of_week, byte month, byte year);
  /* Initialise la date jour/mois/annéé de l'horloge RTC 
  *
  *  Paramètres :
  *   - day_of_month : Numéro de jour dans le mois
  *   - day_of_weeks : Numéro de jour dans la semaine (1-7)
  *   - month : Numéro du mois de l'année
  *   - year : /!\ ne rentrer en paramètre que les 2 derniers digits de l'année en cours  
  * 
  *  Exemple : set_date_rtc(20, 1, 2, 23); //Lundi 20 février 2023
  */
void set_rtc_by_gps();
  /* Initialise la date de l'horloge RTC via les données GPS */
void check_rtc_set();
  /* Regarde si la RTC a été bien été initialisée 
  *   OUI -> rtc_set = true  
  *   NON -> rtc_set = false
  */
/* ---------- Fonctions annexes ----------*/
float get_voltage();
  /* Lit la tension sur la broche VBATT_PIN et retourne la valeur
  *  Constante à modifier définie plus haut (VBATT_PIN)
  */
void scanner_i2c_adress();
  /* Scanne les appareils présents sur le port I2C et renvoie leur adresse */
void led_blinkled(int nbr, int freq);
  /* Fait clignoter la led sur un nombre de cycles et à une fréquence (en seconde) donnés */
void all_sleep();
  /* Endormissement général */
void wake_up();
  /* Réveil général */
void cycle_standard();
  /* Fonction provisoire : cycle de mesure, enchainement d'autres fonctions */




#endif