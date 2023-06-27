#ifndef FONCTIONS
#define FONCTIONS

/* ---------- Librarie calls ----------*/
#include <Arduino.h>
#include <SD.h>           // SD card
#include <DebugLog.h>     // Log info
#include <Wire.h>         // Enable I2C
#include <SPI.h>          // Enable SPI
#include <WiFi.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>

#include <string.h> 
#include <bits/stdc++.h>
#include <stdlib.h>

#include <Ezo_i2c.h>      // include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <Ezo_i2c_util.h> // brings in common print statements

//#include <Adafruit_Sensor.h>
#include "TSYS01.h"       // BlueRobotics temperature sensor
#include <RTClib.h>       // RTC clock
#include <TinyGPS++.h>    // GPS
#include <IridiumSBD.h>   // Iridium module
#include <Adafruit_INA219.h> // Voltage and current sensor

using namespace std;

/* ---------- Definition of constants ----------*/
#define VBATT_PIN A0 // Pin for battery voltage control
#define DELAY 3000   // Sequencer delay

/** Definition of the probe type. See function setting_ec_probe_type(double probeType) */
#define PROBE_TYPE 1.0 

#define FRAME_NUMBER 10      // Number of frames concatenation for sending Iridium (340 bytes max)
#define ACQUISITION_NUMBER 18 // Number of acquistions in each frame (= FRAME_DURATION / 20) (because one acquistion each 20 seconds)
#define FRAME_DURATION 360   // Time of one frame acquisition (360 seconds = 6 minutes)

#define DEBUGLOG_DEFAULT_FILE_LEVEL_TRACE  // Level to write in the Loginfo file
#define DEBUGLOG_DEFAULT_LOG_LEVEL_TRACE   // Level to write in the Loginfo serial port

#define DIAGNOSTICS false

extern bool sending_ok;
extern uint8_t switch_state; // To switch state if MT message tell us

/* ---------- Functions related to the Atlas EC EZO card ----------*/

/** Initiate Atlas EC conductivity sensor */
void initEC();

/** @brief Mesure de conductivité in uS/cm
  * Sends a read request to the EZO EC board and returns the measurement to the serial monitor  
  */
void mesureEC();

/** @brief Defines the type of ec ezo Atlas probe
  *
  * @param PROBE_TYPE defined in the header of functions.h
  * Possible choices : "K0.1"
  *                    "K1.0"
  *                    "K10"
  */  
void setting_ec_probe_type(); 
  
/** @brief Sends the command entered in parameter and returns on the serial port the answer of the sensor ec
  *  
  * @param cmd
  *  Possible choices :
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
void send_ec_cmd_and_response(char cmd[]);

/* ---------- Functions related to the BlueRobotics temperature sensor ----------*/
/** @brief Sends a reading request to the sensor and returns the temperature to the serial monitor */
void mesure_temp();
  
/* ---------- Grove GPS v1.2 + atomic clock related functions ----------*/
/** @brief Initialize the communication with the GPS in UART serial link at 9600 bauds */
void init_gps(); 

/** @brief Sends a read request to the GPS and returns the date to the serial monitor */  
void scanning_gps_time();
  
/** @brief Sends a read request to the GPS and returns the position to the serial monitor */
void scanning_gps_coord();

/** @brief Returns the GPS position if valid frame | Used in scanning_gps() */
void print_coord_gps();
  
/** @brief Returns date and time if valid frame | Used in scanning_gps() */
void print_date_gps();

/** @brief Returns true if GPS signal is available, return false if not */
bool gps_available();
 
/* ---------- Functions related to the SD card and the config file ----------*/
/** @brief Initializes and tests the SD card */
void init_sd();

/** @brief Reads the configuration file */
void lecture_config();

/** @brief Refreshes the program values according to those read in the config file */  
void refresh_config_values();

/** @brief Starts a measurement cycle and stores the measured parameters in datachain */  
void calcul_and_filling_dataframe_write();

/** @brief Writes the content of datachain to the dataFilename file on the SD card */  
void save_dataframe_to_sd();

void save_acquisition_to_sd();

/** @brief Read datalog file in a dataframe_read structure and buffer_read_340 for Iridium sending */
void readSDbinary_to_struct();

/** @brief Read time and GPS data and save it in a buffer for Iridium sending in Recovery cycle */
void readGPS_to_buffer();

void store_data_in_arrays(int acquisition_number);

  
/* ---------- Functions related to the RTC DS3231 Adafruit ----------*/

void init_RTC();

/** @brief Reads the RTC clock and writes to useful variables */
void reading_rtc();

/** @brief Manually initializes the date of the RTC clock
  *
  * @param day_of_month Day number in the month
  * @param day_of_weeks Day of the week number (1-7)
  * @param month Month number of the year
  * @param year  Year @warning /!\ ne rentrer en paramètre que les 2 derniers digits de l'année en cours  
  * 
  *  Example : set_date_rtc(20, 1, 2, 23); //Monday 20 February 2023
  */  
void set_rtc(int day, int month, int year, int hour, int min, int sec);

/** @brief Initialize the date of the RTC via GPS data */  
void set_rtc_by_gps();

/** @brief Check if the RTC has been properly initialized 
  *   If yes -> rtc_set = true  
  *   If no -> rtc_set = false
  * @return 1 if RTC has been correctly initialised, 0 otherwise
  */  
int check_rtc_set();

/** @brief Get UnixTime used for counter */ 
int get_unix_time();


/* ---------- Functions related to INA219 current sensor ----------*/
void init_ina219();
void get_current();  

/* ---------- Functions related to Iridium Rockblock 9603 ----------*/
/** @brief Initialize and test Iridium module */
void init_iridium();

/** @brief Give informations about Iridium module (firmware version + signal quality) */
void print_iridium_infos();

/** @brief Send text data with GPSBuffer by Iridium module in RECOVERY cycle (MO message)
 *         and receive data with receive_buffer (MT message)
*/
void sendreceive_recovery_iridium();

/** @brief Send binary data with buffer_read_340 by Iridium module in DEPLOYED cycle (MO message)
 *         and receive data with receive_buffer (MT message)
*/
void sendreceive_deployed_iridium();

void sendreceive_emergency_iridium();

/** @brief Send text data buffer by Iridium module 
 *  @param text Text to send (between " ")
*/
void send_text_iridium(char text[50]);

/** @brief For multitask and to continue actions even when sending with Iridium */
bool ISBDCallback();

/** @brief Receive data with receive_buffer (MT message) */
void receive_iridium();

/* ---------- Annex functions ----------*/
/** @brief Reads the voltage on the VBATT_PIN pin and returns the value
  * @param VBATT_PIN Constante à modifier définie plus haut 
  * @returns float voltage
  */
float get_voltage();

/** @brief Scans the devices present on the I2C port and returns their address */  
void scanner_i2c_adress();

/** @brief Flashes the LED on a given number of cycles and at a given frequency (in seconds) */  
void led_blinkled(int nbr, int freq);

/** @brief General sleepiness 
 *  @param sleeping_time Time to sleep (in seconds)
*/  
void all_sleep(int sleeping_time);

/** @brief General wake up */  
void all_wakeup();

/** @brief Calculate Root Mean Square value of a size n array*/
float rmsValue(float arr[], int n);


/* ---------- State machine functions ----------*/

/** @brief Initialization mode of the sensors and internal checks / Sleeping mode possible (security if no GPS signal for example) */  
void init_cycle();

/** @brief Classic acquisition mode in deployment */  
void deployed_cycle();

/** @brief Mode with more frequent acquisition/transmission during recovery phases */  
void recovery_cycle();

/** @brief Emergency mode in case of problem (battery, humidity, inconsistent values...) */ 
void emergency_cycle();

/** @brief Init test mode */ 
void test_cycle_init();

/** @brief Test mode */ 
void test_cycle();

#endif