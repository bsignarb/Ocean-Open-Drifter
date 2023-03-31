#include "fonctions.h" // Header
using namespace std;

/* ------------------------------------------------- DECLARATIONS ------------------------------------------------------------------*/

/*---------- General declarations ----------*/ 
#define uS_TO_S_FACTOR 1000000ULL       // Conversion factor for micro seconds to seconds 

//const int greenled = 35;               // Information led 
//const int yellowled = 34;              // Information led 
//const int redled = 39;                 // Information led
int led_mode = 1;                        // Use the LED to indicate what's going on
int debug_mode = 1;                      // Sends information to the serial monitor
int nbrMes = 3;                          // Number of measurements to perform (then redefined by the config file)
int bootCount = 0;                       // Useful to have a 1st cycle of writing in file different from the following cycles
int TIME_TO_SLEEP = 5;                   // Duration between each cycle (deep sleep and wakeup)

/*---------- Carte Atlas EC EZO ----------*/
#define ecAddress 100                    // Board address definition for I2C communication

Ezo_board EC = Ezo_board(100, "EC");     // EC object creation of the Ezo_board class with address 100
int ecDelay = 300;                       // Delays definition       
const int commut_EC = 4;                 // Controls power supply of the EC ezo sensor  
float conductivity, total_dissolved_solids, salinity, seawater_gravity;    

/*---------- BlueRobotics temperature sensor ----------*/
TSYS01 sensor_fastTemp;                  // Bluerobotics temperature sensor declaration
float fast_temp;

/*---------- Grove GPS v1.2 + atomic clock ----------*/
#define RXD_GPS 26                       // UART ports declaration for communication with GPS
#define TXD_GPS 27

//HardwareSerial neogps(1);              // Instance creation for GPS module (Hardware version)
SoftwareSerial neogps(RXD_GPS, TXD_GPS); // Instance creation for GPS module (Software version)   
TinyGPSPlus gps;                         // GPS object creation from TinyGPSPlus class

const int commut_gps = 9;                // GPS power switching 
double lattitude, longitude, altitude, vitesse;
int nb_satellites;
String second_gps, minute_gps, hour_gps, day_gps, month_gps, year_gps; // For date format in several variables
String datenum_gps;                                                    // For date format in 1 writing (datenum = "day/month/year")
String datetime_gps;                                                   // For date format in 1 writing  (datetime = "hour:minute:second")
std::string lat;
std::string lng;

/*---------- SD card and config file ----------*/
#define FRAME_NUMBER 7    // Number of frames concatenation for sending Iridium (340 bytes max)

String datachain = "";     // Data string for storing the measured parameters
char outBuffer[60];        // To be removed shortly
String outBuffer_string;   // To be removed shortly
byte outBuffer_byte[55];   // To be removed shortly

int reading_pos = 0;

typedef struct dataframe   // Structure creation for data storage
{
  uint8_t d;
  uint8_t mth;
  uint16_t y;
  uint8_t h;
  uint8_t min;
  uint8_t s;
  float lat;
  float lng;
  float cond;
  float temp;
};
dataframe dataframe_write;                                              // Structure to write data in the file
dataframe dataframe_read;                                               // Structure to read data from the file

uint8_t buffer_read[sizeof(dataframe_read)];                            // Binary reading file buffer for Iridium sending
uint16_t buffer_read_340[FRAME_NUMBER*sizeof(dataframe_read)];          // Binary reading file buffer for Iridium sending (14 dataframes concatenation)

const int cspin_SD=15;                                                  // SPI bus selection signal
String id_logger, number_measures, delay_batch, led_mode_sd, debug_mode_sd ,clef_test; // config.txt file variables
File confFile;                                                          // To read the config.txt file

String fichier_config = "/config.txt";                                  // Name of the configuration file
String dataFilename = "/datalog.txt";                                   // Data file name
String binFilename = "/binfile.bin";

/*---------- RTC DS3231 Adafruit ----------*/
DS3231 Clock;                                                           // Object creation from DS3231 class
bool Century = false;
bool h12;
bool PM;

String second_rtc, minute_rtc, hour_rtc, day_rtc, month_rtc, year_rtc;  // For date format in several variables
String datenum_rtc;                                                     // For date format in 1 writing (datenum = "day/month/year")
String timenum_rtc;                                                     // For date format in 1 writing(datetime = "hour:minute:second")
String datetime_rtc;       

bool rtc_set = false;                                                   // To know if the RTC has been correctly initialized

/*---------- INA219 current sensor Adafruit ----------*/
//Adafruit_INA219 ina219;

/*---------- IridiumSBD Rockblock 9603 ----------*/
#define RXD_IRID 10                // UART ports declaration for communication with Rockblock
#define TXD_IRID 5

//SoftwareSerial ssIridium(5, 10); // RockBLOCK serial port on 10 5 (Software version)
HardwareSerial ssIridium(1);       // RockBLOCK serial port on 10 5 (Hardware version)
IridiumSBD modem(ssIridium, 2);    // RockBLOCK Object creation with SLEEP pin on 2

int signalQuality = -1;
int err;
char version[12];

/* ------------------------------------------------- FONCTIONS ------------------------------------------------------------------*/

/* ---------- Auxiliary functions ----------*/
float get_voltage(){
  float vbatt = analogRead(VBATT_PIN);
  vbatt *= 2;
  vbatt *= 3.3;
  vbatt /= 1024;
  return vbatt;
}

void scanner_i2c_adress()
{
  Serial.println();
  Serial.println("I2C scanner. Scanning ...\n");
  byte count = 0;

  Wire.begin();
  for (byte i = 8; i < 127; i++)
  {
    Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire.endTransmission () == 0)    // Receive 0 = success (ACK response) 
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);     // PCF8574 7 bit address
      Serial.println (")");
      count++;
    }
  }
  Serial.print ("Found ");      
  Serial.print (count, DEC);        // numbers of devices
  Serial.println (" device(s).\n");
}

void led_blinkled(int nbr, int freq) {   // freq en sec = freq interval between to light on
  for (int i=0; i<nbr; i++) {
    //digitalWrite(greenled, HIGH); 
    delay(freq/2);
    //digitalWrite(greenled, LOW);
    delay(freq/2);
  }
}

void all_sleep(){
  if(debug_mode) Serial.println("\n--- All sleep ---\n");
  digitalWrite(LED_BUILTIN, LOW);        // Turn off and keep off the built-in led during deep sleep
  gpio_hold_en(GPIO_NUM_2); 
  //send_ec_cmd_and_response("L,0");     // EC sensor led off
  send_ec_cmd_and_response("Sleep");     // Command line to put ec sensor in sleeping mode
  delay(500); 
  digitalWrite(commut_EC, LOW);     // Switch off power supply
  neogps.println("$PMTK161,0*28");       // Command line to put GPS in sleeping mode

  //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // Set deep sleep duration
  //esp_deep_sleep_start();                // Switch to deep sleep mode
  delay(500);                       
}

void all_wakeup(){
  if(debug_mode) Serial.println("\n--- All wake up ---\n");
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(commut_EC, HIGH);   // Switch on power supply
  //send_ec_cmd_and_response("Sleep");    // Sending any command to wake up ec sensor
  delay(500);    
  //send_ec_cmd_and_response("L,1");    // EC sensor led on
  neogps.println("a");                  // Sending any character to wake up gps
  delay(500);   
}

/* ---------- Functions related to Atlas EC EZO sensor ----------*/
void initEC(){
  pinMode(commut_EC, OUTPUT);   // Switch on power supply of the EC ezo sensor
  digitalWrite(commut_EC, HIGH);
}

void mesureEC(){
  EC.send_read_cmd(); // Sends a read request to the sensor
  delay(1000);
  if (EC_ENABLED){
    EC.receive_read_cmd(); 
    conductivity = EC.get_last_received_reading(); // Returns the last sensor reading in float
    if(debug_mode){
      Serial.print("Conductivité : ");
      Serial.print(conductivity);
      //receive_and_print_reading(EC); 
      Serial.print(" uS/cm");
    }
  }
  else if (TDS_ENABLED){
    EC.receive_read_cmd(); 
    total_dissolved_solids = EC.get_last_received_reading();
    if(debug_mode){
      Serial.print("Total dissolved solids : ");
      Serial.print(total_dissolved_solids);
      //receive_and_print_reading(EC); 
      Serial.print(" ppm");
    }
  }
  else if (SAL_ENABLED){
    EC.receive_read_cmd();
    salinity = EC.get_last_received_reading();
    if(debug_mode){
      Serial.print("Salinity : ");
      Serial.print(salinity);
      //receive_and_print_reading(EC); 
      Serial.print(" PSU (between 0.00 - 42.00)");
    }
  }
  else if (SG_ENABLED){
    EC.receive_read_cmd();
    seawater_gravity = EC.get_last_received_reading();
    if(debug_mode){
      Serial.print("Sea water gravity : ");
      Serial.print(seawater_gravity);
      //receive_and_print_reading(EC); 
      Serial.print(" (between 1.00 - 1.300)");
    }
  }
  if(debug_mode) Serial.println();
}

void setting_ec_probe_type() { 
  EC.send_cmd_with_num("K,", PROBE_TYPE);  // Sends any command with the number appended as a string afterwards
  delay(ecDelay);
  Serial.println("Probe type : ");
  receive_and_print_response(EC);          // Used to handle receiving responses and printing them in a common format
  Serial.println();
}

void enable_ec_parameters(bool ec, bool tds, bool s, bool sg ) { 
  EC.send_cmd_with_num("O,EC,", ec); 
  delay(ecDelay);
  EC.send_cmd_with_num("O,TDS,", tds);
  delay(ecDelay);
  EC.send_cmd_with_num("O,S,", s);
  delay(ecDelay);
  EC.send_cmd_with_num("O,SG,", sg);
  delay(ecDelay);
  EC.send_cmd("O,?");
  delay(ecDelay);
  if(debug_mode){
    Serial.print("Parameters enabled : ");
    receive_and_print_response(EC);
  }
}

void send_ec_cmd_and_response(char cmd[]) { 
  EC.send_cmd((const char*)cmd);
  delay(ecDelay);
  Serial.print("Response : ");
  receive_and_print_response(EC); 
  Serial.println();
}

/* ---------- Functions related to BlueRobotics temperature sensor ----------*/
void mesure_temp(){
  sensor_fastTemp.read();
  fast_temp = sensor_fastTemp.temperature();
  delay(200);
  if (debug_mode) {
    Serial.print("Temperature : ");
    Serial.print(fast_temp);
    Serial.println(" °C");
  }
}

/* ---------- Functions related to Grove GPS v1.2 + atomic clock related functions ----------*/
void init_gps(){
  //neogps.begin(9600, SERIAL_8N1, RXD, TXD); // begin GPS hardware serial
  neogps.begin(9600);                         // begin GPS software serial

  pinMode(commut_gps, OUTPUT); 
  digitalWrite(commut_gps, HIGH);
}

void scanning_gps_time(){
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available()) // If gps data available
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }

  if(newData == true)   
  {
    newData = false;
    print_date_gps();
  }
  else
  {
    if(debug_mode) Serial.println("No Data detected");
  }  
}

void scanning_gps_coord(){
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available()) // If gps data available
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }

  if(newData == true)   
  {
    newData = false;
    print_coord_gps();
  }
  else
  {
    if(debug_mode) Serial.println("No Data detected");
  }  
}

void print_coord_gps()
{     
  if (gps.location.isValid() == 1)
  {
    lattitude = gps.location.lat(); 
    longitude = gps.location.lng();
    nb_satellites = gps.satellites.value();
    altitude = gps.altitude.meters();
    vitesse = gps.speed.kmph();

    std::stringstream stream_lat;
    std::stringstream stream_lng;
    stream_lat << std::fixed << std::setprecision(6) << lattitude; // Set lattitude precision to 6 digits (2 as default in char variable)
    stream_lng << std::fixed << std::setprecision(6) << longitude; // Set longitude precision to 6 digits (2 as default in char variable)
    lat = stream_lat.str();
    lng = stream_lng.str();

    if(debug_mode){
      Serial.print("Donnees GPS : ");
      
      Serial.print("lattitude : ");
      Serial.print(&(lat[0]));        // Pass from std string to char value (to make it easier to display)
      Serial.print(" longitude : ");
      Serial.print(&(lng[0]));        // Pass from std string to char value (to make it easier to display)
      Serial.print(" | Nombre de satellites : ");
      Serial.print(nb_satellites);

      Serial.print(" | Altitude : ");
      Serial.print(altitude);
      Serial.print(" metres");

      Serial.print(" | Vitesse : ");
      Serial.print(vitesse);
      Serial.println(" km/h");
    }
  }
  else
  {
    if(debug_mode) Serial.println("GPS location is not valid");
  }  
}

void print_date_gps(){
  if (gps.date.isValid() == 1)
  {
    day_gps = gps.date.day();
    month_gps = gps.date.month();
    year_gps = gps.date.year();
    
    // Concatenation in datenum to facilitate processing on datachain (datenum = "day/month/year")
    datenum_gps = "";   
    datenum_gps += day_gps; datenum_gps += "/";
    datenum_gps += month_gps; datenum_gps += "/";
    datenum_gps += year_gps; 

    if(debug_mode){
      Serial.print(F("DATE GPS : "));
      Serial.print(datenum_gps);
    }
  }
  else
  {
    if(debug_mode) Serial.println("GPS date is not valid");
  }  

  if (gps.time.isUpdated() == 1)
  {
    hour_gps = gps.time.hour() + 1;
    minute_gps = gps.time.minute();
    second_gps = gps.time.second();

    // Concaténation en datetime pour faciliter le traitement sur datachain (datetime = "hour:minute:second")
    datetime_gps = "";   
    datetime_gps += hour_gps; datetime_gps += ":";
    datetime_gps += minute_gps; datetime_gps += ":";
    datetime_gps += second_gps; 

    if(debug_mode){
      Serial.print(F(" | TIME GPS (UTC+1) : "));
      Serial.println(datetime_gps);
    }
  }
  else
  {
    if(debug_mode) Serial.println("GPS time is not valid");
  }  

}

/* ---------- Functions related to the SD card and the config file ----------*/
void init_sd(){
  if (debug_mode) Serial.print("Initializing SD card... : ");   
  if (!SD.begin(cspin_SD)) {                                           // Checks that the SD card is present and can be initialized (pin 5 by default)            
    if (debug_mode) Serial.println("Card failed, or not present");
    if (led_mode==1){
      errormessage_sd();
    }
  }
  if (debug_mode) Serial.println("card initialized.");  
  delay(300);  

  // Allows to resume the reading of the binary file at the last frame written at each reboot of the card
  File binFile = SD.open(binFilename, FILE_READ);
  reading_pos = binFile.size();
}

void errormessage_sd(){
  for (int i=0; i <= 8; i++){                   
      //digitalWrite(greenled, HIGH); delay(400);
      //led_blinked(3,100);
      //digitalWrite(greenled, LOW); delay (300);
  }
}

void lecture_config(){
  confFile = SD.open(fichier_config, FILE_READ); // Opens config.txt file on SD card

  char phrase[200];
  byte index = 0;
  char x=0;
  String reste = "";
  int k=0;
  
  if (confFile) {
    if(debug_mode) Serial.println("Opening "+fichier_config);
    while (confFile.available()) {
      x = confFile.read();
      if (x!='\n') {       // If no line break
        phrase[index] = x; // Fill phrase[] with the characters of the current line of the file
        index++;
      } else {             // As soon as we have line break, we process phrase[] to extract the information
        if (index != 0) {
          reste = phrase;
          reste = reste.substring(0,index); // Created under string "reste" recopying phrase[] from 0 to index
          index=0;
          // Deleting comments
          k = reste.indexOf(";");           // Returns the index (position) of the character ";"
          if (k!=0) {                       // If it is not at the very beginning of the line
            if (k!=-1) {                    // If it does exist
              reste = reste.substring(0,k); // Creates a sub-string of what is written before ";".
            }
            reste.trim();                   // Removes spaces at the beginning and at the end
            
            // Extract the values to place them in the program variables
            if (reste.indexOf('=') >0) {                             // Equal sign found
              String clef = reste.substring(0,reste.indexOf('='));   // Creates a sub-string of what is before "=".
              String valeur = reste.substring(reste.indexOf('=')+1); // Creates a sub-string of what is after "=".
              if (clef == "id_logger") id_logger = valeur;           // Association of variables
              if (clef == "delay_batch") delay_batch = valeur;
              if (clef == "number_measures") number_measures = valeur;
              if (clef == "debug_mode") debug_mode_sd = valeur;
              if (clef == "led_mode") led_mode_sd = valeur;                        
              if (clef == "clef_test") clef_test = valeur;
            }
          }
        }          
      }
     }
    confFile.close(); // Close config file
  } else {
    Serial.println("error opening "+fichier_config);
    while(1) errormessage_sd();
  }
}

void refresh_config_values(){
  TIME_TO_SLEEP=delay_batch.toInt();  // Association of program variables with those read from the SD card
  nbrMes=number_measures.toInt();            
  debug_mode=debug_mode_sd.toInt();  
  led_mode=led_mode_sd.toInt(); 
}

void mesure_cycle_to_datachain(){
  reading_rtc(); // Date and time acquisition via gps

  // Create a new datachain to store the measured values
  datachain = "";                               // Init datachain
  datachain += datetime_rtc;                    // Write date and time in datachain

  enable_ec_parameters(EC_ENABLED, TDS_ENABLED, SAL_ENABLED, SG_ENABLED); // Controls the active parameters at the output of the conductivity sensor

  // Reads the sensors several times and adds their values to datachain
  for(int n=1; n<=nbrMes; n++){
    if(debug_mode){Serial.print("--- Measure n° : "); Serial.print(n); Serial.print("/");Serial.println(nbrMes);}

    mesureEC();           // Atlas ec ezo sensor measure
    mesure_temp();        // Temperature sensor measure
    scanning_gps_coord(); // Gps coordinates acquisition
    if(debug_mode) Serial.println("");

    datachain += " | (lat)";  
    datachain += &(lat[0]); datachain += " , (long)";     // Writte lattitude in datachain
    datachain += &(lng[0]); datachain += " ; ";     // Writte longitude in datachain
    datachain += fast_temp; datachain += "°C ; ";   // Writte temperature in datachain
    
    // Writing ec ezo unity according to the enabled parameters
    if (EC_ENABLED){ 
      datachain += conductivity; datachain += "uS/cm ";
    }
    else if (TDS_ENABLED){ 
      datachain += total_dissolved_solids; datachain += "ppm ";
    }
    else if (SAL_ENABLED){
      datachain += salinity; datachain += "PSU ";
    }
    else if (SG_ENABLED){
      datachain += seawater_gravity;
    }

    // outBuffer for Iridum sending  (avec caracteres mise en forme)
    /*sprintf(outBuffer, "%02d/%02d/%d,%02d:%02d:%02d,%0.6lf,%0.6lf,%0.2f,%0.2f", 
      gps.date.day(), 
      gps.date.month(),
      gps.date.year(), 
      gps.time.hour()+2, 
      gps.time.minute(), 
      gps.time.second(),
      gps.location.lat(),
      gps.location.lng(),
      conductivity,
      fast_temp);*/

    // outBuffer for Iridum sending  (trame brute)
    sprintf(outBuffer, "%02d%02d%d%02d%02d%02d%0.6lf%0.6lf%0.2f%0.2f", 
      gps.date.day(), 
      gps.date.month(),
      gps.date.year(), 
      gps.time.hour()+2, 
      gps.time.minute(), 
      gps.time.second(),
      gps.location.lat(),
      gps.location.lng(),
      conductivity,
      fast_temp);

    outBuffer_string = outBuffer; // Char to string

    // String to binary
    Serial.print("Length buffer : ");
    Serial.println(outBuffer_string.length());
    Serial.print("Length dataframe : ");
    Serial.println(sizeof(dataframe_write));
    outBuffer_string.getBytes(outBuffer_byte, outBuffer_string.length() + 1);
    Serial.println("Binary outBuffer : "); 
    for (int i = 0; i < outBuffer_string.length() + 1; i++){
      Serial.print(outBuffer_byte[i], BIN);
      Serial.print(" ");
    }
    Serial.println('\n');  
  }

  // Display datachain and outbuffer on terminal
  if (debug_mode) {
    Serial.print("outBuffer completed : "); 
    Serial.println(outBuffer);
    Serial.println();
  }

  // Filling dataframe_write structure
  dataframe_write.d = gps.date.day();
  dataframe_write.mth = gps.date.month();
  dataframe_write.y = gps.date.year();
  dataframe_write.h = gps.time.hour()+2;
  dataframe_write.min = gps.time.minute();
  dataframe_write.s = gps.time.second();
  dataframe_write.lat = gps.location.lat();
  dataframe_write.lng = gps.location.lng();
  dataframe_write.cond = conductivity;
  dataframe_write.temp = fast_temp;

}

void save_datachain_to_sd(){
  //--- Text format saving (.txt) ---//
  File dataFile = SD.open(dataFilename, FILE_APPEND);                           // FILE_APPEND for esp32, FILE_WRITE for arduino
  if (dataFile) {                                                               // If file available, writes the content of the datachain to the file
    dataFile.println(outBuffer);                                                // Classic data buffer saving (more visual in the file .txt)
    dataFile.write((const uint8_t *)&dataframe_write, sizeof(dataframe_write)); // Data structure saving (more practical and optimized but less visual)
    dataFile.println(" ");
    dataFile.close();
    if (debug_mode==1) Serial.println("Textfile created succesfully");
    if (debug_mode==1) {Serial.print("Filename : "); Serial.println(dataFilename); Serial.println();}
  }
  else {                                                                        // If file not opened, display error
    if (debug_mode==1) Serial.println("error opening txt file");
    for (int i=0; i<=5; i++){
        errormessage_sd();
    }
  }

  //--- Binary format saving (.bin) ---//
  File binFile = SD.open(binFilename, FILE_APPEND);                             // Writing mode opening file  
  if (binFile) {                                                                // If file available
    binFile.write((const uint8_t *)&dataframe_write, sizeof(dataframe_write));  // Write the content of the dataframe_write struct in the file
    delay(50);
    binFile.close();
    if (debug_mode) Serial.println("Binary file created succesfully");
    if (debug_mode) {Serial.print("Filename : "); Serial.println(binFilename); Serial.println();}
  }
  else {                                                                        // If file not opened, display error
    if (debug_mode==1) Serial.println("error opening binary file");
    for (int i=0; i<=5; i++){
        errormessage_sd();
    }
  }
  delay(400);
}

void readSDbinary_to_struct(){
  //for(int i=0; i<=FRAME_NUMBER; i++){  
    File binFile = SD.open(binFilename, FILE_READ);                      // Reading mode opening file  
    if (binFile.available()) {                                           // If file available 
      binFile.seek(reading_pos);                                         // Reading from the last frame recorded since starting the uC
      binFile.read((uint8_t *)&dataframe_read, sizeof(dataframe_read));  // Read the content of the file in dataframe_read struct
      memcpy(buffer_read, &dataframe_read, sizeof(dataframe_read));      // Copy dataframe_read struct in uint8_t buffer for Iridium sending
      if(debug_mode){                                                    // Testing if structure is correctly completed
        Serial.print("dataframe_read sample : ");
        Serial.print(dataframe_read.h);
        Serial.print(":");
        Serial.println(dataframe_read.min);
        Serial.print(":");
        Serial.println(dataframe_read.s);
        Serial.println(dataframe_read.temp);
        Serial.print("dataframe_read size : ");
        Serial.println(sizeof(dataframe_read));
        Serial.print("buffer_read size : ");
        Serial.println(sizeof(buffer_read));
      }
      reading_pos += sizeof(dataframe_read);                            // Increment to go to the next dataframe struct in the file
      delay(50);   
    }
    else {                                                              // If file not opened, display error
      if (debug_mode) Serial.println("error opening reading binary file");
      for (int i=0; i<=5; i++) errormessage_sd();
    }

    //for(int j=0; j<= sizeof(buffer_read); j++) buffer_read_340[i*sizeof(buffer_read)+j] += buffer_read[j];   
  //}
  if(debug_mode){
    Serial.print("buffer_read_340 size : ");
    Serial.println(sizeof(buffer_read_340));
  }   
}

/* ---------- Fonctions liées à la RTC DS3231 Adafruit ----------*/
void reading_rtc() {                         
  int sec = Clock.getSecond();
  if (sec < 10) {
    second_rtc = String(0) + String(sec);
  }
  else {
    second_rtc = sec;
  }
  int minu = Clock.getMinute();
  if (minu < 10) {
    minute_rtc = String(0) + String(minu);
  }
  else {
    minute_rtc = minu;
  }
  int heure = Clock.getHour(h12, PM);
  if (heure < 10) {
    hour_rtc = String(0) + String(heure);
  }
  else {
    hour_rtc = heure;
  }
  int jour = Clock.getDate();
  if (jour < 10) {
    day_rtc = String(0) + String(jour);
  }
  else {
    day_rtc = jour;
  }
  int mois = Clock.getMonth(Century);
  if (mois < 10) {
    month_rtc = String(0) + String(mois);
  }
  else {
    month_rtc = mois;
  }
  year_rtc = Clock.getYear();
  datenum_rtc = ""; datenum_rtc += year_rtc; datenum_rtc += month_rtc; datenum_rtc += day_rtc;
  timenum_rtc = ""; timenum_rtc += hour_rtc; timenum_rtc += minute_rtc; timenum_rtc += second_rtc;
  datetime_rtc = ""; datetime_rtc += day_rtc; datetime_rtc += "/"; datetime_rtc += month_rtc; datetime_rtc += "/20"; datetime_rtc += year_rtc; datetime_rtc += " "; datetime_rtc += hour_rtc; datetime_rtc += ":"; datetime_rtc += minute_rtc; datetime_rtc += ":"; datetime_rtc += second_rtc;

  if (debug_mode==1) {
    Serial.println("\nRTC values :");
    Serial.print("Date RTC : "); Serial.println(datenum_rtc);
    Serial.print("Time RTC : "); Serial.println(timenum_rtc);
    Serial.print("DateTime RTC "); Serial.println(datetime_rtc);
    Serial.println();
  }
}

void set_time_rtc(byte hour, byte min, byte sec){
  Clock.setHour(hour);
  Clock.setMinute(min);
  Clock.setSecond(sec);
}

void set_date_rtc(byte day_of_month, byte day_of_week, byte month, byte year){
  Clock.setDate(day_of_month);
  Clock.setDoW(day_of_week);
  Clock.setMonth(month);
  Clock.setYear(year);
}

void set_rtc_by_gps(){
  if(debug_mode) {
    Serial.println("Setting RTC from GPS");
  }
  if (gps.date.isValid() && gps.time.isValid())
  {
    // Updating hour:minute:second
    Clock.setHour(gps.time.hour()+1); 
    Clock.setMinute(gps.time.minute());
    Clock.setSecond(gps.time.second());
    
    // Updating day/month/year 
    Clock.setDate(gps.date.day()); 
    Clock.setMonth(gps.date.month());
    Clock.setYear(gps.date.year()-2000); // Keep only the last 2 digits of the year
    
    if(debug_mode) {Serial.print("RTC set from GPS at ");Serial.println(datetime_rtc);Serial.println();}
  }else{
    if(debug_mode) Serial.println("No GPS fix yet. Cant set RTC yet.\n");
  }  
}

int check_rtc_set()
{
  if(Clock.getYear() != 23)  // If year < 2010 (only the 2 last digits are taken in account)
  {
    rtc_set = false;        // RTC is not correctly set
    Serial.println("RTC time not set\n");
    return 0;
  }else{
    rtc_set = true;         // RTC has been set
    Serial.println("RTC time set\n");
    return 1;
  }
}

/* ---------- Functions related to INA219 current sensor ----------
void init_ina219(){
  if (! ina219.begin()) {
    if(debug_mode) Serial.println("Failed to find INA219 chip");
  }
  else{ 
    if(debug_mode) Serial.println("Correctly found INA219 chip");
  }
}

void get_current(){
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  if(debug_mode){
    Serial.println("------- Measuring voltage and current with INA219 -------");
    Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
    Serial.println("");
  }

  delay(2000);
}
*/

/* ---------- Functions related to Iridium Rockblock 9603 ----------*/
void init_iridium(){

  pinMode(2, OUTPUT); 
  digitalWrite(2, LOW);

  //modem.setPowerProfile(1);                  // This is a low power application
  //ssIridium.begin(19200);                    // Start the serial port connected to the satellite modem (Software version)
  ssIridium.begin(19200, SERIAL_8N1, 10, 5);   // Start the serial port connected to the satellite modem (Hardware version)

  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);

  // Begin satellite modem operation
  Serial.println("Starting modem...");
  err = modem.begin();          // Wake up the modem and prepare it to communicate.
  if (err != ISBD_SUCCESS)
  {
    Serial.print("Begin failed: error ");
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println("No modem detected: check wiring.");
    return;
  }
  else Serial.println("Modem well initialised");
}

void print_iridium_infos(){
  err = modem.getFirmwareVersion(version, sizeof(version)); // Print the firmware revision
  if (err != ISBD_SUCCESS)
  {
     Serial.print("FirmwareVersion failed: error ");
     Serial.println(err);
     return;
  }
  Serial.print("Firmware Version is ");
  Serial.print(version);
  Serial.println(".");

  // Test signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  err = modem.getSignalQuality(signalQuality);
  if (err != ISBD_SUCCESS)
  {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
    return;
  }

  Serial.print("On a scale of 0 to 5, signal quality is currently ");
  Serial.print(signalQuality);
  Serial.println(".");
}

void send_text_iridium(){
  if (debug_mode) Serial.print("Trying to send the text message.  This might take several minutes.\r\n");
  int err = modem.sendSBDText(outBuffer);  // Iridium sending command 
  if (err != ISBD_SUCCESS)
  {
    if(debug_mode){
      Serial.print("Transmission failed with error code ");
      Serial.println(err);
      if (err == ISBD_SENDRECEIVE_TIMEOUT) Serial.println("TimeOut : Try again with a better view of the sky\n");
    }
  }
  else{ if(debug_mode) Serial.println("Text buffer sending Ok\n"); }

  #if DIAGNOSTICS
  void ISBDConsoleCallback(IridiumSBD *device, char c)
  {
    //Serial.write(c);
  }

  void ISBDDiagsCallback(IridiumSBD *device, char c)
  {
    //Serial.write(c);
  }
  #endif
}

void send_binary_iridium(){
  if (debug_mode) Serial.print("Trying to send the binary message.  This might take several minutes.\r\n");
  int err = modem.sendSBDBinary(buffer_read, sizeof(buffer_read)); // Iridium sending command 
  if (err != ISBD_SUCCESS)
  {
    if(debug_mode){
      Serial.print("Transmission failed with error code ");
      Serial.println(err);
      if (err == ISBD_SENDRECEIVE_TIMEOUT) Serial.println("TimeOut : Try again with a better view of the sky\n");
    }
  }
  else{if(debug_mode) Serial.println("Binary buffer sending Ok\n");}

  #if DIAGNOSTICS
  void ISBDConsoleCallback(IridiumSBD *device, char c)
  {
    //Serial.write(c);
  }

  void ISBDDiagsCallback(IridiumSBD *device, char c)
  {
    //Serial.write(c);
  }
  #endif
}