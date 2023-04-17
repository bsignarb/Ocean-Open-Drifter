#include "fonctions.h" // Header
using namespace std;

/* ------------------------------------------------- DECLARATIONS ------------------------------------------------------------------*/

/*---------- General declarations ----------*/ 
#define uS_TO_S_FACTOR 1000000ULL        // Conversion factor for micro seconds to seconds 
int led_mode = 1;                        // Use the LED to indicate what's going on
int debug_mode = 1;                      // Sends important informations to the serial monitor
int debug_mode2 = 0;                     // Sends less important informations to the serial monitor
int nbrMes = 3;                          // Number of measurements to perform (then redefined by the config file)
int bootCount = 0;                       // Useful to have a 1st cycle of writing in file different from the following cycles
int TIME_TO_SLEEP = 40;                  // Duration between each cycle (deep sleep and wakeup)
extern uint8_t switch_state;

/*---------- Carte Atlas EC EZO ----------*/
#define ecAddress 100                    // Board address definition for I2C communication
Ezo_board EC = Ezo_board(100, "EC");     // EC object creation of the Ezo_board class with address 100
int ecDelay = 300;                       // Delays definition       
const int commut_EC = 4;                 // Controls power supply of the EC ezo sensor  
float conductivity, total_dissolved_solids, salinity, seawater_gravity;  
uint16_t conduct_int, tds_int, sal_int, swg_int;

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
String datachain = "";     // Data string for storing the measured parameters
char outBuffer[60];        // To be removed shortly
String outBuffer_string;   // To be removed shortly
byte outBuffer_byte[55];   // To be removed shortly

char GPSBuffer[60];
int reading_pos = 0;

typedef struct dataframe   // Structure creation for data storage
{
  float lat;
  float lng;
  float temp;
  float cond;
  uint16_t y;
  uint8_t d;
  uint8_t mth;
  uint8_t h;
  uint8_t min;
  uint8_t s;
};
dataframe dataframe_write;                                              // Structure to write data in the file
dataframe dataframe_read;                                               // Structure to read data from the file

uint8_t buffer_read[sizeof(dataframe_read)];                            // Binary reading file buffer for Iridium sending
uint8_t buffer_read_340[FRAME_NUMBER*sizeof(dataframe_read)];           // Binary reading file buffer for Iridium sending (14 dataframes concatenation)

const int cspin_SD=15;                                                  // SPI bus selection signal
String id_logger, number_measures, delay_batch, led_mode_sd, debug_mode_sd ,clef_test; // config.txt file variables
File confFile;                                                          // To read the config.txt file
String fichier_config = "/config.txt";                                  // Name of the configuration file
String dataFilename = "/datalog.txt";                                   // Text data file name
String binFilename = "/binfile.bin";                                    // Binary data file name

/*---------- RTC DS3231 Adafruit ----------*/
RTC_DS3231 rtc;
bool Century = false;
bool h12;
bool PM;
uint8_t uint_secRTC, uint_minRTC, uint_hourRTC, uint_dayRTC, uint_monthRTC;
uint16_t uint_yearRTC;
String second_rtc, minute_rtc, hour_rtc, day_rtc, month_rtc, year_rtc;  // For date format in several variables
String datenum_rtc;                                                     // For date format in 1 writing (datenum = "day/month/year")
String timenum_rtc;                                                     // For date format in 1 writing(datetime = "hour:minute:second")
String datetime_rtc;       
uint8_t drifting_time;                                                      // Drifting between RTC and GPS seconds to know if RTC must be adjust or not

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
bool sending_ok = false;
const int commut_irid = 2;         // Rockblock sleeping pin

static bool messageSent = false;
uint8_t receive_buffer[200];
size_t receive_buffer_size = sizeof(receive_buffer);

/* ------------------------------------------------- FONCTIONS ------------------------------------------------------------------*/

/* ---------- Auxiliary functions ----------*/
float get_voltage(){
  float vbatt = (analogRead(VBATT_PIN) * 3.30) / 4095.00;
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
      Serial.print (i, HEX);             // PCF8574 7 bit address
      Serial.println (")");
      count++;
    }
  }
  Serial.print ("Found ");      
  Serial.print (count, DEC);             // Numbers of devices found
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
  digitalWrite(commut_irid, LOW);        // Iridum modem sleeping
  digitalWrite(commut_EC, LOW);          // EC sensor sleeping
  digitalWrite(commut_gps, LOW);         // GPS sleeping
  digitalWrite(LED_BUILTIN, LOW);        // Turn off the built-in led
  gpio_hold_en(GPIO_NUM_2);              // Keep off the built-in led during deep sleep
  //neogps.println("$PMTK161,0*28");     // Command line to put GPS in sleeping mode
  //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // Set deep sleep duration
  //esp_deep_sleep_start();              // Switch to deep sleep mode
  delay(500);                       
}

void all_wakeup(){
  if(debug_mode) Serial.println("\n--- All wake up ---\n");
  digitalWrite(commut_EC, HIGH);         // EC sensor waking up
  digitalWrite(commut_gps, HIGH);        // GPS waking up
  digitalWrite(commut_irid, HIGH);       // Iridium modem waking up
  //neogps.println("a");                 // Sending any character to wake up gps
  delay(500);   
}

/* ---------- Functions related to Atlas EC EZO sensor ----------*/
void initEC(){
  pinMode(commut_EC, OUTPUT);   
  digitalWrite(commut_EC, HIGH); // Switch on power supply of the EC ezo sensor
}

void mesureEC(){
  EC.send_read_cmd(); // Sends a read request to the sensor
  delay(1000);
  // Depends on which parameter will be returned
  if (EC_ENABLED){                                 // Conductivity
    EC.receive_read_cmd(); 
    conductivity = EC.get_last_received_reading(); // Returns the last sensor reading in float
    conduct_int = (uint16_t) conductivity;         // Conversion in uint16_16 to save space for Iridium sending
    if(debug_mode){
      Serial.print("Conductivité : ");
      Serial.print(conductivity);
      //receive_and_print_reading(EC); 
      Serial.print(" uS/cm");
    }
  }
  else if (TDS_ENABLED){                           // Total dissolved solids
    EC.receive_read_cmd(); 
    total_dissolved_solids = EC.get_last_received_reading();
    tds_int = (uint16_t) total_dissolved_solids;
    if(debug_mode){
      Serial.print("Total dissolved solids : ");
      Serial.print(total_dissolved_solids);
      //receive_and_print_reading(EC); 
      Serial.print(" ppm");
    }
  }
  else if (SAL_ENABLED){                           // Salinity
    EC.receive_read_cmd();
    salinity = EC.get_last_received_reading();
    sal_int = (uint16_t) salinity;
    if(debug_mode){
      Serial.print("Salinity : ");
      Serial.print(salinity);
      //receive_and_print_reading(EC); 
      Serial.print(" PSU (between 0.00 - 42.00)");
    }
  }
  else if (SG_ENABLED){                            // Sea water gravity
    EC.receive_read_cmd();
    seawater_gravity = EC.get_last_received_reading();
    swg_int = (uint16_t) seawater_gravity;
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
  if(debug_mode2) Serial.println("Probe type : ");
  receive_and_print_response(EC);          // Used to handle receiving responses and printing them in a common format
  if(debug_mode2) Serial.println();
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
  if(debug_mode2){
    Serial.print("Parameters enabled : ");
    receive_and_print_response(EC);
  }
}

void send_ec_cmd_and_response(char cmd[]) { 
  EC.send_cmd((const char*)cmd);
  delay(ecDelay);
  if(debug_mode) Serial.print("Response : ");
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
      Serial.println(&(lng[0]));      // Pass from std string to char value (to make it easier to display)
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

  if (gps.time.isValid() == 1)
  {
    hour_gps = gps.time.hour() + 2;
    minute_gps = gps.time.minute();
    second_gps = gps.time.second();
    // Concatenation in datenum to facilitate processing on datachain (datetime = "hour:minute:second")
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
  reading_rtc(); // Date and time acquisition via RTC

  // Create a new datachain to store the measured values
  datachain = "";                               // Init datachain
  datachain += datetime_rtc;                    // Write date and time in datachain

  enable_ec_parameters(EC_ENABLED, TDS_ENABLED, SAL_ENABLED, SG_ENABLED); // Controls the active parameters at the output of the conductivity sensor

  // Reads the sensors and add their values to datachain 
  if(debug_mode2) Serial.println("Measure : ");

  mesureEC();           // Atlas ec ezo sensor measure
  mesure_temp();        // Temperature sensor measure
  scanning_gps_coord(); // Gps coordinates acquisition
  if(debug_mode2) Serial.println("");

  datachain += " | (lat)";  
  datachain += &(lat[0]); datachain += " , (long)"; // Writte lattitude in datachain
  datachain += &(lng[0]); datachain += " ; ";       // Writte longitude in datachain
  datachain += fast_temp; datachain += "°C ; ";     // Writte temperature in datachain
  
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

  // outBuffer for Iridum sending (trame brute)
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

  // Filling dataframe_write structure
  dataframe_write.d = uint_dayRTC;
  dataframe_write.mth = uint_monthRTC;
  dataframe_write.y = uint_yearRTC;
  dataframe_write.h = uint_hourRTC;
  dataframe_write.min = uint_minRTC;
  dataframe_write.s = uint_secRTC;
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
    if (debug_mode2) Serial.println("Textfile created succesfully");
    if (debug_mode2) {Serial.print("Filename : "); Serial.println(dataFilename); Serial.println();}
  }
  else {                                                                        // If file not opened, display error
    if (debug_mode) Serial.println("error opening txt file");
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
    
    if (debug_mode2) Serial.println("Binary file created succesfully");
    if (debug_mode2) {Serial.print("Filename : "); Serial.println(binFilename); Serial.println();}
  }
  else {                                                                        // If file not opened, display error
    if (debug_mode) Serial.println("error opening binary file");
    for (int i=0; i<=5; i++){
        errormessage_sd();
    }
  }
  delay(400);
}

void readSDbinary_to_struct(){
  Serial.println("------ READING SD binary to struct ------");
  for(int i=0; i<=FRAME_NUMBER; i++){                                    // Concatenation of n buffer_read in buffer_read_340 
    File binFile = SD.open(binFilename, FILE_READ);                      // Reading mode opening file  
    if (binFile.available()) {                                           // If file available 
      binFile.seek(reading_pos);                                         // Reading from the last frame recorded since starting the uC
      binFile.read((uint8_t *)&dataframe_read, sizeof(dataframe_read));  // Read the content of the file in dataframe_read struct
      //binFile.read((uint8_t *)&buffer_read, sizeof(dataframe_read));
      memcpy(buffer_read, &dataframe_read, sizeof(dataframe_read));      // Copy dataframe_read struct in uint8_t buffer for Iridium sending
      reading_pos += sizeof(dataframe_read);                             // Increment to go to the next dataframe struct in the file
      delay(50);   
    }
    else {                                                               // If file not opened, display error
      if (debug_mode) Serial.println("error opening reading binary file");
      for (int i=0; i<=5; i++) errormessage_sd();
    }
    
    for(int j=0; j<=sizeof(buffer_read); j++) buffer_read_340[i*sizeof(buffer_read)+j] = buffer_read[j];   // Push buffer_read in buffer_read_340 
  }                                                                     
  if(debug_mode){                                                        // Testing if structure and buffers are correctly completed
    Serial.println("dataframe_read sample : ");
    Serial.print("lat : ");
    Serial.println(dataframe_read.lat);
    Serial.print("lng : ");
    Serial.println(dataframe_read.lng);
    Serial.print("temp : ");
    Serial.println(dataframe_read.temp);
    Serial.print("cond : ");
    Serial.println(dataframe_read.cond);
    Serial.print(dataframe_read.y); 
    Serial.print("/");
    Serial.print(dataframe_read.d);
    Serial.print("/");
    Serial.println(dataframe_read.mth);
    Serial.print(dataframe_read.h);
    Serial.print(":");
    Serial.print(dataframe_read.min);
    Serial.print(":");
    Serial.println(dataframe_read.s);
    Serial.print("dataframe_read size : ");
    Serial.println(sizeof(dataframe_read));
    Serial.print("dataframe_read : ");
    for(int i = 0; i < sizeof(dataframe_read); i++) Serial.printf("%02x", (unsigned int) ((char*)&dataframe_read)[i]);
    Serial.println("\n");
    Serial.print("buffer_read size : ");
    Serial.println(sizeof(buffer_read));
    Serial.print("buffer_read : ");
    for(int i=0; i<=sizeof(buffer_read); i++) Serial.printf("%02x", buffer_read[i], HEX);  
    Serial.println("\n");
    Serial.print("buffer_read_340 size : ");
    Serial.println(sizeof(buffer_read_340));
    Serial.print("buffer_read_340 : ");
    for(int i=0; i<=sizeof(buffer_read_340); i++) Serial.printf("%02x", buffer_read_340[i], HEX);  
    Serial.println("\n");
  }   
}

void readGPS_to_buffer(){
  sprintf(GPSBuffer, "%02d/%02d/%d; %02d:%02d:%02d; %0.6lf %0.6lf", 
    gps.date.day(), 
    gps.date.month(),
    gps.date.year(), 
    gps.time.hour()+2, 
    gps.time.minute(), 
    gps.time.second(),
    gps.location.lat(),
    gps.location.lng());
  //outBuffer_string = outBuffer; // Char to string
}

/* ---------- Fonctions liées à la RTC DS3231 Adafruit ----------*/
void init_RTC(){
  rtc.begin();
}

void reading_rtc() {     
  DateTime now = rtc.now();

  uint_secRTC = now.second();
  if (uint_secRTC < 10) {
    second_rtc = String(0) + String(uint_secRTC);
  }
  else {
    second_rtc = uint_secRTC;
  }
  uint_minRTC = now.minute();
  if (uint_minRTC < 10) {
    minute_rtc = String(0) + String(uint_minRTC);
  }
  else {
    minute_rtc = uint_minRTC;
  }
  uint_hourRTC = now.hour();
  if (uint_hourRTC < 10) {
    hour_rtc = String(0) + String(uint_hourRTC);
  }
  else {
    hour_rtc = uint_hourRTC;
  }
  uint_dayRTC = now.day();
  if (uint_dayRTC < 10) {
    day_rtc = String(0) + String(uint_dayRTC);
  }
  else {
    day_rtc = uint_dayRTC;
  }
  uint_monthRTC = now.month();
  if (uint_monthRTC < 10) {
    month_rtc = String(0) + String(uint_monthRTC);
  }
  else {
    month_rtc = uint_monthRTC;
  }
  uint_yearRTC = now.year();
  year_rtc = uint_yearRTC;

  datenum_rtc = ""; datenum_rtc += year_rtc; datenum_rtc += month_rtc; datenum_rtc += day_rtc;
  timenum_rtc = ""; timenum_rtc += hour_rtc; timenum_rtc += minute_rtc; timenum_rtc += second_rtc;
  datetime_rtc = ""; datetime_rtc += day_rtc; datetime_rtc += "/"; datetime_rtc += month_rtc; datetime_rtc += "/"; datetime_rtc += year_rtc; datetime_rtc += " "; datetime_rtc += hour_rtc; datetime_rtc += ":"; datetime_rtc += minute_rtc; datetime_rtc += ":"; datetime_rtc += second_rtc;

  if (debug_mode) {
    if(debug_mode2) Serial.println("\nRTC values :");
    if(debug_mode2) { Serial.print("Date RTC : "); Serial.println(datenum_rtc); }
    if(debug_mode2) { Serial.print("Time RTC : "); Serial.println(timenum_rtc); }
    Serial.print("DateTime RTC "); Serial.println(datetime_rtc);
  }
}

void set_rtc(int day, int month, int year, int hour, int min, int sec){
  rtc.adjust(DateTime(year, 
                      month, 
                      day, 
                      hour, 
                      min, 
                      sec));
}

void set_rtc_by_gps(){
  if(debug_mode) {
    Serial.println("Setting RTC from GPS");
  }
  print_date_gps();   // Reading GPS date
  if (gps.date.isValid() && gps.time.isValid())
  {
    rtc.adjust(DateTime(gps.date.year()-2000, 
                        gps.date.month(), 
                        gps.date.day(), 
                        gps.time.hour()+2, 
                        gps.time.minute(), 
                        gps.time.second()));
    reading_rtc();  //test
    if(debug_mode) {Serial.print("RTC set from GPS at ");Serial.println(datetime_rtc);Serial.println();}
  }else{
    if(debug_mode) Serial.println("No GPS fix yet. Cant set RTC yet.\n");
  }  
}

int check_rtc_set()
{
  DateTime now = rtc.now();                               // Reading RTC time
  print_date_gps();                                       // Reading GPS time
  if (gps.date.isValid() && gps.time.isValid())
  {
    drifting_time = now.second() - gps.time.second();     // Calculate drifting between RTC and GPS seconds
  }
  if(drifting_time > 10 || drifting_time < -10){          // If more than 10 seconds drifting                               
    if(debug_mode2) Serial.println("RTC time not set\n"); // RTC is not correctly set
    return 0;
  }else{
    if(debug_mode2) Serial.println("RTC time set\n");     // RTC is correctly set
    return 1;
  }
}

int get_unix_time(){
  DateTime now = rtc.now();
  return now.unixtime();
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
  pinMode(2, OUTPUT);                          // Set modem sleeping port
  digitalWrite(2, HIGH);                       // Waking up
  //ssIridium.begin(19200);                    // Start the serial port connected to the satellite modem (Software version)
  ssIridium.begin(19200, SERIAL_8N1, 10, 5);   // Start the serial port connected to the satellite modem (Hardware version)
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);

  // Begin satellite modem operation
  Serial.println("Starting modem...");
  err = modem.begin();                         // Wake up the modem and prepare it to communicate.
  if (err != ISBD_SUCCESS)
  {
    Serial.print("Begin failed: error ");
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println("No modem detected: check wiring.");
    return;
  }
  else Serial.println("Modem well initialised");
  digitalWrite(2, LOW);                       // Modem sleeping
}

void print_iridium_infos(){
  digitalWrite(2, HIGH);                      // Waking up modem
  delay(500);
  if(debug_mode2) Serial.println("Scanning Iridium modem version ... ");
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

void sendreceive_text_iridium(){
  digitalWrite(2, HIGH);                                                              // Waking up modem
  delay(500);
  if (debug_mode) Serial.print("Trying to send the text message.  This might take several minutes.\r\n");
  //int err = modem.sendSBDText(GPSBuffer);                                           // Iridium sending command only
  int err = modem.sendReceiveSBDText(GPSBuffer, receive_buffer, receive_buffer_size); // Iridium sending and receiving command 
  if (err != ISBD_SUCCESS)
  {
    if(debug_mode){
      Serial.print("Transmission failed with error code ");
      Serial.println(err);
      if (err == ISBD_SENDRECEIVE_TIMEOUT) Serial.println("TimeOut : Try again with a better view of the sky\n");
    }
  }
  else{ 
    if(debug_mode) Serial.println("Text buffer sending Ok\n"); 
    if(debug_mode){                                                                   // Receive buffer serial print
      Serial.print("Inbound buffer size is ");
      Serial.println(receive_buffer_size);
      Serial.print("Receive buffer : ");
      for(int i=0; i<receive_buffer_size; ++i) Serial.write(receive_buffer[i]);       // Serial monitor print for text dataframe
      Serial.println();
      Serial.print("Messages remaining to be retrieved: ");
      Serial.println(modem.getWaitingMessageCount());
    }

    switch_state = receive_buffer[0];                                                 // To switch state if MT message tell us
    sending_ok = true;
  }

  digitalWrite(2, LOW);                                                               // Modem sleeping

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

void sendreceive_binary_iridium(){
  digitalWrite(2, HIGH);                                                   // Waking up modem
  delay(500);
  if (debug_mode) Serial.print("Trying to send the binary message.  This might take several minutes.\r\n");
  //int err = modem.sendSBDBinary(buffer_read_340, sizeof(buffer_read_340));                                           // Iridium sending command only
  int err = modem.sendReceiveSBDBinary(buffer_read_340, sizeof(buffer_read_340), receive_buffer, receive_buffer_size); // Iridium sending and receiving command 

  //if(debug_mode) Serial.print("Receive messages detected : ");
  //if(debug_mode) Serial.println(modem.getWaitingMessageCount());

  if (err != ISBD_SUCCESS)
  {
    if(debug_mode){
      Serial.print("Transmission failed with error code ");
      Serial.println(err);
      if (err == ISBD_SENDRECEIVE_TIMEOUT) Serial.println("TimeOut : Try again with a better view of the sky\n");
    }
  }
  else{ 
    if(debug_mode) Serial.println("Binary buffer sending Ok\n"); 
    if(debug_mode){                                                                     // Receive buffer serial print
      Serial.print("Inbound buffer size is ");
      Serial.println(receive_buffer_size);
      Serial.print("Receive buffer : ");
      for(int i=0; i<receive_buffer_size; ++i) Serial.write(receive_buffer[i]);                  // Serial monitor print for text dataframe
      //for(int i=0; i<=receive_buffer_size; i++) Serial.printf("%02x", receive_buffer[i], HEX); // Serial monitor print for binary dataframe
      Serial.println();
      Serial.print("Messages remaining to be retrieved: ");
      Serial.println(modem.getWaitingMessageCount());
    }

    switch_state = receive_buffer[0];                                                   // To switch state if MT message tell us
    sending_ok = true;
  }

  digitalWrite(2, LOW);  // Modem sleeping

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

void receive_iridium(){
  if (!messageSent || modem.getWaitingMessageCount() > 0)
  {
    if(debug_mode) Serial.println("sendReceiveSBD Message detected ... ");
    size_t receive_buffer_size = sizeof(receive_buffer);
    
    // First time through send+receive; subsequent loops receive only
    if (!messageSent)
      err = modem.sendReceiveSBDBinary(receive_buffer, 11, receive_buffer, receive_buffer_size);
      //err = modem.sendReceiveSBDText(NULL, receive_buffer, receive_buffer_size);
    else
      err = modem.sendReceiveSBDText(NULL, receive_buffer, receive_buffer_size);

    if(err != ISBD_SUCCESS)
    {
      if(debug_mode) Serial.print("sendReceiveSBD* failed: error ");
      if(debug_mode) Serial.println(err);
    }
    else // success!
    {
      messageSent = true;
      if(debug_mode){
        Serial.print("Inbound buffer size is ");
        Serial.println(receive_buffer_size);
        for(int i=0; i<receive_buffer_size; ++i)
        {
          Serial.write(receive_buffer[i]);
        }
        Serial.println();
        Serial.print("Messages remaining to be retrieved: ");
        Serial.println(modem.getWaitingMessageCount());
      }
    }
  }else{
    if(debug_mode) Serial.println("sendReceiveSBD No message detected");
  }
}