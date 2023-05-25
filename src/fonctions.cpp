#include "fonctions.h" // Header
using namespace std;

/* ------------------------------------------------- DECLARATIONS ------------------------------------------------------------------*/

/*---------- General declarations ----------*/ 
#define uS_TO_S_FACTOR 1000000ULL        // Conversion factor for micro seconds to seconds 
#define FRENCH_TIME 7200
int led_mode = 0;                        // Use the LED to indicate what's going on
bool debug_mode = false;                 // Sends important informations to the serial monitor
int debug_mode2 = 0;                     // Sends less important informations to the serial monitor
int nbrMes = 3;                          // Number of measurements to perform (then redefined by the config file)
int bootCount = 0;                       // Useful to have a 1st cycle of writing in file different from the following cycles
int TIME_TO_SLEEP = 40;                  // Duration between each cycle (deep sleep and wakeup)
extern uint8_t switch_state;             // To switch state if MT message tell us

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

char GPSBuffer[50];
int reading_pos = 0;

int tab_length = FRAME_DURATION/20; // Acquisition every 20 seconds

float tab_lat[FRAME_DURATION/20];  
float tab_lng[FRAME_DURATION/20];
float tab_temp[FRAME_DURATION/20];
float tab_cond[FRAME_DURATION/20];

float lat_sum=0, lng_sum=0, temp_sum=0, cond_sum=0; 
float lat_moy=0, lng_moy=0, temp_moy=0, temp_rms=0, cond_moy=0, cond_rms=0;

typedef struct dataframe   // Structure creation for data storage
{
  float lat_moy;
  float lng_moy;
  float temp_moy;
  float temp_rms;
  float cond_moy;
  float cond_rms;
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
String id_logger, number_measures, delay_batch, led_mode_sd, debug_mode_sd, clef_test; // config.txt file variables
File confFile;                                                          // To read the config.txt file
File incrementfile;                                                     // Storage file with number of last file created (log and data) to increment
int d_int = 0;                                                          // Value to change and increment file names 
String fichier_config = "/config.txt";                                  // Name of the configuration file
String dataFilename = "/datalog.txt";                                   // Text data file name
String binFilename = "/binfile.bin";                                    // Binary data file name
String logFilename = "/log.txt";                                        // Log info file name

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
uint8_t drifting_time;                                                  // Drifting between RTC and GPS seconds to know if RTC must be adjust or not

/*---------- INA219 current sensor Adafruit ----------*/
Adafruit_INA219 ina219;

/*---------- IridiumSBD Rockblock 9603 ----------*/
#define RXD_IRID 10                // UART ports declaration for communication with Rockblock
#define TXD_IRID 5
//SoftwareSerial ssIridium(5, 10); // RockBLOCK serial port on 10 5 (Software version)
HardwareSerial ssIridium(1);       // RockBLOCK serial port on 10 5 (Hardware version)
IridiumSBD modem(ssIridium, 2);    // RockBLOCK Object creation with SLEEP pin on 2
int signalQuality = -1;
int err;
char version[12];                  // Store the Iridium modem version
bool sending_ok = false;           // For Iridium => continue to send same buffer until it's sent (to avoid timeout and data lost) 
const int commut_irid = 2;         // Rockblock sleeping pin

uint8_t receive_buffer[200];    
size_t receive_buffer_size = 200;

/* ------------------------------------------------- FONCTIONS ------------------------------------------------------------------*/

/* ---------- Auxiliary functions ----------*/
float get_voltage(){
  float vbatt = (analogRead(VBATT_PIN) * 3.30) / 4095.00;
  return vbatt;
  LOG_INFO(get_unix_time(),"Reading battery voltage :",vbatt,"\r");
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
  LOG_INFO(get_unix_time(),"Found :",count,"I2C devices\r");
}

void led_blinkled(int nbr, int freq) {   // freq en sec = freq interval between to light on
  for (int i=0; i<nbr; i++) {
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(freq/2);
    digitalWrite(LED_BUILTIN, LOW);
    delay(freq/2);
  }
}

void all_sleep(int sleeping_time){
  LOG_INFO(get_unix_time(),"All sleep for", sleeping_time,"seconds \r");
  //digitalWrite(commut_irid, LOW);        // Iridum modem sleeping
  digitalWrite(commut_EC, LOW);          // EC sensor sleeping
  digitalWrite(commut_gps, LOW);         // GPS sleeping
  digitalWrite(LED_BUILTIN, LOW);        // Turn off the built-in led
  gpio_hold_en(GPIO_NUM_2);              // Keep off the built-in led during deep sleep
  //neogps.println("$PMTK161,0*28");     // Command line to put GPS in sleeping mode
  esp_sleep_enable_timer_wakeup(sleeping_time * uS_TO_S_FACTOR); // Set deep sleep duration
  Serial.flush();
  esp_deep_sleep_start();                // Switch to deep sleep mode

  //delay(1000*sleeping_time);           // Sleep during sleeping_time in seconds (decomment the line if no uC deep sleep)         
}

void all_wakeup(){
  LOG_INFO(get_unix_time(),"All wake up\r");
  digitalWrite(commut_EC, HIGH);         // EC sensor waking up
  digitalWrite(commut_gps, HIGH);        // GPS waking up
  digitalWrite(commut_irid, HIGH);       // Iridium modem waking up
  //neogps.println("a");                 // Sending any character to wake up gps
  delay(500);   
}

float rmsValue(float arr[], int n)
{
    float square = 0;
    float mean = 0.0, root = 0.0;
    // Calculate square.
    for (int i = 0; i < n; i++) {
        square += pow(arr[i], 2);
    }
    // Calculate Mean.
    mean = (square / (float)(n));
    // Calculate Root.
    root = sqrt(mean);
    return root;
}

/* ---------- Functions related to Atlas EC EZO sensor ----------*/
void initEC(){
  pinMode(commut_EC, OUTPUT);   
  digitalWrite(commut_EC, HIGH); // Switch on power supply of the EC ezo sensor
  LOG_INFO(get_unix_time(),"Init EC sensor\r");
}

void mesureEC(){
  EC.send_read_cmd(); // Sends a read request to the sensor
  delay(600);
  EC.receive_read_cmd(); 
  conductivity = EC.get_last_received_reading(); // Returns the last sensor reading in float
  conduct_int = (uint16_t) conductivity;         // Conversion in uint16_16 to save space for Iridium sending
  LOG_INFO(get_unix_time(),"Reading conductivity :", conductivity);
}

void setting_ec_probe_type() { 
  EC.send_cmd_with_num("K,", PROBE_TYPE);  // Sends any command with the number appended as a string afterwards
  delay(ecDelay);
  if(debug_mode2) Serial.println("Probe type : ");
  receive_and_print_response(EC);          // Used to handle receiving responses and printing them in a common format
  if(debug_mode2) Serial.println();
}

void send_ec_cmd_and_response(char cmd[]) { 
  EC.send_cmd((const char*)cmd);
  delay(ecDelay);
  LOG_INFO(get_unix_time(),"Response\r");
  receive_and_print_response(EC); 
}

/* ---------- Functions related to BlueRobotics temperature sensor ----------*/
void mesure_temp(){
  sensor_fastTemp.read();
  fast_temp = sensor_fastTemp.temperature();
  delay(200);
  LOG_INFO(get_unix_time(),"Reading temperature :",fast_temp," °C\r");
}

/* ---------- Functions related to Grove GPS v1.2 + atomic clock related functions ----------*/
void init_gps(){
  //neogps.begin(9600, SERIAL_8N1, RXD, TXD); // begin GPS hardware serial
  neogps.begin(9600);                         // begin GPS software serial
  pinMode(commut_gps, OUTPUT); 
  digitalWrite(commut_gps, HIGH);
  LOG_INFO(get_unix_time(),"Init GPS\r");
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
    LOG_ERROR(get_unix_time(),"No GPS time data detected\r");
  }  
}

void scanning_gps_coord(){
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available()) // If gps data available
    {
      if (gps.encode(neogps.read())) newData = true;
    }
  }
  if(newData == true)   
  {
    newData = false;
    print_coord_gps();
  }
  else
  {
    LOG_ERROR(get_unix_time(),"No GPS coord data detected\r");
  }  
  
}

void print_coord_gps()
{     
  if (gps.location.isValid() == 1)
  {
    lattitude = gps.location.lat(); 
    longitude = gps.location.lng();
    LOG_INFO(get_unix_time(),"Reading GPS :", String(lattitude,6), " ; ", String(longitude,6), "\r");
  }
  else
  {
    LOG_ERROR(get_unix_time(),"GPS location is not valid\r");
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
    LOG_INFO(get_unix_time(),"Reading GPS date :",datenum_gps);
  }
  else
  {
    LOG_ERROR(get_unix_time(),"GPS date is not valid\r");
  }  

  if (gps.time.isValid() == 1)
  {
    hour_gps = gps.time.hour();
    minute_gps = gps.time.minute();
    second_gps = gps.time.second();
    // Concatenation in datenum to facilitate processing on datachain (datetime = "hour:minute:second")
    datetime_gps = "";   
    datetime_gps += hour_gps; datetime_gps += ":";
    datetime_gps += minute_gps; datetime_gps += ":";
    datetime_gps += second_gps; 
    LOG_INFO(get_unix_time(),"Reading GPS time : ",datetime_gps,"\r");
  }
  else
  {
    LOG_ERROR(get_unix_time(),"GPS time is not valid\r");
  }  
}

bool gps_available(){
  /*boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available()) // If gps data available
    {
      if (gps.encode(neogps.read())) newData = true;
    }
  }
  if(newData == true)   
  {
    newData = false;
    if (gps.location.isValid())
    {
      LOG_INFO(get_unix_time(),"GPS location is valid\r");
      return true;
    }
    else
    {
      LOG_ERROR(get_unix_time(),"GPS location is not valid\r");
      return false;
    }  
  }
  else
  {
    LOG_ERROR(get_unix_time(),"No Data detected, can't communicate with GPS...\r");
    return false;
  }  */
  return true;
}

/* ---------- Functions related to the SD card and the config file ----------*/
void init_sd(){
  LOG_INFO(get_unix_time(),"Initializing SD card... : ");  
  if (SD.begin(cspin_SD)) {                                            // Checks that the SD card is present and can be initialized (pin 5 by default)            
    LOG_ATTACH_FS_AUTO(SD, logFilename, FILE_APPEND);                  // Set file system to save every log automatically 
    LOG_SET_OPTION(0, 1, 0);                                           // LOG_SET_OPTION(file, line, func) Set if it will be printed (1) or not (0)
    LOG_INFO(get_unix_time(),"Card initialized\r"); 
  }else{
    LOG_ERROR(get_unix_time(),"Card failed or not present\r");
  }
  delay(300);  

  if(!SD.exists("/increment_file_name.txt")){                          // If increment file name doesn't exist
    incrementfile = SD.open("/increment_file_name.txt", FILE_WRITE);   // Create it
    LOG_INFO(get_unix_time(), "increment_file_name.txt created\r");
    incrementfile.close();
  }
  incrementfile = SD.open("/increment_file_name.txt", FILE_READ);      // Open increment file
  if(incrementfile){
    LOG_INFO(get_unix_time(), "increment_file_name.txt opened\r");
    while(incrementfile.available()){
      d_int = incrementfile.read();     // Update increment value according to last value stored in the file
    }
    incrementfile.close();
    LOG_INFO(get_unix_time(), "increment file value d_int = ", d_int, "\r");
  }else{
    LOG_INFO(get_unix_time(), "Opening increment_file_name.txt failed\r");
  }
  dataFilename = "/datalog_" + String(d_int+1,DEC) + ".txt";           // Updating file names with incrementation
  binFilename = "/binfile_" + String(d_int+1,DEC) + ".bin";
  logFilename = "/log_" + String(d_int+1,DEC) + ".txt"; 

  LOG_ATTACH_FS_AUTO(SD, logFilename, FILE_APPEND);                    // Set file system to save every log automatically 

  File dataFile = SD.open("/increment_file_name.txt", FILE_APPEND);             
  if(dataFile){                                                        // If increment file available
    d_int = d_int + 1;
    dataFile.write(d_int);                                             // Update and write the new d_int value for next reading
    dataFile.close();
    LOG_INFO(get_unix_time(), "Updating d_int value in increment_file_name.txt", d_int, "\r");
  }
  else LOG_ERROR(get_unix_time(),"Error opening increment file\r");    // If file not opened, display error
  
  if(!SD.exists(binFilename)){                                         // If bin file name doesn't exist     
    File binFile = SD.open(binFilename, FILE_WRITE);                   // Create it
    LOG_INFO(get_unix_time(), binFilename, " created\r");
    binFile.close();
    binFile = SD.open(binFilename, FILE_READ);                         // Allow to resume the reading of the binary file at the last frame written at each reboot of the card
    reading_pos = binFile.size();
    binFile.close();
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
    LOG_INFO(get_unix_time(),"Opening",fichier_config,"\r");
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
    LOG_ERROR(get_unix_time(),"Error opening",fichier_config,"\r");
  }
}

void refresh_config_values(){
  TIME_TO_SLEEP=delay_batch.toInt();  // Association of program variables with those read from the SD card
  nbrMes=number_measures.toInt();            
  debug_mode=debug_mode_sd.toInt();  
  led_mode=led_mode_sd.toInt(); 
}

void calcul_and_filling_dataframe_write(){
  neogps.println("a");                 // Sending any character to wake up gps
  digitalWrite(commut_EC, HIGH);         // EC sensor waking up
  delay(500);
  LOG_INFO("");
  LOG_INFO(get_unix_time(),"Calcul and filling dataframe_write\r");
  reading_rtc();                             // Date and time acquisition via RTC
  for(int i = 0; i < tab_length; i++){       // Tabs sum calculation 
    lat_sum = lat_sum + tab_lat[i];
    lng_sum = lng_sum + tab_lng[i];
    temp_sum = temp_sum + tab_temp[i];
    cond_sum = cond_sum + tab_cond[i];
  }
  lat_moy = lat_sum / tab_length;            // Average calculation to store in dataframe_write 
  lng_moy = lng_sum / tab_length;
  temp_moy = temp_sum / tab_length;
  cond_moy = cond_sum / tab_length;

  lat_sum = 0; lng_sum = 0; temp_sum = 0; cond_sum = 0;

  temp_rms = rmsValue(tab_temp, tab_length); // RMS calculation to store in dataframe_write 
  cond_rms = rmsValue(tab_cond, tab_length);

  LOG_INFO(get_unix_time(), "lat moy : ", lat_moy, " long moy : ", lng_moy, " temp moy : ", temp_moy, " temp rms : ", temp_rms, "cond moy : ", cond_moy, "cond rms : ", cond_rms, "\r");

  for(int i = 0; i < tab_length; i++){       // Empty tabs for next frame's acquisitions
    tab_lat[i] = 0;
    tab_lng[i] = 0;
    tab_temp[i] = 0;
    tab_cond[i] = 0;
  }

  dataframe_write.d = uint_dayRTC;           // Filling dataframe_write structure for .bin file (for Iridium sending)
  dataframe_write.mth = uint_monthRTC;
  dataframe_write.y = uint_yearRTC;
  dataframe_write.h = uint_hourRTC;
  dataframe_write.min = uint_minRTC;
  dataframe_write.s = uint_secRTC;
  dataframe_write.lat_moy = lat_moy;  
  dataframe_write.lng_moy = lng_moy;
  dataframe_write.cond_moy = cond_moy;
  dataframe_write.cond_rms = cond_rms;
  dataframe_write.temp_moy = temp_moy;
  dataframe_write.temp_rms = temp_rms;

  // Filling outBuffer for .txt file storage in each acquisition
  sprintf(outBuffer, "FRAME : %02d/%02d/%d %02d:%02d:%02d lat_moy : %0.6lf long_moy : %0.6lf cond_moy : %0.2f cond_rms : %0.2f temp_moy : %0.2f temp_rms : %0.2f",     
    uint_dayRTC, 
    uint_monthRTC,
    uint_yearRTC, 
    uint_hourRTC, 
    uint_minRTC, 
    uint_secRTC,
    lat_moy,
    lng_moy,
    cond_moy,
    cond_rms,
    temp_moy,
    temp_rms);
}

void store_data_in_arrays(int acquisition_number){                        // Store each acqusition of the frame in float arrays
  //neogps.println("a");                 // Sending any character to wake up gps
  //digitalWrite(commut_EC, HIGH);       // EC sensor waking up
  //delay(500);

  LOG_DEBUG(get_unix_time(),"Store each acqusition of the frame in float arrays\r");
  reading_rtc();                                                          // Date and time acquisition via RTC
  mesureEC();                                                             // Atlas ec ezo sensor measure
  mesure_temp();                                                          // Temperature sensor measure
  scanning_gps_coord();                                                   // Gps coordinates acquisition

  save_acquisition_to_sd();                                               // Save each acquisition of the frame in .txt file on SD card

  tab_lat[acquisition_number] = gps.location.lat();                      // Fill frame's tabs
  tab_lng[acquisition_number] = gps.location.lng();
  tab_temp[acquisition_number] = fast_temp;
  tab_cond[acquisition_number] = conductivity;

  //digitalWrite(commut_EC, LOW);        // EC sensor sleeping
  //neogps.println("$PMTK161,0*28");     // Command line to put GPS in sleeping mode
}

void save_dataframe_to_sd(){
  LOG_DEBUG(get_unix_time(),"Saving dataframe to SD\r");
  //--- Text format saving (.txt) ---//
  File dataFile = SD.open(dataFilename, FILE_APPEND);                           // FILE_APPEND for esp32, FILE_WRITE for arduino
  if(dataFile){                                                                // If file available, writes the content of the datachain to the file                                         
    dataFile.print("outBuffer : ");                                             // Classic data buffer saving (more visual in the file .txt)
    dataFile.println(outBuffer);  
    dataFile.println(" ");
    dataFile.close();
    LOG_DEBUG(get_unix_time(),"Textfile ", dataFilename, " created successfully\r");
  }
  else {                                                                        // If file not opened, display error
    LOG_ERROR(get_unix_time(),"Error opening txt file\r");
  }

  //--- Binary format saving (.bin) ---//
  File binFile = SD.open(binFilename, FILE_APPEND);                             // Writing mode opening file 
   
  if (binFile) {                                                                // If file available
    binFile.write((const uint8_t *)&dataframe_write, sizeof(dataframe_write));  // Write the content of the dataframe_write struct in the file
    delay(50);
    binFile.close();
    LOG_DEBUG(get_unix_time(),"Binaryfile ", dataFilename, " created successfully\r");
  }
  else {                                                                        // If file not opened, display error
    LOG_ERROR(get_unix_time(),"Error opening binary file\r");
  }
  delay(400);
}

void save_acquisition_to_sd(){
  LOG_DEBUG(get_unix_time(),"Saving acquisition to SD\r");
  //--- Text format saving (.txt) ---//
  File dataFile = SD.open(dataFilename, FILE_APPEND);                           // FILE_APPEND for esp32, FILE_WRITE for arduino
  if (dataFile) {                                                               // If file available, writes the content of the datachain to the file
    dataFile.print(datetime_rtc); dataFile.print(" ; ");
    dataFile.print(conductivity); dataFile.print(" ; ");
    dataFile.print(fast_temp); dataFile.print(" ; ");
    dataFile.print(String(lattitude,6)); dataFile.print(" ; "); dataFile.println(String(longitude,6));
    dataFile.close();
    LOG_DEBUG(get_unix_time(),"Acquisition correctly saved in file", dataFilename, "\r");
  }
  else {                                                                        // If file not opened, display error
    LOG_ERROR(get_unix_time(),"Error opening txt file\r");
  }
}

void readSDbinary_to_struct(){
  LOG_INFO(get_unix_time(),"Reading SD binary file to struct\r");
  for(int i = 0; i <= FRAME_NUMBER; i++){                                // Concatenation of n buffer_read in buffer_read_340 
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
      LOG_ERROR(get_unix_time(),"Error opening reading binary file\r");
    }
    for(int j=0; j<=sizeof(buffer_read); j++) buffer_read_340[i*sizeof(buffer_read)+j] = buffer_read[j];   // Push buffer_read in buffer_read_340 
  }                                                                     
  if(debug_mode2){                                                        // Testing if structure and buffers are correctly completed
    Serial.print("dataframe_read : ");
    for(int i = 0; i < sizeof(dataframe_read); i++) Serial.printf("%02x", (unsigned int) ((char*)&dataframe_read)[i]);
    Serial.print("buffer_read : ");
    for(int i=0; i<=sizeof(buffer_read); i++) Serial.printf("%02x", buffer_read[i], HEX);  
    Serial.print("buffer_read_340 : ");
    for(int i=0; i<=sizeof(buffer_read_340); i++) Serial.printf("%02x", buffer_read_340[i], HEX);  
  }   
  LOG_INFO(get_unix_time(), "dataframe_read sample : lat moy : ", dataframe_read.lat_moy, " long moy : ", dataframe_read.lng_moy, " temp moy : ", dataframe_read.temp_moy, "cond moy : ", dataframe_read.cond_moy, "\r");
  LOG_INFO(get_unix_time(), "                       ", dataframe_read.y, "/", dataframe_read.d, "/", dataframe_read.mth, "  ", dataframe_read.h, ":", dataframe_read.min, ":", dataframe_read.s, "\r");
  LOG_INFO(get_unix_time(), "                        dataframe_read size : ", sizeof(dataframe_read), "\r");
  //LOG_INFO(get_unix_time(), "                        dataframe_read content : ", (char*)&dataframe_read, "\r");
  LOG_INFO(get_unix_time(), "                        buffer_read size : ", sizeof(buffer_read), "\r");
  //LOG_INFO(get_unix_time(), "                        buffer_read content : ", buffer_read, "\r");
  LOG_INFO(get_unix_time(), "                        buffer_read_340 size : ", sizeof(buffer_read_340), "\r");
  //LOG_INFO(get_unix_time(), "                        buffer_read_340 content : ", buffer_read_340, "\r");
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
  LOG_INFO(get_unix_time(),"Init RTC\r");
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

  LOG_INFO(get_unix_time(),"Reading RTC : ", datetime_rtc, "\r");
}

void set_rtc(int day, int month, int year, int hour, int min, int sec){
  LOG_INFO(get_unix_time(),"RTC manually adjusted\r");
  rtc.adjust(DateTime(year, 
                      month, 
                      day, 
                      hour, 
                      min, 
                      sec));
}

void set_rtc_by_gps(){
  LOG_INFO(get_unix_time(),"Setting RTC from GPS\r");
  print_date_gps();   // Reading GPS date
  if (gps.date.isValid() && gps.time.isValid())
  {
    rtc.adjust(DateTime(gps.date.year()-2000, 
                        gps.date.month(), 
                        gps.date.day(), 
                        gps.time.hour(), 
                        gps.time.minute(), 
                        gps.time.second()));
    reading_rtc();    // Test (to delete)
    LOG_INFO(get_unix_time(),"RTC set from GPS at", datetime_rtc, "\r");
  }else{
    LOG_ERROR(get_unix_time(),"No GPS fixed yet. Cant set RTC yet\r");
  }  
}

int check_rtc_set()
{
  LOG_INFO(get_unix_time(),"Check RTC set\r");
  DateTime now = rtc.now();                               // Reading RTC time
  print_date_gps();                                       // Reading GPS time
  if (gps.date.isValid() && gps.time.isValid())
  {
    drifting_time = now.second() - gps.time.second();     // Calculate drifting between RTC and GPS seconds
  }
  if(drifting_time > 10 || drifting_time < -10){          // If more than 10 seconds drifting                               
    if(debug_mode2) Serial.println("RTC time not set\n"); // RTC is not correctly set
    LOG_ERROR(get_unix_time(),"RTC time not set\r");
    return 0;
  }else{
    if(debug_mode2) Serial.println("RTC time set\n");     // RTC is correctly set
    LOG_INFO(get_unix_time(),"RTC time set\r");
    return 1;
  }
}

int get_unix_time(){
  //DateTime now = rtc.now();
  //return now.unixtime();
  return millis()/1000;
}

/* ---------- Functions related to INA219 current sensor ---------- */
void init_ina219(){
  if (! ina219.begin()) {
    LOG_INFO(get_unix_time(),"Failed to find INA219 chip\r");
  }
  else{ 
    LOG_INFO(get_unix_time(),"Correctly found INA219 chip\r");
  }
}

void get_current(){
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float sum_current = 0;
  float loadvoltage = 0;
  float power_mW = 0;
  float capacity_Ah = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  sum_current = sum_current + current_mA;
  capacity_Ah = (sum_current / 1000) * 3600;
  
  if(debug_mode){
    Serial.println("------- Measuring voltage and current with INA219 -------");
    Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
    Serial.println("");
  }

  LOG_INFO(get_unix_time(),"Correctly found INA219 chip\r");

  delay(2000);
}

/* ---------- Functions related to Iridium Rockblock 9603 ----------*/
void init_iridium(){
  //ssIridium.begin(19200);                    // Start the serial port connected to the satellite modem (Software version)
  ssIridium.begin(19200, SERIAL_8N1, 10, 5);   // Start the serial port connected to the satellite modem (Hardware version)
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);

  // Begin satellite modem operation
  LOG_INFO(get_unix_time(),"Starting Iridium modem\r");
  err = modem.begin();                         // Wake up the modem and prepare it to communicate.
  if (err != ISBD_SUCCESS)
  {
    LOG_ERROR(get_unix_time(),"Iridium starting failed with error ", err, "\r");
    if (err == ISBD_NO_MODEM_DETECTED)
      LOG_ERROR(get_unix_time(),"No modem detected, check wiring\r");
    return;
  }
  else {
    LOG_INFO(get_unix_time(),"Modem well initialized\r");
  }
  LOG_INFO(get_unix_time(),"Sleeping modem\r");

  modem.sleep();                       // Modem sleeping
}

void print_iridium_infos(){
  if(modem.isAsleep()) modem.begin();                       // Waking up modem
  delay(500);
  LOG_INFO(get_unix_time(),"Scanning Iridium modem version\r");
  err = modem.getFirmwareVersion(version, sizeof(version)); // Print the firmware revision
  if (err != ISBD_SUCCESS)
  {
     Serial.print("FirmwareVersion failed: error ");
     Serial.println(err);
     LOG_ERROR(get_unix_time(),"Firmware version failed with error ", err, "\r");
     return;
  }
  LOG_INFO(get_unix_time(),"Firmware version is ", version, "\r");

  // Test signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  err = modem.getSignalQuality(signalQuality);
  if (err != ISBD_SUCCESS)
  {
    LOG_ERROR(get_unix_time(),"Signal quality failed with error ", err, "\r");
    return;
  }
  LOG_INFO(get_unix_time(),"On a scale of 0 to 5, signal quality is currently ", signalQuality, "\r");
}

void sendreceive_recovery_iridium(){
  if(modem.isAsleep()) modem.begin();                                                   // Waking up modem
  delay(500);
  LOG_INFO(get_unix_time(),"Trying to send Iridium message, this might take several minutes\r");
  receive_buffer_size = sizeof(receive_buffer);
  for(int i = 0; i < receive_buffer_size; i++) receive_buffer[i] = 0;                                            // Clearing receive buffer
  //int err = modem.sendSBDText(GPSBuffer);                                                                      // Iridium sending command only
  int err = modem.sendReceiveSBDText(GPSBuffer, receive_buffer, receive_buffer_size);                            // Iridium sending and receiving command 
  //int err = modem.sendReceiveSBDBinary(buffer_read, sizeof(buffer_read), receive_buffer, receive_buffer_size); // Test with binary instead of text (to delete)
  if (err != ISBD_SUCCESS)
  {
    PRINTLN_FILE("");
    LOG_ERROR(get_unix_time(),"Transmission failed with error ", err, "\r");
    if (err == ISBD_SENDRECEIVE_TIMEOUT) LOG_ERROR("TIME OUT\r");
  }
  else{ 
    PRINTLN_FILE("");
    LOG_INFO(get_unix_time(),"Text buffer sending Ok\r");
    if(debug_mode2){                                                              // Receive buffer serial print
      Serial.print("Receive buffer : ");
      for(int i=0; i<receive_buffer_size; ++i) Serial.write(receive_buffer[i]);  // Serial monitor print for text dataframe
      Serial.println();
    }
    LOG_INFO(get_unix_time(),"DATA RECEIVING ! buffer size is ", receive_buffer_size, "; Receive_buffer[0] : ", receive_buffer[0], "\r");
    switch_state = receive_buffer[0];                                            // To switch state if MT message tell us
    sending_ok = true;
  }
  modem.sleep();                                                          // Modem sleeping

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

void sendreceive_deployed_iridium(){
  if(modem.isAsleep()) modem.begin();                                // Waking up modem
  delay(500);
  LOG_INFO(get_unix_time(),"Trying to send Iridium message, this might take several minutes\r");
  receive_buffer_size = sizeof(receive_buffer);
  for(int i = 0; i < receive_buffer_size; i++) receive_buffer[i] = 0;                                                    // Clearing buffer
  //int err = modem.sendSBDBinary(buffer_read_340, sizeof(buffer_read_340));                                             // Iridium sending command only
  int err = modem.sendReceiveSBDBinary(buffer_read_340, sizeof(buffer_read_340), receive_buffer, receive_buffer_size); // Iridium sending and receiving command 
  //int err = modem.sendReceiveSBDBinary(buffer_read, sizeof(buffer_read), receive_buffer, receive_buffer_size);           // Test to use only 1 credit instead of 7 for buffer_read_340
  if (err != ISBD_SUCCESS)
  {
    PRINTLN_FILE("");
    LOG_ERROR(get_unix_time(),"Transmission failed with error ", err, "\r");
    if (err == ISBD_SENDRECEIVE_TIMEOUT) LOG_ERROR("TIME OUT\r");
  }
  else{ 
    PRINTLN_FILE("");
    LOG_INFO(get_unix_time(),"Binary buffer sending Ok\r");
    if(debug_mode2){                                     // Receive buffer serial print
      Serial.print("Receive buffer : ");
      for(int i=0; i<receive_buffer_size; ++i) Serial.write(receive_buffer[i]);                  // Serial monitor print for text dataframe
      //for(int i=0; i<=receive_buffer_size; i++) Serial.printf("%02x", receive_buffer[i], HEX); // Serial monitor print for binary dataframe
      Serial.println();
    }
    LOG_INFO(get_unix_time(),"DATA RECEIVING ! buffer size is ", receive_buffer_size, "; Receive_buffer : ", receive_buffer[0], "\r");
    switch_state = receive_buffer[0];                   // To switch state if MT message tell us
    sending_ok = true;                                  // For Iridium => continue to send same buffer until it's sent (used in main to avoid timeout and data lost)
  }
  modem.sleep();                                 // Modem sleeping

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

void sendreceive_emergency_iridium(){
  if(modem.isAsleep()) modem.begin();                                                   // Waking up modem
  delay(500);
  LOG_INFO(get_unix_time(),"Trying to send Iridium message, this might take several minutes\r");
  receive_buffer_size = sizeof(receive_buffer);
  for(int i = 0; i < receive_buffer_size; i++) receive_buffer[i] = 0;                                            // Clearing receive buffer
  int err = modem.sendReceiveSBDText("! LOW VOLTAGE !", receive_buffer, receive_buffer_size);                    // Iridium sending and receiving command 
  if (err != ISBD_SUCCESS)
  {
    PRINTLN_FILE("");
    LOG_ERROR(get_unix_time(),"Transmission failed with error ", err, "\r");
    if (err == ISBD_SENDRECEIVE_TIMEOUT) LOG_ERROR("TIME OUT\r");
  }
  else{ 
    PRINTLN_FILE("");
    LOG_INFO(get_unix_time(),"Text buffer sending Ok\r");
    if(debug_mode2){                                                              // Receive buffer serial print
      Serial.print("Receive buffer : ");
      for(int i=0; i<receive_buffer_size; ++i) Serial.write(receive_buffer[i]);  // Serial monitor print for text dataframe
      Serial.println();
    }
    LOG_INFO(get_unix_time(),"DATA RECEIVING ! buffer size is ", receive_buffer_size, "; Receive_buffer[0] : ", receive_buffer[0], "\r");
    switch_state = receive_buffer[0];                                            // To switch state if MT message tell us
    sending_ok = true;
  }
  modem.sleep();                                                          // Modem sleeping

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

void send_text_iridium(char text[50]){
  if(modem.isAsleep()) modem.begin();                         // Waking up modem
  delay(500);
  LOG_INFO(get_unix_time(),"Trying to send Iridium message, this might take several minutes\r");

  int err = modem.sendSBDText(text);                          // Iridium sending command only
  if (err != ISBD_SUCCESS)
  {
    LOG_ERROR(get_unix_time(),"Transmission failed with error ", err, "\r");
    if (err == ISBD_SENDRECEIVE_TIMEOUT) LOG_ERROR("TIME OUT\r");
  }
  else{ 
    LOG_INFO(get_unix_time(),"Text buffer sending Ok\r");
    sending_ok = true;
  }
  modem.sleep();                                              // Modem sleeping

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
  LOG_INFO("Receive_Iridium()\r");
  if(modem.isAsleep()) modem.begin();                         // Waking up modem
  
  receive_buffer_size = sizeof(receive_buffer);
  for(int i = 0; i < receive_buffer_size; i++) receive_buffer[i] = 0;                    // Clearing receive buffer
  
  int err = modem.sendReceiveSBDText(NULL, receive_buffer, receive_buffer_size);         // Iridium receive only command 

  if(err != ISBD_SUCCESS){
    PRINTLN_FILE("");
    LOG_ERROR(get_unix_time(),"Transmission failed with error ", err, "\r");
    if (err == ISBD_SENDRECEIVE_TIMEOUT) LOG_ERROR("TIME OUT\r");
  }
  else{ // success!
    LOG_INFO(get_unix_time(),"DATA RECEIVING ! buffer size is ", receive_buffer_size, "; Receive_buffer : ", receive_buffer[0], "\r");
    switch_state = receive_buffer[0];                   // To switch state if MT message tell us
  }

  modem.sleep();                                                          // Modem sleeping
}