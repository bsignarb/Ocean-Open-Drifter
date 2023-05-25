#include "fonctions.h"

extern TSYS01 sensor_fastTemp;                                     // External declaration of the BlueRobotics temperature sensor
RTC_DATA_ATTR enum :byte {INIT, DEPLOYED, RECOVERY, EMERGENCY} state_mode;  // State machine to switch mode
uint8_t switch_state = 0;                                          // To switch state if MT message tell us           
int waiting_time = 0, waiting_counter1, waiting_counter2;          // How long has GPS been unavailable

int average_sending_time = 120;                                    // Default average sending time of 2 minutes = 120 seconds 
int margin_time = FRAME_NUMBER;                                    // FRAME_NUMBER seconds margin time (1sec min) for delay between each Iridium sending 
int counter1_acqu = 0, counter2_acqu = 0;                          // Counters to synchronize delays between acquisitions
int counter1_send = 0, counter2_send = 0;                          // Counters to calculate delays between each recovery Iridium sending
extern bool sending_ok;                                            // For Iridium => continue to send same buffer until it's sent (to avoid timeout and data lost)
bool first_init_passed = false;                                    // Differentiates first cycle of the Init from the following ones (and to know when first init succeeded for ISBDCallback() function)
int actual_acquisition = 0;                                        // To synchronize acquisitions between classic ones and ISBDcallback() ones
int actual_cycle = 1;                                              // To synchronize frames between classic ones and ISBDcallback() ones
int time_between_each_acquisition = 16000;                         // Time between each acquisition (16 seconds to have a total period of 20 seconds because of the time acquisition about 4 seconds)

int emergency_sleeping_time = 300;                                 // Emergency sleeping time (5 min)
bool first_time_emergency = true;                                  // To not go back to emergency state if MT message ask to go in another state

void setup() {
  Serial.begin(115200);                                            // Start the console serial port 
  state_mode = INIT;                                               // Init state machine to deployed mode
  //WiFi.disconnect();
}

void loop() {     
  // Switch state condition - Iridium MT messages
  if(switch_state == 49) state_mode = INIT;                        // Change to INIT state (because of Iridium MT message) (dec "49" = char "1")
  if(switch_state == 50) state_mode = DEPLOYED;                    // Change to DEPLOYED state (because of Iridium MT message) (dec "50" = char "2")
  if(switch_state == 51) state_mode = RECOVERY;                    // Change to RECOVERY state (because of Iridium MT message) (dec "51" = char "3")
  if(switch_state == 52) state_mode = EMERGENCY;                   // Change to EMERGENCY state (because of Iridium MT message) (dec "52" = char "4")

  // ------------------------------------------------ SWITCH STATE MACHINE ---------------------------------------------------- //
  switch(state_mode){                                           
    case INIT :                                                    // Waiting state : pass in DEPLOYED mode as soon as the GPS has found its position
      LOG_INFO("\r"); 
      LOG_INFO("--- CASE INIT ---\r"); 
      init_cycle();
      //test_cycle_init();

      // Wait until GPS has found its position. If it doesn't found in a certain time, go to sleep to save battery life
      waiting_time = 0;
      waiting_counter1 = get_unix_time();          
      while((gps_available() == false) && (waiting_time < 300) ){  // While GPS not available and waiting_time < 5 minutes => Do nothing, just wait until one of the conditions is true
        waiting_counter2 = get_unix_time();
        waiting_time = waiting_counter2 - waiting_counter1;        // Calculate waiting time
        LOG_INFO("Waiting time : ", waiting_time, "\r"); 
      }   
      if(waiting_time >= 300) state_mode = INIT;                   // If GPS not available since more than 5 minutes => Change to INIT state (2nd part for sleeping)
      else                    state_mode = DEPLOYED;               // GPS is available => Change to DEPLOYED state
      //state_mode = DEPLOYED;
      break;

    case DEPLOYED :                                                // Standard acquisition cycles
      LOG_INFO("\r"); 
      LOG_INFO("--- CASE DEPLOYED ---\r"); 
      deployed_cycle();
      //test_cycle();
      break;

    case RECOVERY :                                                // More frequent cycles
      LOG_INFO("\r"); 
      LOG_INFO("--- CASE RECOVERY ---\r"); 
      recovery_cycle();
      break;

    case EMERGENCY :
      LOG_INFO("\r"); 
      LOG_INFO("--- CASE EMERGENCY ---\r"); 
      emergency_cycle();
      if(get_voltage() > 3.25){                                    // Battery level is good again 
        state_mode = DEPLOYED;                                     // Change to DEPLOYED state                
        LOG_INFO(get_unix_time(), "Battery level good again : ", get_voltage(), "V\r");                 
      }
      delay(1000);
      break;
  }

  // ------------------------------------------ STATE CHANGES CONDITIONS ----------------------------------------------------- //
  // GPS position : Verify if GPS position is available. If it doesn't found in a certain time, go to sleep to save battery life
  /*waiting_time = 0;
  waiting_counter1 = get_unix_time();          
  while((gps_available() == false) && (waiting_time < 300) ){  // While GPS not available and waiting_time < 5 minutes => Do nothing, just wait until one of the conditions is true
    waiting_counter2 = get_unix_time();
    waiting_time = waiting_counter2 - waiting_counter1;        // Calculate waiting time
    LOG_INFO(millis(), "Waiting time : ", waiting_time, "\r"); 
  }   
  if(waiting_time >= 300) state_mode = INIT; */                // If GPS not available since more than 5 minutes => Change to INIT state (2nd part for sleeping)
                      
  
  // Battery voltage
  if(get_voltage() < 3.2 && get_voltage() > 0.5){              // Low battery level
    if(first_time_emergency){                                  // To not go back to emergency state if MT message ask to go in another state
      state_mode = EMERGENCY;                                  // Change to EMERGENCY state
      first_time_emergency = false;
    }
    LOG_ERROR(get_unix_time(), "Low battery level : ", get_voltage(), "V\r");
  }

  LOG_INFO("\r"); 
  LOG_INFO("----------------------------------------------------------------------------------\r"); 
}

void init_cycle(){
  if(first_init_passed == false){    // First Init cycle
    LOG_FILE_SET_LEVEL(DebugLogLevel::LVL_INFO); // Set level for Debuglog
    pinMode(LED_BUILTIN, OUTPUT); 
    //pinMode(16, OUTPUT);           // Declare powerfull led port
    Wire.begin();                    // For I2C communication 
    delay(50);                       // Allows time for the I2C electronic to initialize correctly
    initEC();                        // Init conductivity sensor
    delay(50);
    sensor_fastTemp.init();          // Init temperature sensor
    init_RTC();
    init_gps();                      // Init UART communication of GPS
    init_sd();                       // Init and test SD card
    lecture_config();                // Read config file
    refresh_config_values();         // Refresh program values according to those read in config file
    init_iridium();                  // Init Iridium module

    first_init_passed = true;
  }
  else{                              // Following Init cycles for sleeping mode
    all_sleep(120);                  // Sleeping for 2 minutes (120 seconds) because of no GPS signal
    //all_wakeup();
  }
}

void deployed_cycle(){
  if(!check_rtc_set()){                     // If RTC not correctly set
    set_rtc_by_gps();                       // RTC update by gps time
  }

  //--- Datalogging ---//
  first_time_emergency = true;                                                         // Used in emergency_cycle() to send only one time Iridium sending
  for(int cycle_number = actual_cycle; cycle_number <= FRAME_NUMBER; cycle_number++){  // FRAME_NUMBER times frames before Iridium sending
    actual_cycle = cycle_number;                                                       // To synchronize frames between classic ones (Deployed and Recovery) and ISBDcallback() ones

    for(int i = actual_acquisition; i < ACQUISITION_NUMBER; i++){                      // ACQUISTION_NUMBER times acquisitions before calcul and storage in frame
      PRINTLN_FILE("");
      PRINTLN_FILE("-------------------------------------------- Frame cycle : ", cycle_number, "/", FRAME_NUMBER, " ; Acquisition cycle : ", i+1, "/", ACQUISITION_NUMBER, "\r");
      PRINTLN("");
      PRINTLN("-------------------------------------------- Frame cycle : ", cycle_number, "/", FRAME_NUMBER, " ; Acquisition cycle : ", i+1, "/", ACQUISITION_NUMBER, "\r");  
      actual_acquisition = i;                                                          // To synchronize acquisitions between classic ones (Deployed and Recovery) and ISBDcallback() ones       
      store_data_in_arrays(i);                                                         // Do one acqusition and store data in arrays
     
      delay(time_between_each_acquisition);                                            // Delay between each acquisition
    }
    actual_acquisition = 0;                          // To synchronize acquisitions between classic ones and ISBDcallback() ones
    calcul_and_filling_dataframe_write();            // Calculate average and rms values from tab and store the parameters in dataframe_write struct
    save_dataframe_to_sd();                          // Write the content of dataframe_write to the "dataFilename" file on the SD card
    
    /*receive_iridium();                             // Verify if MT message is waiting to be received
    if((switch_state != 50) && (switch_state != 0)){ // If MT message received tell us to change state
      esp_sleep_enable_timer_wakeup(5*1000000);      // Deep sleep duration of 5 seconds
      Serial.flush();
      esp_deep_sleep_start();                        // Start deep sleep  (The uC will deep sleep a very short time and wake up in the good state according to Iridium MT message)           
    }*/
  }
  actual_cycle = 1;                         // To synchronize frames between classic ones and ISBDcallback() ones
  counter1_acqu = millis()/1000;          // counter1_acqu used to calculate time between each acquisition (in recovery cycle and ISBDCallback)

  readSDbinary_to_struct();                 // Read datalog file in a dataframe_read structure for Iridium sending

  //--- Iridium sending ---//
  LOG_INFO("GOING TO IRIDIUM PHASE");
  print_iridium_infos();                    // Iridium modem informations
  sending_ok = false;
  while(sending_ok == false)                // Continue to send same buffer until it's sent (to avoid timeout and data lost)
  {
    sendreceive_deployed_iridium();         // Sending data with buffer_read_340 (MO message) AND receiving data with receive_buffer (MT message)
  }
  counter1_send = millis()/1000;            // counter1_send used to calculate time between each recovery Iridium sending (in recovery cycle)
  LOG_INFO("END OF IRIDIUM PHASE");
}

void recovery_cycle(){
  //--- Datalog - Even in Recovery cycle we continue acquisitions and save it to SD card ---// 
  for(int i = actual_acquisition; i < ACQUISITION_NUMBER; i++){                      // ACQUISTION_NUMBER times acquisitions before calcul and storage in frame
    PRINTLN_FILE("");
    PRINTLN_FILE("-------------------------------------------- Frame cycle : ", actual_cycle, "/", FRAME_NUMBER, " ; Acquisition cycle : ", i+1, "/", ACQUISITION_NUMBER, "\r");
    PRINTLN("");
    PRINTLN("-------------------------------------------- Frame cycle : ", actual_cycle, "/", FRAME_NUMBER, " ; Acquisition cycle : ", i+1, "/", ACQUISITION_NUMBER, "\r");  
    actual_acquisition = i;                                                          // To synchronize acquisitions between classic ones (Deployed and Recovery) and ISBDcallback() ones       
    store_data_in_arrays(i);                                                         // Do one acqusition and store data in arrays
     
    delay(time_between_each_acquisition);                                            // Delay between each acquisition
  }
  actual_acquisition = 0;                          // To synchronize acquisitions between classic ones and ISBDcallback() ones
  calcul_and_filling_dataframe_write();            // Calculate average and rms values from tab and store the parameters in dataframe_write struct
  save_dataframe_to_sd();                          // Write the content of dataframe_write to the "dataFilename" file on the SD card
  readGPS_to_buffer();
  
  if(actual_cycle < FRAME_NUMBER) actual_cycle++; // Incrementation of actual_cycle for futures frames (FRAME_NUMBER times frames before Iridium sending) 
  else actual_cycle = 1;           
  counter1_acqu = millis()/1000;          // counter1_acqu used to calculate time between each acquisition (in recovery cycle and ISBDCallback)
  
  //--- Iridium sending ---//
  LOG_INFO("GOING TO IRIDIUM PHASE");
  print_iridium_infos();                    // Iridium modem informations
  sending_ok = false;
  while(sending_ok == false)                // Continue to send same buffer until it's sent (to avoid timeout and data lost)
  {
    sendreceive_recovery_iridium();                // Send directly time and position with GPSBuffer (MO message) AND receiving data with receive_buffer (MT message)
  }
  LOG_INFO("END OF IRIDIUM PHASE");
}

void emergency_cycle(){
  sendreceive_emergency_iridium();                            // Sending Iridium message to inform low battery level and Scanning if MT message is available 
  if(switch_state == 52) all_sleep(emergency_sleeping_time);  // If MT message didn't ask to switch state, go sleeping for 5 minutes
}

void test_cycle_init(){
  Wire.begin();                  // For I2C communication 
  sensor_fastTemp.init();        // Init temperature sensor
  //init_gps();                  // Init communication UART of GPS
  //initEC();
}

void test_cycle(){
  //mesure_cycle_to_datachain(); // Start a measurement cycle and store the measured parameters in dataframe_write struct
  //save_dataframe_to_sd();      // Write the content of dataframe_write to the "dataFilename" file on the SD card
  print_iridium_infos();         // Iridium modem informations
  //receive_iridium();
  delay(10000);
}

bool ISBDCallback()
{
  //--- Datalog - Even when sending with Iridium we continue acquisitions and save it to SD card ---// 
  if(first_init_passed == true){                                          // When first INIT cycle succeeded 
    counter2_acqu = millis()/1000;                                           // counter2_acqu used to calculate time between each acquisition
    if(1000*(counter2_acqu - counter1_acqu) > time_between_each_acquisition){       // If it's time to do a new acquisition (if its not time yet, do nothing)
      PRINTLN_FILE("");   
      PRINTLN_FILE("----------- ISBD CALLBACK() ---------------- Frame cycle : ", actual_cycle, "/", FRAME_NUMBER, " ; Acquisition cycle : ", actual_acquisition+1, "/", ACQUISITION_NUMBER, "\r");
                    
      PRINTLN("");
      PRINTLN("----------- ISBD CALLBACK() ---------------- Frame cycle : ", actual_cycle, "/", FRAME_NUMBER, " ; Acquisition cycle : ", actual_acquisition+1, "/", ACQUISITION_NUMBER, "\r");                     
      store_data_in_arrays(actual_acquisition);                        // Do one acquisition and store data in arrays

      if(actual_acquisition < (ACQUISITION_NUMBER-1)){
        actual_acquisition++;                         // Incrementation of actual_acquisition for futures acquisitions (ACQUISITION_NUMBER times frames before Iridium sending) 
      }
      else{                                           // One frame is correctly completed with ACQUISITION_NUMBER (10) acquisitions
        calcul_and_filling_dataframe_write();         // Calculate average and rms values from tab and store the parameters in dataframe_write struct
        save_dataframe_to_sd();                       // Write the content of dataframe_write to the "dataFilename" file on the SD card
        actual_acquisition = 0;  

        if(actual_cycle < FRAME_NUMBER) actual_cycle++; // Incrementation of actual_cycle for futures frames (FRAME_NUMBER times frames before Iridium sending) 
        else actual_cycle = 1;                  
      } 
      PRINTLN_FILE("");
      counter1_acqu = millis()/1000;                // counter1_acqu used to calculate time between each acquisition  
      counter1_send = millis()/1000;                // counter1_send used to calculate time between each recovery Iridium sending
    } else{
      //LOG_INFO("Not yet ISBDCallback acquisition, counter1 :", counter1_acqu, ", counter2 : ", counter2_acqu, ", diff : ", (counter2_acqu - counter1_acqu), "\r");
    }   
  }
  return true;
}

#if DIAGNOSTICS
  void ISBDConsoleCallback(IridiumSBD *device, char c)
  {
    Serial.write(c);
  }

  void ISBDDiagsCallback(IridiumSBD *device, char c)
  {
    Serial.write(c);
  }
#else 
  void ISBDConsoleCallback(IridiumSBD *device, char c)
  {
    //Do nothing
  }

  void ISBDDiagsCallback(IridiumSBD *device, char c)
  {
    //Do nothing
  }
#endif

