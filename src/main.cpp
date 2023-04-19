#include "fonctions.h"

extern TSYS01 sensor_fastTemp;                                    // External declaration of the BlueRobotics temperature sensor
enum :byte {INIT, DEPLOYED, RECOVERY, COM, EMERGENCY} state_mode; // State machine to switch mode
uint8_t switch_state = 0;                                         // To switch state if MT message tell us           
int debug_mode_main = 1;                                          // Sends important informations to the serial monitor
bool first_time_emergency = true;                                 // To send emergency Iridium message only one time
int waiting_time = 0, waiting_counter1, waiting_counter2;         // How long has GPS been unavailable

int average_sending_time = 120;                                   // Default average sending time of 2 minutes = 120 seconds 
int margin_time = FRAME_NUMBER;                                   // FRAME_NUMBER seconds margin time (1sec min) for delay between each Iridium sending 
int delay_between_datalogging, counter1, counter2, last_sending_time = 120; // Delay calculation between each acquisition (according to the previous sending time)  
extern bool sending_ok;                                           // For Iridium => continue to send same buffer until it's sent (to avoid timeout and data lost)
bool first_init_passed = false;                                   // Differentiates first cycle of the Init from the following ones (and to know when first init succeeded for ISBDCallback() function)

void setup() {
  Serial.begin(115200);      // Start the console serial port 
  state_mode = INIT;         // Init state machine to deployed mode
  //WiFi.disconnect();
}

void loop() {     
  switch(state_mode){
    case INIT :              // Waiting state : pass in DEPLOYED mode as soon as the GPS has found its position
      if(debug_mode_main) Serial.println("--- Case INIT ---");
      init_cycle();
      //test_cycle_init();

      // Wait until GPS has found its position. If it doesn't found in a certain time, go to sleep to save battery life
      waiting_time = 0;
      waiting_counter1 = get_unix_time();          
      while((gps_available() == false) && (waiting_time < 300) ){  // While GPS not available and waiting_time < 5 minutes => Do nothing, just wait until one of the conditions is true
        waiting_counter2 = get_unix_time();
        waiting_time = waiting_counter2 - waiting_counter1;        // Calculate waiting time
        if(debug_mode_main) Serial.print("waiting time : ");       // Test (to delete)
        if(debug_mode_main) Serial.println(waiting_time);  
      }   
      if(waiting_time >= 300) state_mode = INIT;                   // If GPS not available since more than 5 minutes => Change to INIT state (2nd part for sleeping)
      else                    state_mode = DEPLOYED;               // GPS is available => Change to DEPLOYED state
      break;

    case DEPLOYED :                                                // Standard acquisition cycles
      if(debug_mode_main) Serial.println("--- Case DEPLOYED ---");
      deployed_cycle();
      //test_cycle();
      break;

    case RECOVERY :                                                // More frequent cycles
      if(debug_mode_main) Serial.println("--- Case RECOVERY ---");
      recovery_cycle();
      break;

    case COM :                                                     // Still really useful ?
      if(debug_mode_main) Serial.println("--- Case COM ---");
      // TODO : Envoi Iridium de la trame datachain 
      //        Conditions pour repasser en mode DEPLOYED ou RECOVERY (gestion du lien descendant)
      state_mode = DEPLOYED; 
      break;

    case EMERGENCY :
      if(debug_mode_main) Serial.println("--- Case EMERGENCY ---");
      emergency_cycle();
      if(get_voltage() > 2.95){                                    // Good battery level back
        state_mode = DEPLOYED;                                     // Change to DEPLOYED state
        if(debug_mode_main){
          Serial.print("Battery level good again : ");
          Serial.print(get_voltage());
          Serial.println("V\n");
        }                                      
      }
      delay(1000);
      break;
  }
  if(debug_mode_main) Serial.println("\n----------------------------------------------------------------------------\n");

  // ------------------------------------------ STATE CHANGES CONDITIONS ----------------------------------------------------- //
  // GPS position : Verify if GPS position is available. If it doesn't found in a certain time, go to sleep to save battery life
  waiting_time = 0;
  waiting_counter1 = get_unix_time();          
  while((gps_available() == false) && (waiting_time < 300) ){  // While GPS not available and waiting_time < 5 minutes => Do nothing, just wait until one of the conditions is true
    waiting_counter2 = get_unix_time();
    waiting_time = waiting_counter2 - waiting_counter1;        // Calculate waiting time
    Serial.print("waiting time : "); Serial.println(waiting_time);  // Test to delete
  }   
  if(waiting_time >= 300){                                      // If GPS not available since more than 5 minutes => Go to sleep for a certain time to save battery life 
    state_mode = INIT;                                          // Change to INIT state (2nd part for sleeping)
  } 
  else{                                                        // GPS is available
    state_mode = DEPLOYED;                                     // Change to DEPLOYED state
  }
  // Iridium MT messages
  if(switch_state == 49) state_mode = INIT;        // Change to INIT state (because of Iridium MT message) (dec "49" = char "1")
  if(switch_state == 50) state_mode = DEPLOYED;    // Change to DEPLOYED state (because of Iridium MT message) (dec "50" = char "2")
  if(switch_state == 51) state_mode = RECOVERY;    // Change to RECOVERY state (because of Iridium MT message) (dec "51" = char "3")
  if(switch_state == 52) state_mode = EMERGENCY;   // Change to EMERGENCY state (because of Iridium MT message) (dec "52" = char "4")
  
  // Battery voltage
  if(get_voltage() < 2.9 && get_voltage() > 0.5){  // Low battery level
    state_mode = EMERGENCY;                        // Change to EMERGENCY state
    if(debug_mode_main){
      Serial.print("Low battery level : ");
      Serial.print(get_voltage());
      Serial.println("V\n");
    }
  }
}

void init_cycle(){
  if(first_init_passed == false){ // First Init cycle
    pinMode(LED_BUILTIN, OUTPUT); 
    //pinMode(16, OUTPUT);   // Declare powerfull led port
    initEC();                // Init conductivity sensor
    delay(50);               // Allows time for the I2C electronic to initialize correctly
    Wire.begin();            // For I2C communication 
    sensor_fastTemp.init();  // Init temperature sensor
    init_gps();              // Init UART communication of GPS
    init_sd();               // Init and test SD card
    lecture_config();        // Read config file
    refresh_config_values(); // Refresh program values according to those read in config file
    init_iridium();          // Init Iridium module
    //init_ina219();         // Init INA219 (current sensor)

    first_init_passed = true;
  }
  else{                      // Following Init cycles for sleeping mode
    all_sleep(120);          // Sleeping for 2 minutes (120 seconds) because of no GPS signal
    //all_wakeup();
  }
}

void deployed_cycle(){
  //--- Datalogging ---//
  first_time_emergency = true;
  digitalWrite(2, LOW);                // Iridium modem sleeping
  for(int cycle_number=1; cycle_number<=FRAME_NUMBER; cycle_number++){  // FRAME_NUMBER times acquisition before Iridium sending
    if(debug_mode_main){
      Serial.print("Measure cycle number : ");
      Serial.print(cycle_number);
      Serial.print("/");
      Serial.println(FRAME_NUMBER);
      Serial.print("UnixTime : ");
      Serial.println(get_unix_time());
    }
    mesure_cycle_to_datachain();       // Start a measurement cycle and store the measured parameters in dataframe_write struct
    save_datachain_to_sd();            // Write the content of dataframe_write to the "dataFilename" file on the SD card
    
    delay_between_datalogging = 1000 * ( ( /*last_sending_time*/ 14 + margin_time ) / FRAME_NUMBER ); // Delay calculation between each acquisition (according to the previous sending time)  
    if(debug_mode_main){
      Serial.print("delay between each datalog : ");
      Serial.print(delay_between_datalogging);
      Serial.println(" ms\n");
    }
    delay(delay_between_datalogging);  // Delay between each acquisition    
  }

  readSDbinary_to_struct();            // Read datalog file in a dataframe_read structure for Iridium sending

  //--- Iridium sending ---//
  counter1 = get_unix_time();          // Counter1 to calculate the Iridium sending time 
  print_iridium_infos();               // Iridium modem informations
  sending_ok = false;
  /*while(sending_ok == false)         // Continue to send same buffer until it's sent (to avoid timeout and data lost)
  {
    sendreceive_deployed_iridium();    // Sending data with buffer_read_340 (MO message) AND receiving data with receive_buffer (MT message)
  }*/
  counter2 = get_unix_time();               // Counter2 to calculate the Iridium sending (and receive?) time 
  last_sending_time = counter2 - counter1;  // Iridium sending (and receive?) time 
  if(debug_mode_main) Serial.print("Last sending time : ");
  if(debug_mode_main) Serial.println(last_sending_time);
  
  if(!check_rtc_set()){                // If RTC not correctly set
    set_rtc_by_gps();                  // RTC update by gps time
  }
}

void recovery_cycle(){
  //--- Datalog - Even in Recovery cycle we continue acquisitions and save it to SD card ---// 
  mesure_cycle_to_datachain();   // Start a measurement cycle and stores the measured parameters in datachain
  save_datachain_to_sd();        // Writes the content of datachain to the dataFilename file on the SD card

  //--- Iridium sending ---//
  readGPS_to_buffer();           // Read time and GPS data and save it in a buffer 
  print_iridium_infos();         // Iridium modem informations
  sending_ok = false;
  while(sending_ok == false)     // Continue to send same buffer until it's sent (to avoid timeout and data lost)
  {
    //sendreceive_recovery_iridium();  // Send directly time and position
    sendreceive_deployed_iridium();    // Sending data with buffer_read_340 (MO message) AND receiving data with receive_buffer (MT message)
  }
  delay(20000);                  // Wait for 10s before next acquisition 
}

void emergency_cycle(){
  if(first_time_emergency == true){
    send_text_iridium("WARNING - LOW VOLTAGE");  // Only one time Iridium sending to inform low voltage battery 
    first_time_emergency = false;
  } 
}

void test_cycle_init(){
  Wire.begin();                  // For I2C communication 
  sensor_fastTemp.init();        // Init temperature sensor
  //init_gps();                  // Init communication UART of GPS
  //initEC();
}

void test_cycle(){
  //mesure_cycle_to_datachain(); // Start a measurement cycle and store the measured parameters in dataframe_write struct
  //save_datachain_to_sd();      // Write the content of dataframe_write to the "dataFilename" file on the SD card

  print_iridium_infos();         // Iridium modem informations
  receive_iridium();
  delay(10000);
}

bool ISBDCallback()
{
  //unsigned ledOn = (millis() / 1000) % 2;  // Every second LED blinking 
  //digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);

  //--- Datalog - Even when sending with Iridium we continue acquisitions and save it to SD card ---// 
  if(first_init_passed == true){   // Do when first init succeeded
    mesure_cycle_to_datachain();   // Start a measurement cycle and stores the measured parameters in datachain
    save_datachain_to_sd();        // Writes the content of datachain to the dataFilename file on the SD card
    delay(1000);
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
    //Serial.write(c);
  }

  void ISBDDiagsCallback(IridiumSBD *device, char c)
  {
    //Serial.write(c);
  }
#endif

