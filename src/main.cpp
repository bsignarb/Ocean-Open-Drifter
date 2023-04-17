#include "fonctions.h"

extern TSYS01 sensor_fastTemp;                                    // External declaration of the BlueRobotics temperature sensor
enum :byte {INIT, DEPLOYED, RECOVERY, COM, EMERGENCY} state_mode; // State machine to switch mode
uint8_t switch_state = 0;
bool init_first_cycle = 0;                                        // Differentiates first cycle of the Init from the following ones 
int debug_mode_main = 1;                                          // Sends important informations to the serial monitor

int average_sending_time = 120;                                   // Default average sending time of 2 minutes = 120 seconds 
int margin_time = FRAME_NUMBER;                                   // FRAME_NUMBER seconds margin time (1sec min) for delay between each Iridium sending 
int delay_between_datalogging, counter1, counter2, last_sending_time = 120;
extern bool sending_ok;
int test_isbd_callback = 0;


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
      // TODO : Wait until GPS has found its position, if it don't found in a certain time go to other state
      //test_cycle_init();  
      
      state_mode = DEPLOYED; // Change to DEPLOYED state
      break;

    case DEPLOYED :          // Standard acquisition cycles
      if(debug_mode_main) Serial.println("--- Case DEPLOYED ---");
      deployed_cycle();
      //test_cycle();

      // TODO : Condition pour passer en mode RECOVERY
      //        Condition pour passer en mode COM
      
      if(get_voltage() < 2.9 && get_voltage() > 0.5){    // Low battery level
        if(debug_mode_main){
          Serial.print("Low battery level : ");
          Serial.print(get_voltage());
          Serial.println("V\n");
        }
        state_mode = EMERGENCY;                          // Change to EMERGENCY state
      }
      break;

    case RECOVERY :                                      // More frequent cycles
      if(debug_mode_main) Serial.println("--- Case RECOVERY ---");
      recovery_cycle();
      
      // TODO : Condition pour passer en mode DEPLOYED
      state_mode = RECOVERY; 
      break;

    case COM : // Really useful ?
      if(debug_mode_main) Serial.println("--- Case COM ---");
      // TODO : Envoi Iridium de la trame datachain 
      //        Conditions pour repasser en mode DEPLOYED ou RECOVERY (gestion du lien descendant)
      state_mode = DEPLOYED; 
      break;

    case EMERGENCY :
      if(debug_mode_main) Serial.println("--- Case EMERGENCY ---");

      if(get_voltage()>2.9){    // Good battery level back
        if(debug_mode_main){
          Serial.print("Battery level good again : ");
          Serial.print(get_voltage());
          Serial.println("V\n");
        }
        state_mode = DEPLOYED;  // Change to DEPLOYED state
      }
      delay(1000);
      break;
  }
  if(debug_mode_main) Serial.println("\n----------------------------------------------------------------------------\n");

  if(switch_state == 49) state_mode = INIT;       // Change to INIT state (because of Iridium MT message) (dec 49 = char 1)
  if(switch_state == 50) state_mode = DEPLOYED;   // Change to DEPLOYED state (because of Iridium MT message) (dec 50 = char 2)
  if(switch_state == 51) state_mode = RECOVERY;   // Change to RECOVERY state (because of Iridium MT message) (dec 51 = char 3)
  if(switch_state == 52) state_mode = EMERGENCY;  // Change to EMERGENCY state (because of Iridium MT message) (dec 52 = char 4)
}

void init_cycle(){
  if(init_first_cycle == 0){ // First Init cycle
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

    init_first_cycle = 1;
  }
  else{                      // Following Init cycles for sleeping mode
    // TODO : sleeping if no GPS signal
    //all_sleep();
    // TODO : wait a certain time or test if GPS signal is back to return in the "normal" state
  }
}

void deployed_cycle(){
  //--- Datalogging ---//
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
  // TODO : Make sending non-blocking, to continue the acquisitions (with bool ISBDCallback()?)
  //        Necessary to switch in COM state ?
  test_isbd_callback = 1;              // Test for ISBDCallaback() for multitask (to delete soon)
  counter1 = get_unix_time();          // Counter1 to calculate the Iridium sending time 
  print_iridium_infos();               // Iridium modem informations
  sending_ok = false;
  /*while(sending_ok == false)           // Continue to send same buffer until it's sent (to avoid timeout and data lost)
  {
    sendreceive_binary_iridium();      // Sending data with buffer_read_340 (MO message) AND receiving data with receive_buffer (MT message)
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
  //--- Datalogging - Even in Recovery cycle we continue acquisitions and save it to SD card ---// 
  mesure_cycle_to_datachain();   // Start a measurement cycle and stores the measured parameters in datachain
  save_datachain_to_sd();        // Writes the content of datachain to the dataFilename file on the SD card

  //--- Iridium sending ---//
  readGPS_to_buffer();           // Read time and GPS data and save it in a buffer 
  print_iridium_infos();         // Iridium modem informations
  sending_ok = false;
  while(sending_ok == false)     // Continue to send same buffer until it's sent (to avoid timeout and data lost)
  {
    sendreceive_text_iridium();  // Send directly time and position
  }
  delay(10000);                  // Wait for 10s before next acquisition 
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
/*
bool ISBDCallback()
{
  //if(test_isbd_callback == 1){
    //unsigned ledOn = (millis() / 1000) % 2;  // Blink LED every second
    //digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
    //return true;
  //}
}
*/
