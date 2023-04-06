#include "fonctions.h"

extern TSYS01 sensor_fastTemp;                                    // External declaration of the BlueRobotics temperature sensor
enum :byte {INIT, DEPLOYED, RECOVERY, COM, EMERGENCY} state_mode; // State machine to switch mode
bool init_first_cycle = 0;                                        // Differentiates first cycle of the Init from the following ones 
int debug_mode_main = 1;                                               // Sends important informations to the serial monitor

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
      
      state_mode = DEPLOYED; // Change to DEPLOYED state
      break;

    case DEPLOYED :          // Standard acquisition cycles
      if(debug_mode_main) Serial.println("--- Case DEPLOYED ---");
      deployed_cycle();
      //test_cycle();

      // TODO Condition pour passer en mode RECOVERY
      // TODO Condition pour passer en mode SENDING
      break;

    case RECOVERY :          // More frequent cycles
      if(debug_mode_main) Serial.println("--- Case RECOVERY ---");
      recovery_cycle();
      
      // TODO Condition pour passer en mode DEPLOYED
      break;

    case COM :
      if(debug_mode_main) Serial.println("--- Case COM ---");
      // TODO Envoi Iridium de la trame datachain 
      // Envoi toutes 5min environ ?
      // Ici qu'on convertit trame en binaire ou avant ?
      // TODO Conditions pour repasser en mode DEPLOYED ou RECOVERY (gestion du lien descendant)
      state_mode = DEPLOYED; 
      break;

    case EMERGENCY :
      if(debug_mode_main) Serial.println("--- Case EMERGENCY ---");

      break;
  
    if(debug_mode_main) Serial.println("\n----------------------------------------------------------------------------\n");
  }
}

void init_cycle(){
  if(init_first_cycle == 0){ // First Init cycle
    pinMode(LED_BUILTIN, OUTPUT); 
    pinMode(16, OUTPUT);     // Declare powerfull led port
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

    //pinMode(2, OUTPUT);    // Sleep Iridium modem
    //digitalWrite(2, LOW);

    init_first_cycle = 1;
  }
  else{                      // Following Init cycles for sleeping mode
    // TODO veille si pas de signal GPS 

  }

  if(!check_rtc_set()){    // If RTC not correctly initialized
    set_rtc_by_gps();      // RTC update via gps data
  }
}

void deployed_cycle(){
  //Serial.println("--- EC sensor and gps ON ---");
  //digitalWrite(9, HIGH);        // GPS ON
  //digitalWrite(4, HIGH);        // EC sensor ON

  //--- Datalogging ---//
  for(int cycle_number=1; cycle_number<=FRAME_NUMBER; cycle_number++){  // FRAME_NUMBER times acquisition before Iridium sending
    if(debug_mode_main){
      Serial.print("Measure cycle number : ");
      Serial.print(cycle_number);
      Serial.print("/");
      Serial.println(FRAME_NUMBER);
    }
    mesure_cycle_to_datachain(); // Starts a measurement cycle and store the measured parameters in dataframe_write struct
    save_datachain_to_sd();      // Writes the content of dataframe_write to the "dataFilename" file on the SD card
    delay(1000);                 // Wait for 1 second before next acquisition
  }

  readSDbinary_to_struct();      // Read datalog file in a dataframe_read structure for Iridium sending

  //--- Iridium sending ---//
  //print_iridium_infos();
  //send_binary_iridium();
  //send_text_iridium();

  if(!check_rtc_set()){          // if RTC not correctly initialized
    set_rtc_by_gps();            // RTC update via gps data
  }
}

void recovery_cycle(){
  //--- Datalogging ---//
  mesure_cycle_to_datachain();   // Starts a measurement cycle and stores the measured parameters in datachain
  save_datachain_to_sd();        // Writes the content of datachain to the dataFilename file on the SD card
  readSDbinary_to_struct();      // Read datalog file in a dataframe_read structure for Iridium sending

  //--- Iridium sending ---//
  print_iridium_infos();
  //send_binary_iridium();
  //send_text_iridium();

  delay(500);                   // Wait for 500 ms before next acquisition 
}

void test_cycle_init(){
  Wire.begin();                      // For I2C communication 
  sensor_fastTemp.init();            // Init temperature sensor
  //init_gps();                      // Init communication UART of GPS
  //initEC();
}

void test_cycle(){
  
}


