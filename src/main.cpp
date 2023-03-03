#include "fonctions.h"

extern TSYS01 sensor_fastTemp; // External declaration of the BlueRobotics temperature sensor

extern const int control_pin_EC; 

int i = 0;
enum :byte {IDLE, DEPLOYED, RECOVERY, SENDING} state_mode; // State machine to switch mode

//const int led = D9;
//unsigned compteur = 0;
//const int greenled = 25;  // on utilise la Led interne à la board esp32

void setup() {
  Serial.begin(115200);              // For serial monitor

  pinMode(LED_BUILTIN, OUTPUT); 

  pinMode(control_pin_EC, OUTPUT);   // initiate on Atlas EC
  digitalWrite(control_pin_EC, HIGH);

  Wire.begin();                      // For I2C communication 
  sensor_fastTemp.init();            // Init temperature sensor
  init_gps();                        // Init communication UART of GPS

  //set_date_rtc(0, 0, 0, 2005);     // Time disruption test to check the gps update 
  set_rtc_by_gps();                  // RTC update via gps data

  test_sd();                         // Init and test SD card
  lecture_config();                  // Read config file
  refresh_config_values();           // Refreshes the program values according to those read in the config file

  state_mode = DEPLOYED;             // Init state machine to deployed mode

  //init_ina219();                     // Init INA219 (current sensor)
  WiFi.disconnect();
 
}

void loop() {     
  /*enable_ec_parameters(EC_ENABLED, TDS_ENABLED, SAL_ENABLED, SG_ENABLED); 
  mesureEC();
  mesure_temp();*/
  
  if(!check_rtc_set()){  // if RTC not correctly initialized
    set_rtc_by_gps();    // RTC update via gps data
  }

  switch(state_mode){
    case IDLE :          // Waiting state : pass in DEPLOYED mode as soon as the GPS has found its position
      deployed_cycle();  
      break;

    case DEPLOYED :      // Standard acquisition cycle
      deployed_cycle();
      //get_current();   
      //scanner_i2c_adress(); 
      /*all_sleep();       // Sleeping mode
      Serial.println("Mesures capteur ec en veille : ");
      get_current(); 
      delay(3000);       // Wait for 3 seconds before next acquisition (in ms)
      all_wakeup(); */     // Waking up

      delay(2000); 

      // TODO Condition pour passer en mode RECOVERY
      // TODO Condition pour passer en mode SENDING

      break;

    case RECOVERY :      // More frequent cycles
      recovery_cycle();
      delay(1000);       // Wait for 1 second before next acquisition (in ms)

      // TODO Condition pour passer en mode DEPLOYED
      break;

    case SENDING :
        // TODO Envoi Iridium de la trame datachain 
        // TODO Conditions pour repasser en mode DEPLOYED ou RECOVERY
        state_mode = DEPLOYED; 
      break;
  

    
    
    Serial.println("\n----------------------------------------------------------------------------\n");
  }

  // Tests...
  /*Serial.println("led on\n");
  send_ec_cmd_and_response("L,1"); // EC sensor led on
  mesureEC(); 
  delay(3000);
  Serial.println("led off\n");
  send_ec_cmd_and_response("L,0"); // EC sensor led on
  delay(3000);*/
 

}





