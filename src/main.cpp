#include <Arduino.h>
#include "fonctions.h"

#include <Wire.h>  // enable I2C

//const int led = D9;
//unsigned compteur = 0;

// déclaration pour gestion des Led et interrupteur
//const int greenled = 25;  // on utilise la Led interne à la board esp32

//  mode de fonctionnement, valeur par défault en cas de de defaut de lecture du fichier config
int led_mode = 1;        // utilise la Led pour controler ce qui se passe
int debug_mode = 1;      // envoi les infos sur les liaison serie

// definition pour la carte Atlas
#define ecAddress 100
byte ecCode = 0;                    // Used to hold the I2C response code.
byte ecInChar = 0;                  // Used as a 1 byte buffer to store in bound bytes from the EC Circuit.
char ecData[48];                    // We make a 48 byte character array to hold incoming data from the EC circuit.
int ecDelay = 600;                  // Used to change the delay needed depending on the command sent to the EZO Class EC Circuit. 600 par defaut. It is the correct amount of time for the circuit to complete its instruction.
char *ec;                           // Char pointer used in string parsing.
char *tds;                          // Char pointer used in string parsing.
char *sal;                          // Char pointer used in string parsing.
char *sg;                           // Char pointer used in string parsing.

void mesureEC();


void setup() {
  Serial.begin(115200);
      Wire.begin();  // for I2C communication


      delay(500);
      
}

void loop() {
  //Serial.println("Boucle" + String(compteur));
  /*digitalWrite(led, LOW);
  delay(1000);
  digitalWrite(led, HIGH);
  delay(3000);
  compteur++;*/

  //float v = get_voltage(); 
  //Serial.println("Vbat : " + String(v));

  // Creating datachain 
  String datachain = "";
  mesureEC(); 
  datachain += ecData; datachain += ";"; 

  Serial.println("Mesure EC : " + datachain);

}


void mesureEC() {
  //if (debug_mode==1) Serial.println("--- EC Sensor :");
  Wire.beginTransmission(ecAddress);   // Call the circuit by its ID number.
  Wire.write('r');                     // r for reading sensor
  Wire.endTransmission();              // End the I2C data transmission.
  delay(ecDelay);                      // Reading time needing, 600 by default
  Wire.requestFrom(ecAddress, 48, 1);  // Call the circuit and request 48 bytes (this is more than we need)
  ecCode = Wire.read();                // The first byte is the response code, we read this separately.
  byte i = 0;                          // Counter used for EC_data array.
  while (Wire.available()) {           // Are there bytes to receive.
    ecInChar = Wire.read();            // Receive a byte.
    ecData[i] = ecInChar;              // Load this byte into our array.
    i += 1;                            // Incur the counter for the array element.
    if (ecInChar == 0) {               // If we see that we have been sent a null command.
      i = 0;                           // Reset the counter i to 0.
      Wire.endTransmission();          // End the I2C data transmission.
      break;                           // Exit the while loop.
    }
  }
  switch (ecCode) {                    // Switch case based on what the response code is.
    case 1:                            // Decimal 1.
      if (debug_mode==1) Serial.println("EC Success");    // Means the command was successful.
      break;                           // Exits the switch case.
    case 2:                            // Decimal 2.
      if (debug_mode==1) Serial.println("EC Failed");     // Means the command has failed.
      break;                           // Exits the switch case.
    case 254:                          // Decimal 254.
      if (debug_mode==1) Serial.println("EC Pending");    // Means the command has not yet been finished calculating.
      break;                           // Exits the switch case.
    case 255:                          // Decimal 255.
      if (debug_mode==1) Serial.println("EC No Data");    // Means there is no further data to send.
      break;                           // Exits the switch case.
  }
  ec = strtok(ecData, ",");
  tds = strtok(NULL, ",");
  sal = strtok(NULL, ",");
  sg = strtok(NULL, ",");    // Let's pars the string at each comma.

  //if (debug_mode==1) { Serial.print("EC value : "); Serial.println(ecData);}

}