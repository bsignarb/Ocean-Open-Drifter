#include "fonctions.h"

//  mode de fonctionnement, valeur par défault en cas de de defaut de lecture du fichier config
int led_mode = 1;        // utilise la Led pour controler ce qui se passe
int debug_mode = 1;      // envoi les infos sur les liaison serie

// definition pour la carte Atlas
#define ecAddress 100
byte ecCode = 0;                    // Used to hold the I2C response code.
byte ecInChar = 0;                  // Used as a 1 byte buffer to store in bound bytes from the EC Circuit.
char ecData[48];                    // We make a 48 byte character array to hold incoming data from the EC circuit.
char salData[48];                    // We make a 48 byte character array to hold incoming data from the EC circuit.
char tdsData[48];
char sgData[48];
int ecDelay = 300;                  // Used to change the delay needed depending on the command sent to the EZO Class EC Circuit. 600 par defaut. It is the correct amount of time for the circuit to complete its instruction.
int ecDelay2 = 600;
char *ec;                           // Char pointer used in string parsing.
char *tds;                          // Char pointer used in string parsing.
char *sal;                          // Char pointer used in string parsing.
char *sg;                           // Char pointer used in string parsing.

Ezo_board EC = Ezo_board(100, "EC");      //create an EC circuit object who's address is 100 and name is "EC"

using namespace std;

void SplitStr(string str)
{
    string s = "";
    std::cout<<"The split string is:";
    for (auto x : str)
    {
        if (x == ',')
        {
            std::cout << s << endl;
            s = "";
        }
        else {
            s = s + x;
        }
    }
    std::cout << s << endl;
}

float get_voltage(){
  float vbatt = analogRead(vbatt_pin);
  vbatt *= 2;
  vbatt *= 3.3;
  vbatt /= 1024;
  return vbatt;
}

void mesureEC(){
  EC.send_read_cmd(); //send a read command
  delay(1000);
  Serial.print("Mesure : ");
  receive_and_print_reading(EC); //get the reading from the EC circuit
  Serial.println();
}

void setting_ec_probe_type(double probeType) { 
  EC.send_cmd_with_num("K,", probeType);
  delay(ecDelay);
  Serial.println("--- Probe type : ");
  receive_and_print_response(EC);
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
  Serial.print("Parameters enabled : ");
  receive_and_print_response(EC);
  Serial.println();
}


void mesureEClib() {
  if (debug_mode==1) Serial.println("--- EC Sensor :");
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

  //Serial.print("ECtotal value : "); Serial.println(ecData);

  /*ec = strtok(ecData, ",");
  Serial.print("EC value : "); Serial.println(ecData);
  tds = strtok(NULL, ",");
  Serial.print("TDS value : "); Serial.println(ecData);
  sal = strtok(NULL, ",");
  Serial.print("SAL value : "); Serial.println(ecData);
  sg = strtok(NULL, ",");    // Let's pars the string at each comma.
  Serial.print("SG value : "); Serial.println(ecData);*/

  //char trame = ecData.split(',');

  std::string s(ecData);
  string str = "25,56,33 35,42";
  SplitStr(str);

  /*std::string ec_string = s.substr(0, s.find(delimiter));
  std::string tds_string = s.substr(1, s.find(delimiter));
  std::string sal_string = s.substr(2, s.find(delimiter));
  std::string sg_string = s.substr(3, s.find(delimiter));
  
  std::cout << "ec : " << ec_string << endl;
  std::cout << "tds: " << tds_string << endl; 
  std::cout << "sal: " << sal_string << endl;
  std::cout << "sg: " << sg_string << endl;*/

  /*ec = strtok(ecData, ",");
  tds = strtok(ecData, ",");
  sal = strtok(ecData, ",");
  sg = strtok(ecData, ",");    // Let's pars the string at each comma.
  */


  if (debug_mode==1) { 
    //Serial.print("EC value : "); Serial.println(ecData);
    //Serial.print("Sal value : "); Serial.println(sal);
    }
}

void request_ec(char computerData[]) {  
  if (debug_mode==1) Serial.println("--- EC Sensor :");
  Wire.beginTransmission(ecAddress);   
  Wire.write(computerData); 
  delay(ecDelay2); 
  Wire.write('r');                      
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
  /*ec = strtok(ecData, ",");
  tds = strtok(NULL, ",");
  sal = strtok(NULL ",");
  sg = strtok(NULL, ",");    // Let's pars the string at each comma.*/

  if (debug_mode==1) { 
    Serial.print("EC value : "); Serial.println(ecData);
    Serial.print("Sal value : "); Serial.println(salData);
    }
}
      