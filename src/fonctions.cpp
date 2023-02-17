#include "fonctions.h" // Header

using namespace std;

/* Mode de fonctionnement, valeur par défault en cas de de defaut de lecture du fichier config */
int led_mode = 1;        // Utilise la Led pour contrôler ce qu'il se passe
int debug_mode = 1;      // Envoie les infos sur le moniteur série

/*---------- Carte Atlas EC EZO ----------*/
#define ecAddress 100 // Définition de l'adresse de la carte pour la communication I2C
int ecDelay = 300;    // Délais               
int ecDelay2 = 600;

Ezo_board EC = Ezo_board(100, "EC");  //create an EC circuit object who's address is 100 and name is "EC"
/* Paramètres non utilisés pour le moment 
byte ecCode = 0;                    // Used to hold the I2C response code.
byte ecInChar = 0;                  // Used as a 1 byte buffer to store in bound bytes from the EC Circuit.
char ecData[48];                    // We make a 48 byte character array to hold incoming data from the EC circuit.
char salData[48];                   // We make a 48 byte character array to hold incoming data from the EC circuit.
char tdsData[48];
char sgData[48];
char *ec;                           // Char pointer used in string parsing.
char *tds;                          // Char pointer used in string parsing.
char *sal;                          // Char pointer used in string parsing.
char *sg;                           // Char pointer used in string parsing.
*/

/*---------- Capteur de température BlueRobotics ----------*/
TSYS01 sensor_fastTemp; //Déclaration capteur de température BlueRobotics
float  wat_temp, fast_temp;

/*---------- GPS Grove v1.2 ----------*/
#define RXD 25            //Définition des ports de communication UART pour le GPS
#define TXD 26 
HardwareSerial neogps(1); // Creating GPS module instance
TinyGPSPlus gps;          //Objet TinyGPSPlus

/* ---------- Fonctions annexes ----------*/
/*SplitStr Pas utilisée pour le moment
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
*/

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
  Serial.println("I2C scanner. Scanning ...");
  byte count = 0;

  Wire.begin();
  for (byte i = 8; i < 127; i++)
  {
    Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
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
  Serial.println (" device(s).");
}

/* ---------- Fonctions liées à la carte Atlas EC EZO ----------*/
void mesureEC(){
  EC.send_read_cmd(); //send a read command
  delay(1000);
  if (EC_ENABLED){
    Serial.print("Conductivité : ");
    receive_and_print_reading(EC); 
    Serial.print(" uS/cm");
  }
  else if (TDS_ENABLED){
    Serial.print("Total dissolved solids : ");
    receive_and_print_reading(EC); 
    Serial.print(" ppm");
  }
  else if (SAL_ENABLED){
    Serial.print("Salinity : ");
    receive_and_print_reading(EC); 
    Serial.print(" PSU (between 0.00 - 42.00)");
  }
  else if (SG_ENABLED){
    Serial.print("Sea water gravity : ");
    receive_and_print_reading(EC); 
    Serial.print(" (between 1.00 - 1.300)");
  }
  Serial.println();
}

void setting_ec_probe_type() { 
  EC.send_cmd_with_num("K,", PROBE_TYPE);  //sends any command with the number appended as a string afterwards
  delay(ecDelay);
  Serial.println("Probe type : ");
  receive_and_print_response(EC); //used to handle receiving responses and printing them in a common format
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
}

void send_ec_cmd_and_response(char cmd[]) { 
  EC.send_cmd((const char*)cmd);
  delay(ecDelay);
  Serial.print("Response : ");
  receive_and_print_response(EC); 
  Serial.println();
}

/* Plus utilisées pour le moment Anciennes fonctions de mesure EC EZO
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

  // ec = strtok(ecData, ",");
  // Serial.print("EC value : "); Serial.println(ecData);
  // tds = strtok(NULL, ",");
  // Serial.print("TDS value : "); Serial.println(ecData);
  // sal = strtok(NULL, ",");
  // Serial.print("SAL value : "); Serial.println(ecData);
  // sg = strtok(NULL, ",");    // Let's pars the string at each comma.
  // Serial.print("SG value : "); Serial.println(ecData);

  //char trame = ecData.split(',');

  std::string s(ecData);
  string str = "25,56,33 35,42";
  SplitStr(str);

  // std::string ec_string = s.substr(0, s.find(delimiter));
  // std::string tds_string = s.substr(1, s.find(delimiter));
  // std::string sal_string = s.substr(2, s.find(delimiter));
  // std::string sg_string = s.substr(3, s.find(delimiter));
  
  // std::cout << "ec : " << ec_string << endl;
  // std::cout << "tds: " << tds_string << endl; 
  // std::cout << "sal: " << sal_string << endl;
  // std::cout << "sg: " << sg_string << endl;

  // ec = strtok(ecData, ",");
  // tds = strtok(ecData, ",");
  // sal = strtok(ecData, ",");
  // sg = strtok(ecData, ",");    // Let's pars the string at each comma.
  


  if (debug_mode==1) { 
    //Serial.print("EC value : "); Serial.println(ecData);
    //Serial.print("Sal value : "); Serial.println(sal);
    }
}

void request_ec(char computerData[]) {  
  if (debug_mode) Serial.println("--- EC Sensor :");
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
  // ec = strtok(ecData, ",");
  // tds = strtok(NULL, ",");
  // sal = strtok(NULL ",");
  // sg = strtok(NULL, ",");    // Let's pars the string at each comma.

  if (debug_mode==1) { 
    Serial.print("EC value : "); Serial.println(ecData);
    Serial.print("Sal value : "); Serial.println(salData);
    }
}
*/

/* ---------- Fonctions liées au capteur de température BlueRobotics ----------*/
void mesure_temp(){
  sensor_fastTemp.read();
  fast_temp = sensor_fastTemp.temperature();
  delay(200);
  if (debug_mode) {
    Serial.print("Température : ");
    Serial.print(fast_temp);
    Serial.println(" °C");
  }
}



/* ---------- Fonctions liées au GPS Grove v1.2 ----------*/
void init_gps(){
  neogps.begin(9600, SERIAL_8N1, RXD, TXD); // begin GPS hardware serial
}

void scanning_gps(){
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available()) // Si données gps disponibles
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
    print_pos_gps();
    print_date_gps();
  }
  else
  {
    if(debug_mode) Serial.println("No Data detected");
  }  
}

void print_pos_gps()
{     
  if (gps.location.isValid() == 1)
  {
    if(debug_mode){
      Serial.println("Données GPS : ");
      
      Serial.print("Coordonnées : (Lat)");
      Serial.print(gps.location.lat());

      Serial.print(", (Lng)");
      Serial.println(gps.location.lng());
      
      Serial.print("Nombre de satellites : ");
      Serial.print(gps.satellites.value());

      Serial.print(" | Altitude : ");
      Serial.print(gps.altitude.meters());
      Serial.print(" mètres");

      Serial.print(" | Vitesse : ");
      Serial.print(gps.speed.kmph());
      Serial.println("km/h");
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
    if(debug_mode){
      Serial.print(F("DATE : "));
      Serial.print(gps.date.day());
      Serial.print(F("/"));
      Serial.print(gps.date.month());
      Serial.print(F("/"));
      Serial.println(gps.date.year());
    }
  }
  else
  {
    if(debug_mode) Serial.println("GPS date is not valid");
  }  

  if (gps.time.isUpdated() == 1)
  {
    if(debug_mode){
      Serial.print(F("TIME (UTC+1) : "));
      Serial.print(gps.time.hour() + 1);
      Serial.print(F(":"));
      Serial.print(gps.time.minute());
      Serial.print(F(":"));
      Serial.println(gps.time.second());
    }
  }
  else
  {
    if(debug_mode) Serial.println("GPS time is not valid");
  }  

}