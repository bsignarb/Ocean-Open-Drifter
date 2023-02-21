#include "fonctions.h" // Header

using namespace std;
/* ------------------------------------------------- DECLARATIONS ------------------------------------------------------------------*/

/*---------- Déclarations générales ----------*/ 
const int greenled = 25;   // Led d'information
const int control_pin = 4; // pin qui controle l'allumage du régulateur

int led_mode = 1;          // Utilise la Led pour contrôler ce qu'il se passe
int debug_mode = 1;        // Envoie les infos sur le moniteur série

int TIME_TO_SLEEP = 10;    // Durée d'endormissement entre 2 cycles complets de mesures (en sec) (rédéfinie ensuite par le fichier config)
int nbrMes = 3;            // Nombre de mesures à effectuer (rédéfinie ensuite par le fichier config)
int bootCount = 0;         // Utile pour avoir un 1er cycle d'écriture dans fichier différent des cycles suivants

/*---------- Carte Atlas EC EZO ----------*/
#define ecAddress 100                // Définition de l'adresse de la carte pour la communication I2C
int ecDelay = 300;                   // Définition des délais               
int ecDelay2 = 600;

Ezo_board EC = Ezo_board(100, "EC"); // Création d'un object EC de la classe Ezo_board d'adresse 100 

float conductivity, total_dissolved_solids, salinity, seawater_gravity;

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
TSYS01 sensor_fastTemp;                           //Déclaration capteur de température BlueRobotics
float fast_temp;

/*---------- GPS Grove v1.2 + horloge atomique ----------*/
#define RXD 25                                    //Définition des ports de communication UART pour le GPS
#define TXD 26 
HardwareSerial neogps(1);                         // Creation instance module GPS
TinyGPSPlus gps;                                  // Création d'un objet de la classe TinyGPSPlus

float lattitude, longitude, altitude, vitesse;
int nb_satellites;
String coord;                                     // Pour format de position GPS en 1 seule écriture (coord = "Lattitude,Longitude")

String second_gps, minute_gps, hour_gps, day_gps, month_gps, year_gps;    // Pour format de date en plusieurs variables
String datenum_gps;                                   // Pour format de date en 1 seule écriture (datenum = "day/month/year")
String datetime_gps;                                  // Pour format de date en 1 seule écriture (datetime = "hour:minute:second")

/*---------- Carte SD et fichier config ----------*/
String datachain = "";                                                                 // Chaine de donnée pour le stockage des paramètres mesurés
const int cspin_SD=5;                                                                  // Signal de sélection bus SPI
String id_logger, number_measures, delay_batch, led_mode_sd, debug_mode_sd ,clef_test; // Variables du fichier config.txt
File confFile;                                                                         // Pour lecture du fichier config.txt

String fichier_config = "/config.txt";                                                 // Nom du fichier de configuration
String dataFilename = "/datalog.txt";                                                  // Nom du fichier de données

/*---------- RTC DS3231 Adafruit ----------*/
DS3231 Clock;                                                           // Création d'un objet de la classe DS3231
bool Century = false;
bool h12;
bool PM;

String second_rtc, minute_rtc, hour_rtc, day_rtc, month_rtc, year_rtc;  // Pour format de date en plusieurs variables
String datenum_rtc;                                                     // Pour format de date en 1 seule écriture (datenum = "day/month/year")
String timenum_rtc;                                                     // Pour format de date en 1 seule écriture (datetime = "hour:minute:second")
String datetime_rtc;       

bool rtc_set = false;                                                   // Permet de savoir si la RTC a bien été initialisée

/* ------------------------------------------------- FONCTIONS ------------------------------------------------------------------*/

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

void led_blinkled(int nbr, int freq) {   // freq en sec = freq interval between to light on
  for (int i=0; i<nbr; i++) {
    digitalWrite(greenled, HIGH); 
    delay(freq/2);
    digitalWrite(greenled, LOW);
    delay(freq/2);
  }
}

void all_sleep(){
  digitalWrite(greenled, LOW);
  digitalWrite(control_pin, LOW); 
  delay(300);                       // on laisse le temps au régulateur de bien s'éteindre avant de passer en deep sleep
}

void wake_up(){
  digitalWrite(greenled, HIGH);
  digitalWrite(control_pin, HIGH); 
  delay(500);   
}

/* ---------- Fonctions liées à la carte Atlas EC EZO ----------*/
void mesureEC(){
  EC.send_read_cmd(); // Envoie une demande de lecture au capteur
  delay(1000);
  if (EC_ENABLED){
    EC.receive_read_cmd(); 
    conductivity = EC.get_last_received_reading(); // Retourne la dernière lecture du capteur en float
    if(debug_mode){
      Serial.print("Conductivité : ");
      Serial.print(conductivity);
      //receive_and_print_reading(EC); 
      Serial.print(" uS/cm");
    }
  }
  else if (TDS_ENABLED){
    EC.receive_read_cmd(); 
    total_dissolved_solids = EC.get_last_received_reading();
    if(debug_mode){
      Serial.print("Total dissolved solids : ");
      Serial.print(total_dissolved_solids);
      //receive_and_print_reading(EC); 
      Serial.print(" ppm");
    }
  }
  else if (SAL_ENABLED){
    EC.receive_read_cmd();
    salinity = EC.get_last_received_reading();
    if(debug_mode){
      Serial.print("Salinity : ");
      Serial.print(salinity);
      //receive_and_print_reading(EC); 
      Serial.print(" PSU (between 0.00 - 42.00)");
    }
  }
  else if (SG_ENABLED){
    EC.receive_read_cmd();
    seawater_gravity = EC.get_last_received_reading();
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
  if(debug_mode){
    Serial.print("Parameters enabled : ");
    receive_and_print_response(EC);
  }
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

void scanning_gps_time(){
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

    // Concaténation en coord pour faciliter le traitement sur datachain (coord = "Lattitude,Longitude")
    coord = "(Lat)";   
    coord += lattitude; 
    coord += ",(Long)";
    coord += longitude; 

    if(debug_mode){
      Serial.print("Données GPS : ");
      
      Serial.print(coord);
      
      Serial.print(" | Nombre de satellites : ");
      Serial.print(nb_satellites);

      Serial.print(" | Altitude : ");
      Serial.print(altitude);
      Serial.print(" mètres");

      Serial.print(" | Vitesse : ");
      Serial.print(vitesse);
      Serial.println(" km/h");
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
    
    // Concaténation en datenum pour faciliter le traitement sur datachain (datenum = "day/month/year")
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

  if (gps.time.isUpdated() == 1)
  {
    hour_gps = gps.time.hour() + 1;
    minute_gps = gps.time.minute();
    second_gps = gps.time.second();

    // Concaténation en datetime pour faciliter le traitement sur datachain (datetime = "hour:minute:second")
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

/* ---------- Fonctions liées à la carte SD et au fichier de config ----------*/
void test_sd(){
  if (debug_mode) Serial.print("Initializing SD card... : ");   
  if (!SD.begin(cspin_SD)) {                                           // Vérifie que la carte SD soit présente et puisse être initialisée (pin 5 par défaut)             
    if (debug_mode) Serial.println("Card failed, or not present");
    if (led_mode==1){
      errormessage_sd();
    }
  }
  if (debug_mode) Serial.println("card initialized.");  
  delay(300);  
}

void errormessage_sd(){
  for (int i=0; i <= 8; i++){                    // dans la boucle for il y a 1s, donc on a 8 seconde au total
      digitalWrite(greenled, HIGH); delay(400);
      //led_blinked(3,100);
      digitalWrite(greenled, LOW); delay (300);
  }
}

void lecture_config(){
  confFile = SD.open(fichier_config, FILE_READ); // Ouvre fichier config.txt sur carte SD

  char phrase[200];
  byte index = 0;
  char x=0;
  String reste = "";
  int k=0;
  
  if (confFile) {
    if(debug_mode) Serial.println("Opening "+fichier_config);
    while (confFile.available()) {
      x = confFile.read();
      if (x!='\n') {       // Si pas de retour à la ligne
        phrase[index] = x; // On remplit phrase[] des caractères de la ligne actuelle du fichier
        index++;
      } else {             // Dés qu'on a saut de ligne, on traite phrase[] pour en extraire les infos
        if (index != 0) {
          reste = phrase;
          reste = reste.substring(0,index); // Créé sous chaine "reste" recopiant phrase[] de 0 jusqu'à index
          index=0;
          // Suppression des commentaires
          k = reste.indexOf(";");           // Retourne l'index (la position) du caractère ";"
          if (k!=0) {                       // S'il n'est pas en tout début de ligne
            if (k!=-1) {                    // S'il existe bel et bien
              reste = reste.substring(0,k); // Crée une sous chaine de ce qu'il est écrit avant ";"
            }
            reste.trim();                   // Supprime les espaces au début et à la fin
            
            // Extrait les valeurs pour les placer dans les variables du programme
            if (reste.indexOf('=') >0) {                             // Signe égal trouvé
              String clef = reste.substring(0,reste.indexOf('='));   // Crée une sous chaine de ce qui se trouve avant "="
              String valeur = reste.substring(reste.indexOf('=')+1); // Crée une sous chaine de ce qui se trouve après "="
              if (clef == "id_logger") id_logger = valeur;           // Association des variables
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
    confFile.close(); // Ferme le fichier config
  } else {
    Serial.println("error opening "+fichier_config);
    while(1) errormessage_sd();
  }
}

void refresh_config_values(){
  TIME_TO_SLEEP=delay_batch.toInt();  // Association des variables du programme avec celles lues sur la carte SD
  nbrMes=number_measures.toInt();            
  debug_mode=debug_mode_sd.toInt();  
  led_mode=led_mode_sd.toInt(); 
}

void mesure_cycle_to_datachain(){
  reading_rtc(); // Acquisition date et heure via gps   

  // Crée une nouvelle datachain pour stocker les valeurs mesurées
  datachain = "";                               // Init datachain
  datachain += datetime_rtc;                    // Ecrit date et heure dans datachain     
  
  enable_ec_parameters(EC_ENABLED, TDS_ENABLED, SAL_ENABLED, SG_ENABLED); 

  // Lit les capteurs plusieurs fois et ajoute leurs valeurs à datachain
  for(int n=1; n<=nbrMes; n++){
    if(debug_mode){Serial.print("--- Mesure n° : "); Serial.print(n); Serial.print("/");Serial.println(nbrMes);}

    mesureEC();          // Mesure capteur Atlas ec ezo
    mesure_temp();       // Mesure capteur de température
    scanning_gps_coord(); // Acquisition coordonnées gps
    if(debug_mode) Serial.println("");

    datachain += " | ";  
    datachain += coord; datachain += " ; ";       // Ecrit coordonnées dans datachain    
    datachain += fast_temp; datachain += "°C ; "; // Ecrit température dans datachain
    
    // Ecriture ec ezo en fonction des paramètres enabled
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
  }

  // Affiche datachain sur le terminal
  if (debug_mode==1) {
    Serial.print("Datachain complétée : "); 
    Serial.println(datachain);
    Serial.println();
  }
}

void save_datachain_to_sd(){
  File dataFile = SD.open(dataFilename, FILE_APPEND);    // FILE_APPEND pour esp32, FILE_WRITE pour arduino
  if (dataFile) {                                        // Si fichier dispo, écrit le contenu de la datachain dans le fichier
    dataFile.println(datachain);
    dataFile.close();
    if (debug_mode==1) Serial.println("Fichier créé avec succès");
    if (debug_mode==1) {Serial.print("Filename : "); Serial.println(dataFilename); Serial.println();}
  }
  else {                                                 // Si fichier non ouvert, afficher erreur
    if (debug_mode==1) Serial.println("error opening file");
    for (int i=0; i<=5; i++){
        errormessage_sd();
    }
  }
  delay(400);
}

/* ---------- Fonctions liées à la RTC DS3231 Adafruit ----------*/
void reading_rtc() {                         
  int sec = Clock.getSecond();
  if (sec < 10) {
    second_rtc = String(0) + String(sec);
  }
  else {
    second_rtc = sec;
  }
  int minu = Clock.getMinute();
  if (minu < 10) {
    minute_rtc = String(0) + String(minu);
  }
  else {
    minute_rtc = minu;
  }
  int heure = Clock.getHour(h12, PM);
  if (heure < 10) {
    hour_rtc = String(0) + String(heure);
  }
  else {
    hour_rtc = heure;
  }
  int jour = Clock.getDate();
  if (jour < 10) {
    day_rtc = String(0) + String(jour);
  }
  else {
    day_rtc = jour;
  }
  int mois = Clock.getMonth(Century);
  if (mois < 10) {
    month_rtc = String(0) + String(mois);
  }
  else {
    month_rtc = mois;
  }
  year_rtc = Clock.getYear();
  datenum_rtc = ""; datenum_rtc += year_rtc; datenum_rtc += month_rtc; datenum_rtc += day_rtc;
  timenum_rtc = ""; timenum_rtc += hour_rtc; timenum_rtc += minute_rtc; timenum_rtc += second_rtc;
  datetime_rtc = ""; datetime_rtc += day_rtc; datetime_rtc += "/"; datetime_rtc += month_rtc; datetime_rtc += "/20"; datetime_rtc += year_rtc; datetime_rtc += " "; datetime_rtc += hour_rtc; datetime_rtc += ":"; datetime_rtc += minute_rtc; datetime_rtc += ":"; datetime_rtc += second_rtc;

  if (debug_mode==1) {
    Serial.println("\nRTC values :");
    Serial.print("Date RTC : "); Serial.println(datenum_rtc);
    Serial.print("Time RTC : "); Serial.println(timenum_rtc);
    Serial.print("DateTime RTC "); Serial.println(datetime_rtc);
    Serial.println();
  }
}

void set_time_rtc(byte hour, byte min, byte sec){
  Clock.setHour(hour);
  Clock.setMinute(min);
  Clock.setSecond(sec);
}

void set_date_rtc(byte day_of_month, byte day_of_week, byte month, byte year){
  Clock.setDate(day_of_month);
  Clock.setDoW(day_of_week);
  Clock.setMonth(month);
  Clock.setYear(year);
}

void set_rtc_by_gps(){
  if(debug_mode) {
    Serial.println("Setting RTC from GPS");
  }
  if (gps.date.isValid() && gps.time.isValid())
  {
    // Mise à jour heure:minute:seconde
    Clock.setHour(gps.time.hour()+1); 
    Clock.setMinute(gps.time.minute());
    Clock.setSecond(gps.time.second());
    
    // Mise à jour jour/mois/année
    Clock.setDate(gps.date.day()); 
    Clock.setMonth(gps.date.month());
    Clock.setYear(gps.date.year()-2000); // Ne garde que les 2 derniers digits de l'année
    
    if(debug_mode) {Serial.print("RTC set from GPS at ");Serial.println(datetime_rtc);Serial.println();}
  }else{
    if(debug_mode) Serial.println("No GPS fix yet. Can't set RTC yet.\n");
  }  
}

void check_rtc_set()
{
  if(Clock.getHour(h12, PM) == 0 && Clock.getMinute() == 0 && Clock.getSecond() == 0 && Clock.getYear() < 2010)
  {
    rtc_set = false;
    Serial.println("RTC time not set\n");
  }else{
    rtc_set = true;
    Serial.println("RTC time set\n");
  }
}



void cycle_standard(){
  test_sd();                   // Initialise et teste la carte SD
  lecture_config();            // Lit le fichier de configuration
  refresh_config_values();     // Réactualise les valeurs du programme en fonctions de celles lues dans le fichier config 
  mesure_cycle_to_datachain(); // Lance un cycle de mesure et stocke les paramètres mesurés dans datachain 
  save_datachain_to_sd();      // Vient écrire le contenu de datachain dans le fichier dataFilename sur la carte SD
}

