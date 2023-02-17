#include "fonctions.h"

extern TSYS01 sensor_fastTemp; // Déclaration externe du capteur de température BlueRobotics

/* --------------- Déclarations du Séquenceur -----------------------*/
void step1();  // Déclaration des fonctions steps utilisées dans le séquenceur
void step2();

// Appelle les steps dans l'ordre avec un délai entre chaque 
Sequencer2 Seq(&step1, DELAY,  // Délai défini dans le header | 1000 for read cmd 
               &step2, 0);     

//const int led = D9;
//unsigned compteur = 0;
//const int greenled = 25;  // on utilise la Led interne à la board esp32

void setup() {
  Serial.begin(115200);   // Pour le moniteur série
  Wire.begin();           // Pour la communication I2C
  sensor_fastTemp.init(); // Init du capteur de température
  init_gps();             // Init communication UART du GPS
  Seq.reset();            // Init du séquenceur
}

void loop() {     
  Seq.run();  // Lancement du séquenceur
}

/*------------------- Etapes du séquenceur ---------------------------*/
void step1(){
  enable_ec_parameters(EC_ENABLED, TDS_ENABLED, SAL_ENABLED, SG_ENABLED); 
  mesureEC();
  mesure_temp();
  scanning_gps();
  
}
void step2(){
  Serial.println();
}



