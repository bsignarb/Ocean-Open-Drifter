#include "fonctions.h"

#define PROBE_TYPE 1.0 

/* Séquenceur
//forward declarations of functions to use them in the sequencer before defining them
void step1();  
void step2();
void step3();

Sequencer3 Seq(&step1, 350, // calls the steps in sequence with time in between them
               &step2, 0,   // 1000 for read cmd
               &step3, 350);  
*/

//const int led = D9;
//unsigned compteur = 0;

//déclaration pour gestion des Led et interrupteur
//const int greenled = 25;  // on utilise la Led interne à la board esp32

void setup() {
  Serial.begin(115200);
  Wire.begin();  // for I2C communication
  //Seq.reset(); // init du séquenceur
}

void loop() {
  //Seq.run();  // Lancement du séquenceur
  
  //setting_ec_probe_type(PROBE_TYPE);
  enable_ec_parameters(0, 0, 1, 0); // (ec, tds, sal, sg)
  mesureEC();
}

/* Etapes du séquenceur
void step1(){
  //send a read command. we use this command instead of PH.send_cmd("R"); 
  //to let the library know to parse the reading                   
  EC.send_cmd("L,?");

}
void step2(){
  //EC.send_read_cmd();
  
}
void step3(){
  receive_and_print_response(EC);
  //receive_and_print_reading(EC);             //get the reading from the EC circuit
  Serial.println();
}
*/

