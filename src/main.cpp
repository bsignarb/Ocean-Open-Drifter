#include <Arduino.h>
#include "fonctions.h"

const int led = D9;
unsigned compteur = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("TEST");

  pinMode(led, OUTPUT);
}

void loop() {
  //Serial.println("Boucle" + String(compteur));
  digitalWrite(led, LOW);
  delay(1000);
  digitalWrite(led, HIGH);
  delay(3000);
  compteur++;

  float v = get_voltage(); 
  Serial.println("Vbat : " + String(v));
}