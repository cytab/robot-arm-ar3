#include "manette.h"
#include "config_manette.h"

float target[7] = {10, 11,12,13,14,15,1};
Manette manette;
float* message;
void setup() {
  // put your setup code here, to run once:
  manette.sendHandshake();
  Serial.print("Reussi");
  Serial.println(" ");
}

void loop() {
  message = manette.getMessageRecu();
  // put your main code here, to run repeatedly:
  Serial.print("Message venant du controleur : ");
  for(int i = 0 ; i < NOMBRE_DE_MESSAGE_RECU; i++){
                  
      Serial.print(message[i]);
  }
  Serial.println("");
  delay(100);
}
