#include "controleur.h"
#include "config_controleur.h"

float target[7] = {10, 11,12,13,14,15,1};
Controleur control;
float* message ; 
void setup() {
  // put your setup code here, to run once:
  control.waitHandshake();
  Serial.print("Reussi");
  Serial.println(" ");

}

void loop() {
  control.checkLimitSwitch();
  // put your main code here, to run repeatedly:
  message = control.getMessageRecu();
  // put your main code here, to run repeatedly:
  Serial.print("Message venant de la manette : ");
      for(int i = 0 ; i < NOMBRE_DE_MESSAGE_RECU; i++){
          Serial.print(message[i]);
          Serial.print(" ");
      }
      control.formulateMessage();
      Serial.println("\n-------------------\n");
      Serial.print("LimiteSwitch : ");
        for(int i = 0 ; i < 6; i++){
          Serial.print(control.getLimit(i));
          Serial.print(" ");
        }
      Serial.println("\n-------------------\n");
      delay(100);
      

      
}
