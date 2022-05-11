  /*********************************************************************************************************
Controleur du bras robotique

Utilisation d'un Teensy 3.5

Auteur :
 **********************************************************************************************************/
 

#include "controleur.h"
#include "config_controleur.h"

#ifdef DEBUG // Debug MODE
/*********************** DEBUG ***************************************************************************/

Controleur controleur ;
float messageSendDebug[6];
float* messageReceiveDebug ; 

int J1 = 0; int J2 =1; int J3 = 2; int J4 = 3; int J5 = 4; int J6 = 5;
float targets[6] ={-10,-10,-10,0,360,-90};
float targets1[6] ={0,0,0,0,0,0};
//float targets[6] ={100,45,25,100,75,90};
float targetzero[6] ={0,0,0,0,0,0};

int countDebug = 0;

void setup()
{
  Serial.begin(BAUD_RATE);
  delay(1000);
  Serial.println("****************** DEBUG - START ****************** \n");
  
/**************  HOMING CYCLE - DEBUG ********************************************/
  controleur.setNewMaxSpeed(3);
  #ifdef MOVETOHOMEINSTEPS
    Serial.println("-------- moveToHomeInStepts -----------------------");
    controleur.moveToHomeInSteps(1);
    Serial.println("--------------------------------------------------\n");
  #endif //MOVETOHOMEINSTEPS
/*************** END HOMING CYCLE - DEBUG ***************************************/


/**************  CONVERT ANGLE - DEBUG ********************************************/


/**************  END CONVERT ANGLE - DEBUG ********************************************/
  
/**************  SPI - DEBUG ********************************************/
    #ifdef SPI
      controleur.getSPI().onTransfer(myCallback);
    #endif
/************** END SPI - DEBUG ********************************************/

/**************  UART - DEBUG ********************************************/
    #ifdef UART
      controleur.waitHandshake();
    #endif
/************** END SPI - DEBUG ********************************************/

}

/**************  SPI - DEBUG ********************************************/
#ifdef SPI
void myCallback(uint16_t *buffer, uint16_t length, AsyncMST info)                       //Inerrrput routine function 
{
  //controleur.interruptSPI(buffer, length, info);
}
  #endif
/************** END SPI - DEBUG ********************************************/

void loop() 
{
countDebug++;
    /************Les différents modes de fonctionnement selon les commandes SPI reçues *********************
     *                                                                                                     *                                                                                                                                                                              
     *                                                                                                     *
     *              SELECTION DES MODES - CHOIX DES VITESSES - GENERATION DE TRAJECTOIRES                  *
     *                                                                                                     *
     *                     Selection du style du mouvement - Relatif ou Absolue                                                                                    *
     *                                                                                                     *
     *            À LA FIN DE CETTE SECTION ON DOIT AVOIR UNE NOUVELLE POSITION À ATTEINDRE                *
     *                 OU UNE DEMANDE DE CHANGEMENT DE VITESSE OU DE MOTEUR À CONTRÔLER                    *
     *                                                                                                     *
     *                                                                                                     *
     * ************************************* FIN DE LA SECTION ********************************************/



/************Les différents modes de fonctionnement selon les commandes SPI reçues *********************
*
*
* Une fois les nouvelles positions reçues à partir des cinématiques inverse et/ou (directe)
* Il faut vérifier que ces positions sont dans les limites autorisées du mouvement avec
*
*
*/
/**************  SAFETY CHECK - DEBUG ********************************************/
  #ifdef SAFETY_ANGLE
    bool checkAngleLimits(float* angle);
/* Si les angles sont dans l'espace de travail du bras on set l'attribut _nextAnglePos de l'objet Controleur*/
  #endif
/************** SAFETY CHECK - DEBUG ********************************************/

/***************** UART - DEBUG *******************************************************/
#ifdef UART_DEBUG
 messageReceiveDebug = controleur.getMessageRecu();
  Serial.print("Message venant de la manette : ");
  for(int i = 0 ; i < NOMBRE_DE_MESSAGE_RECU; i++){
      Serial.print(messageReceiveDebug[i]);
      Serial.print(" ");
      }
  Serial.println(" ");
  delay(100);

  for(int i=0; i<NOMBRE_DE_MESSAGE_ENOVYER; i++){
    messageSendDebug[i] = countDebug;
  }
  controleur.setMessageToSend(messageSendDebug);
#endif
/*if(controleur.checkAngleLimits(targets)==true)
{
  /* Si les angles sont dans l'espace de travail du bras on set 
  ** l'attribut _nextAnglePos de l'objet Controleur 
  controleur.setNextPos(targets); 
  control.moveAsync();

/***************** END UART - DEBUG ***************************************************/

/***************** UART - REAL-TIME***************************************************/
#ifdef UART
controleur.formulateMessage();
   messageReceiveDebug = controleur.getMessageRecu();
  Serial.print("Message venant de la manette : ");
  for(int i = 0 ; i < NOMBRE_DE_MESSAGE_RECU; i++){
      Serial.print(messageReceiveDebug[i]);
      Serial.print(" ");
      }
      Serial.println(" ");
  
#endif

  /* On peut alors réaliser le mouvement des moteurs 
  ** vers leurs cibles 

 controleur.moveAsync();

//////////////////////////////////////////////////////////////////////////////////////////////////
  /* On devrait être capable de vérifier s'il y a des erreurs  
  ** pendant le mouvement avec l'état des alarmes OC des drivers */

  /* Si pour un axe, il y a une alarme, voici les étapes à suivre:  
  ** - Disable et Enable le driver du moteur en faute 
  ** - Homing cycle pour ce seul moteur
  ** - Relancer le mouvement vers la cible non atteinte en mouvement absolue
  ** - S'il s'agit d'un problème de collision, recalculer la cible avec la cin indirecte
//////////////////////////////////////////////////////////////////////////////////////////////////



}
else
{
  Serial.println("!!Danger!! - Vérifiez les angles désirés");
}
*/


}
/*********************** END DEBUG ***************************************************************************/

}
#else // Controleur MODE

#endif
