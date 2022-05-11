/*********************************************************************************************************
Manette pour bras robotique

Utilisation d'un Teensy 3.5

Auteurs: David-Olivier Roy et Cyrille Tabe.
 **********************************************************************************************************/
#include "manette.h"

/*********************** DEBUG ***************************************************************************/
#ifdef DEBUG
Manette manette(DEBUG);

Matrix<6> articulationsDebug;
Matrix<6> positionsDebug;
Matrix<12> stateDebug;

float messageSendDebug[7];
float *messageReceiveDebug;
int idPacketDebug = 0;

int countDebug = 0;

void setup (void)
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("****************** DEBUG - START ****************** \n");

/************** CINÉMATIQUE DIRECTE - DEBUG ********************************************/
#ifdef DIRECT  
  Serial.println("CINEMATIQUE DIRECTE - DEBUG");
  Serial.println("-------- ARTICULATIONS REPOS-----------------------");
  for(int i = 0; i < 6; i++)
    articulationsDebug(i) = 0;
  manette.setArticulations(articulationsDebug);
  manette.cinematiqueDirect();
  Serial.println("--------------------------------------------------\n");
  
  Serial.println("--------- ARTICULATION IMPOSSIBLE ---------------------------");
  Serial.println("--------- Articulation 4 limite ---------------------------\n");
  manette.setLimitAngle(4, -1, 1);
  
  positionsDebug(0) = 756.44;
  positionsDebug(1) = 0.00;
  positionsDebug(2) = 151.27;
  positionsDebug(3) = -M_PI;
  positionsDebug(4) = 1.57;
  positionsDebug(5) = 1.57;
    
  manette.setPositions(positionsDebug);
  manette.cinematiqueInverse();
  
  manette.setLimitAngle(4, -3.14, 3.14);
  
  Serial.println("-------------------------------------------------\n");
  
  manette.setArticulations(articulationsDebug);
  manette.cinematiqueDirect();
  Serial.println("-------------------------------------------------\n");


  for(int i = 0; i < 6; i++){
    Serial.print("-------- ARTICULATION #");
    Serial.print(i+1);
    Serial.println("--------------------------");
    for(int j = 0; j < 6; j++)
      articulationsDebug(j) = 0;
    
    articulationsDebug(i) = PI/2;
    manette.setArticulations(articulationsDebug);

    manette.cinematiqueDirect();
    Serial.println("--------------------------------------------------\n");
  }
  Serial.println("");
#endif
/*************** END CINÉMATIQUE DIRECTE - DEBUG ***************************************/


/*************** CINÉMATIQUE INVERSE - DEBUG *******************************************/
#ifdef INVERSE
  Serial.println("CINEMATIQUE INVERSE - DEBUG");
  Serial.println("-------- POSITIONS REPOS ---------------------------");
  positionsDebug(0) = 756.44;
  positionsDebug(1) = 0.00;
  positionsDebug(2) = 151.27;
  positionsDebug(3) = -M_PI;
  positionsDebug(4) = 1.57;
  positionsDebug(5) = 0.00;
    
  manette.setPositions(positionsDebug);

  manette.cinematiqueInverse();
  Serial.println("---------------------------------------------------\n");
/*
  Serial.println("--------- POSITION IMPOSSIBLE ---------------------------");
  positionsDebug(0) = 1000;
  positionsDebug(1) = 1000;
  positionsDebug(2) = 1000;
  positionsDebug(3) = 0.00;
  positionsDebug(4) = 0.00;
  positionsDebug(5) = 0.00;
    
  manette.setPositions(positionsDebug);
  manette.cinematiqueInverse();
  Serial.println("-------------------------------------------------\n");
*/

  Serial.println("--------- POSITION #1 -----------------------------");
  positionsDebug(0) = 0.00;
  positionsDebug(1) = -756.44;
  positionsDebug(2) = 151.27;
  positionsDebug(3) = M_PI/2;
  positionsDebug(4) = 0.00;
  positionsDebug(5) = M_PI/2;
    
  manette.setPositions(positionsDebug);
  manette.cinematiqueInverse();
  Serial.println("---------------------------------------------------\n");

  Serial.println("--------- POSITION #2 -----------------------------");
  positionsDebug(0) = 64.20;
  positionsDebug(1) = 0.00;
  positionsDebug(2) = -540.97;
  positionsDebug(3) = M_PI;
  positionsDebug(4) = 0.00;
  positionsDebug(5) = 0.00;
    
  manette.setPositions(positionsDebug);
  manette.cinematiqueInverse();
  Serial.println("---------------------------------------------------\n");

  Serial.println("--------- POSITION #3 -----------------------------");
  positionsDebug(0) = 314.20;
  positionsDebug(1) = 0.00;
  positionsDebug(2) = -290.97;
  positionsDebug(3) = M_PI;
  positionsDebug(4) = 0.00;
  positionsDebug(5) = 0.00;
    
  manette.setPositions(positionsDebug);
  manette.cinematiqueInverse();
  Serial.println("-------------------------------------------------\n");

  Serial.println("--------- POSITION #4 ---------------------------");
  positionsDebug(0) = 756.44;
  positionsDebug(1) = 0.00;
  positionsDebug(2) = 151.27;
  positionsDebug(3) = -M_PI/2;
  positionsDebug(4) = M_PI/2;
  positionsDebug(5) = 0.00;
    
  manette.setPositions(positionsDebug);
  manette.cinematiqueInverse();
  Serial.println("-------------------------------------------------\n");

  Serial.println("--------- POSITION #5 ---------------------------");
  positionsDebug(0) = 535.54;
  positionsDebug(1) = 0.00;
  positionsDebug(2) = -69.63;
  positionsDebug(3) = M_PI;
  positionsDebug(4) = 0.00;
  positionsDebug(5) = 0.00;
    
  manette.setPositions(positionsDebug);
  manette.cinematiqueInverse();
  Serial.println("-------------------------------------------------\n");

  Serial.println("--------- POSITION #6 ---------------------------");
  positionsDebug(0) = 756.44;
  positionsDebug(1) = 0.00;
  positionsDebug(2) = 151.27;
  positionsDebug(3) = -M_PI/2;
  positionsDebug(4) = M_PI/2;
  positionsDebug(5) = 0.00;
    
  manette.setPositions(positionsDebug);
  manette.cinematiqueInverse();
  Serial.println("-------------------------------------------------\n");
  
  for(int i = 0; i < 6; i++)
    articulationsDebug(i) = 0;

  positionsDebug(0) = 756.44;
  positionsDebug(1) = 0.00;
  positionsDebug(2) = 151.27;
  positionsDebug(3) = -M_PI;
  positionsDebug(4) = 1.57;
  positionsDebug(5) = 0.00;
    
  manette.setArticulations(articulationsDebug);
  manette.setPositions(positionsDebug);

#endif 
/***************** END CINÉMATIQUE INVERSE - DEBUG ***********************************/
/***************** UART-- SEND SHAKE--- ***********************************/
  Serial.println("----------- UART - SEND SHAKE -------------------");
  manette.sendHandshake();
  Serial.println("-------------------------------------------------\n");

}

#ifdef SPI
void myCallback(uint16_t *buffer, uint16_t length, AsyncMST info)         //Inerrrput routine function 
{
  manette.interruptSPI(buffer, length, info);
}
#endif


void loop(void)
{
countDebug++;

/***************** MANETTE - DEBUG ****************************************************/
#ifdef MANETTE  
  Serial.println("        MANETTE - DEBUG"); 
  Serial.print("-------- ITTERATION #");
  Serial.print(countDebug);
  Serial.println("--------------------------");   
  manette.readInput();
  manette.js2Move();
  manette.GPIOManette::printInput();  
  manette.printInput();
  Serial.println("-------------------------------------------------\n");
#endif
/***************** END MANETTE - DEBUG ***********************************************/

/***************** SPI - DEBUG *******************************************************/
#ifdef SPI  
  idPacketDebug++;
    
  for(int i=0; i<6; i++){
    messageSendDebug[i] = 2*i;
  }
    
  manette.sendSPI(messageSendDebug,sizeof(messageSendDebug),idPacketDebug);
  manette.listenSPI();
#endif
/***************** END SPI - DEBUG ***************************************************/

/***************** UART - DEBUG *******************************************************/
#ifdef UART_DEBUG
  messageReceiveDebug = manette.getMessageRecu();
/*
  Serial.print("Message venant du controleur : ");
  for(int i = 0 ; i < NOMBRE_DE_MESSAGE_RECU; i++){
      Serial.print(messageReceiveDebug[i]);
      Serial.print(" ");
      }
  Serial.println(" ");
*/
    Serial.print("Message envoyé par Manette: ");
  for(int i=0; i<NOMBRE_DE_MESSAGE_ENOVYER; i++){
    messageSendDebug[i] = i*countDebug;
    Serial.print(messageSendDebug[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
  delay(100);


  manette.setMessageToSend(messageSendDebug);
#endif
/***************** END UART - DEBUG ***************************************************/

/*****************  UART - REAL-TIME ***************************************************/
#ifdef UART
  messageReceiveDebug = manette.getMessageRecu();
  manette.formulateMessage();
    Serial.print("Message venant du controleur : ");
  for(int i = 0 ; i < NOMBRE_DE_MESSAGE_RECU; i++){
      Serial.print(messageReceiveDebug[i]);
      Serial.print(" ");
      }
  Serial.println(" ");

  delay(100);
#endif
/*****************  END UART - REAL-TIME ***************************************************/
delay(100);
}
/************************ END DEBUG ****************************************************************************/

#else
/*********************** MAIN **********************************************************************************/
Manette manette;

void setup (void)
{
  Serial.begin(115200);
  manette.getSPI().onTransfer(myCallback);
}

#ifdef SPI
void myCallback(uint16_t *buffer, uint16_t length, AsyncMST info)         //Inerrrput routine function 
{
  manette.interruptSPI(buffer, length, info);
}
#endif

void loop(void)
{
  manette.readInput();
  manette.js2Move();
  manette.printInput();
#ifdef SPI
  manette.listenSPI();
#elif

#endif
}

/************************ END MAIN ***************************************************************************/
#endif
