#include "gpioControleur.h"

/*
* Constructeur de GPIOcontroleur
* return void
*/
GPIOControleur::GPIOControleur()
{   
#ifndef UART
   _spi("SLAVE", "STANDALONE")
   _spi.begin();
#else
    /*********** Serial1 ***************/
    Serial1.setRX(RX);
    Serial1.setTX(TX);
  
    Serial1.begin(BAUD_RATE);
    Serial.begin(BAUD_RATE);
    _sync_mode = true;
    _lost_while_running= false;

#endif
    
    for (int i = 0 ; i < NB_MOTEUR ; i++){
        _pMoteurs[i] =  new Stepper(JSTEP_PIN[i], JDIR_PIN[i]);
        pinMode(JLIM_PIN[i], INPUT_PULLUP);
    }
    checkLimitSwitch();
}


GPIOControleur::~GPIOControleur(){
    for (int i =0; i < NB_MOTEUR;i++){
        delete _pMoteurs[i];
    }
    if (threads.getState(_threadId1) == Threads::RUNNING)  {
        threads.kill(_threadId1);
    }
    if (threads.getState(_threadId2) == Threads::RUNNING)  {
        threads.kill(_threadId2);
    }

}

#ifdef SPI
    // float test[6];
    // gpio.setSendSPI(test,sizeof(test));
    void GPIOControleur::setSendSPI(float *send, size_t size){
        if(size/sizeof(send[0]) == SPI_SIZE_CONTROLEUR){
            for(int i = 0; i < SPI_SIZE_CONTROLEUR; i++){
                _spiSend[i] = send[i];
            }
        }

        else{
            Serial.println("ERROR");
            Serial.print("Setting Sending SPI : SPI_SIZE_CONTROLEUR != ");
            Serial.println(size/sizeof(send[0]));
        }

    
    }


    SPI_MSTransfer GPIOControleur::getSPI(){
        return _spi;
    }

    volatile float* GPIOControleur::getReceive(){
        return _spiReceive;
    }

    void GPIOControleur::sendSPI(int packetID){
        _spi.transfer16((uint16_t *) _spiSend, sizeof(_spiSend), packetID, 1);
    }

    // gpio.sendSPI(test,sizeof(test),69);
    void GPIOControleur::sendSPI(float *send, size_t size, int packetID){
        if(size/sizeof(send[0]) == SPI_SIZE_CONTROLEUR){
            for(int i = 0; i < SPI_SIZE_CONTROLEUR; i++)
                _spiSend[i] = send[i];
                
            _spi.transfer16((uint16_t *) _spiSend, size, packetID, 1);
        }

        else{
            Serial.println("ERROR");
            Serial.print("Sending SPI : SPI_SIZE_CONTROLEUR != ");
            Serial.println(size/sizeof(send[0]));
        }
    }

    void GPIOControleur::listenSPI(){
        _spi.events();
    }

    void GPIOControleur::interruptSPI(uint16_t *buffer, uint16_t length, AsyncMST info){
        if (length/sizeof(buffer) == SPI_SIZE_MANETTE){
            Serial.print("PacketID: "); Serial.println(info.packetID);
            Serial.print("Length: "); Serial.println(length);
            for ( uint16_t i = 0; i < length/sizeof(buffer); i++ ) {
                float *temp = (float *)buffer;
                _spiReceive[i]= temp[i];
                Serial.print(_spiReceive[i]); Serial.print(" ");
            }
            Serial.println();
        }
        else{
            Serial.println("ERROR");
            Serial.print("Reception SPI : SPI_SIZE_MANETTE != ");
            Serial.println(length/sizeof(buffer));
        }
    }
#else

/*
* Get le id du thread specifier
* whichThread : choix en premier thread (sending thread) ou 
* deuxieme (recieving thread)
*/
int GPIOControleur::getThreadId(int whichThread){
    if (whichThread == 1) {
        return _threadId1;
    }else if(whichThread == 2){
        return _threadId2;
    }
}

/*
* Get la variable contenant le message à envoyer
*/
float* GPIOControleur::getMessageToSend(){
    return _messageToSend ;
}

/*
* Get la variable contenant le message à recu
*/
float* GPIOControleur::getMessageRecu(){
    return _messages ;
}

/*
* lorsque la connexion est perdue pendant que le thread est entrian  
* de rouler ce flag est activé
*/
void GPIOControleur::setLost(bool state){
   _lost_while_running = state;
}


/*
* Set la variable sync_mode qui montre si le système est mode sybnchronisation
* pour l'instant on sait pas si c'est encore utile
* state : state de le synchronisation 
* true si en mode sync sinon false 
*/
void GPIOControleur::setSyncMode(bool state){
   _sync_mode = state;
}

/*
* Set la variable sync_mode qui montre si le système est mode sybnchronisation
* pour l'instant on sait pas si c'est encore utile
* value : value avec laquelle remplacer le message à envoyer
* value est de la forme : {ID, DATA(_)}
*/
void GPIOControleur::setMessageToSend(float* value){
    for(int i = 0 ; i < NOMBRE_DE_MESSAGE_ENOVYER; i++){
        _messageToSend[i] = value[i];
    }
}

/*
* Check if message received is corrupted
*/
bool GPIOControleur::isCorrupted(float valeur){
  bool state = false;
  if (std::isnan(valeur)) state= true;
  if (std::isinf(valeur)) state= true;
  if (valeur > 1000) state= true;  // constant determined empirically
  if (valeur < -1000) state= true;
  // add if possible limit angle !!!!!!!!!!!!!!!!!!
  return state;
}

/*
* retourn l'état de synchronisation
*/
bool GPIOControleur::isSyncMode(){
    return _sync_mode;
}
/*
* return if connexion is lost
*/
bool GPIOControleur::isLost(){
    return _lost_while_running;
}

/* 
* Send a message containing : 
* manette :{ID :packetID , Angles : [a1,a2,a3,a4,a5,a6], Servo : Open/Close}
* controleur : acknowledge message 
* Paramètre :
* arg : objet GPIOControleur actuel contenant l<Adresse de toutes
* les variaables 
* returnn void 
*/
void GPIOControleur::sendUART(void *arg){

    GPIOControleur *thisobject = static_cast<GPIOControleur*> (arg);

    int id2 = thisobject->getThreadId(THREAD2);
    float* messaget = thisobject->getMessageToSend();

    while(1){
        threads.wait(id2, PERIOD_OF_THREAD);
        if (!thisobject->isSyncMode()){
            // verrifier la position acatuelle !!!!
            // envoyer letat des switch aussi 
            Serial1.write(SYNC_BIT);
            for (int i = 0; i < NOMBRE_DE_MESSAGE_ENOVYER ; i++){
                byte *b = (byte *) &messaget[i];
                Serial1.write(b, sizeof(float));
            }
        }
    }
}

/*
* Receive a message 
* args: instance de l<objet ce qui permettra au thread de changer 
* directement aux bonnes adresses
*return void 
*/
void GPIOControleur::receiveUART(void *arg){
    GPIOControleur *thisobject = static_cast<GPIOControleur*> (arg);
    int id1 = thisobject->getThreadId(THREAD1);
    float* messager = thisobject->getMessageRecu();
    byte rec[BUFFER_SIZE];
    float count = 0;
    while(1){
        threads.wait(id1, PERIOD_OF_THREAD); 
        Serial.println("Debug purpose");
        count = 0;
             Serial.print("supposed to work  : ");
            bool corrupted = false;
            if (Serial1.available() && !thisobject->isSyncMode() && !thisobject->isLost()) {
                byte c = Serial1.read();

                Serial.print("Sync bit : ");
                Serial.println(c);
                 
                while(c != ID_RECU && c != RECEPTION_HANDSHAKE_BIT &&   !thisobject->isLost()){
                    count++;
                    if (Serial1.available())
                        c = Serial1.read();
                    if (count == TICK_WAIT_FOR_AVAILIBILY){
                        Serial.println("Lost now : ");
                        thisobject->setLost(true);}
                    }
                if(c == RECEPTION_HANDSHAKE_BIT){thisobject->setLost(true);}
                if (c == ID_RECU){
                    for(int i = 0 ; i< NOMBRE_DE_MESSAGE_RECU;i++){
                        for (int j = 0; j < sizeof(float); j++){
                    
                        while(!Serial1.available()){}
                        rec[j] = Serial1.read();
                    }
                    // DO SANITY CHECK HERE
                    float sample = *(float*)(rec);
                    corrupted = thisobject->isCorrupted(sample) || corrupted ;
                    if (!corrupted && c== ID_RECU){
                        messager[i] = sample;
                    }else{
                        
                    }
                    
                }
                }
            }
            else{
                if(thisobject->isSyncMode() || thisobject->isLost()){
                    thisobject->setLost(true);
                    thisobject->setSyncMode(true);
                    
                    Serial.println("Resynchronixing ....");
                    thisobject->waitHandshake();      

                    thisobject->setLost(false);
                    thisobject->setSyncMode(false);
            }
            }
        }
        
    
}

/*
* Protocole handshake qui va s'occuper d'attendre en boucle 
* un certain bit de handshake , lui permettant de répondre 
* avec un bit ok et de recevoir le bit de creation de thread et de créer les thread
*/
void GPIOControleur::waitHandshake(){
        byte first_shake = 0 ;
        byte comfirmation_shake = 0 ;
        Serial.println("Start");
        Serial.print(" ");
        for (int i = 0; i < NOMBRE_ESSAIE; i++){
            Serial1.write(IM_LOST);
            delay(5);
        }

        while (first_shake != RECEPTION_HANDSHAKE_BIT){ 
            if (Serial1.available())
                first_shake = Serial1.read();
        }

        Serial.println("j'ai recu ton shake");
        Serial.print(" ");
        while(comfirmation_shake != COMFIRMATION_SHAKE){
            Serial1.write(CONNECTION_DONE);
            if (Serial1.available())
                comfirmation_shake = Serial1.read();
        }
        setSyncMode(false);
        Serial.println("je lance mon thread");
        Serial.print(" ");
        if (!_lost_while_running){
        _threadId1 = threads.addThread(receiveUART, this);
        }else{
           
        }
        if (!_lost_while_running){
            _threadId2 = threads.addThread(sendUART, this);
        }else{
          
        }
}




#endif

/*
* return void
*/
bool GPIOControleur::getLimit(int n){
   return _limits[n];
}


/*
* Vérfie la valeur des limit switch et met à jour _limit
* return: void
*/
void GPIOControleur::checkLimitSwitch(){
    for (int i=0;i<NB_MOTEUR;i++) {
        _limits[i] = digitalRead(JLIM_PIN[i]);
    }
}

/*
* set la position actuel du compteur des moteurs
* arg: int32_t position , est un pointeur vers un tableau 
* de taille 6 (exp: setpositon initiale des moteurs à 0 ) 
* return: void
*/
void GPIOControleur::setPosition(int32_t *position){
    for(int i = 0; i<NB_MOTEUR; i++){
         _pMoteurs[i]->setPosition(position[i]);
 
    }
}

/*
* set la nouvelle cible de positon des moteurs 
* arg: int32_t target , est un pointeur vers un tableau 
* de taille 6 (exp: setarget des moteurs à 1000 ) 
* arg2: bool absolute : if true use setTargetabsolute else use setTargetRel
* return: void
*/
void GPIOControleur::setTarget(int32_t *target, bool absolute){
    for(int i = 0; i<NB_MOTEUR; i++){
        if (absolute)
            _pMoteurs[i]->setTargetAbs(target[i]);
        else
            _pMoteurs[i]->setTargetRel(target[i]);  
    }
}


/**
 * @brief get la position actuel des moteurs
 * 
 * @param nMoteur[in] le numéro du moteur dont on voudrait connaitre la position 
 * @return int32_t 
 */
int32_t GPIOControleur::getPosition(int nMoteur){
    return _pMoteurs[nMoteur]->getPosition();
}