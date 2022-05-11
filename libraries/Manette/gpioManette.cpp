#include "gpioManette.h"

GPIOManette::GPIOManette() : 
  _lcd(LCD_CS, LCD_DC, LCD_RST, LCD_MOSI, LCD_SCLK, LCD_MISO),
  _spi("_lcd",SPI_CS , &SPI_BUS)
{
  // Mode
  pinMode(SWITCH_MODE1, INPUT_PULLUP);
  pinMode(SWITCH_MODE2, INPUT_PULLUP);

  // Bouton Gauche
  pinMode(BOUTON_G1, INPUT_PULLUP);
  pinMode(BOUTON_G2, INPUT_PULLUP);

  // Bouton Droit
  pinMode(BOUTON_D1, INPUT_PULLUP);
  pinMode(BOUTON_D2, INPUT_PULLUP);
  pinMode(BOUTON_D3, INPUT_PULLUP);
  pinMode(BOUTON_D4, INPUT_PULLUP);

  // JoyStick Gauche
  pinMode(JS_VXG, INPUT);
  pinMode(JS_VYG, INPUT);
  pinMode(JS_BOUTG, INPUT_PULLUP);

  // JoyStick Droit
  pinMode(JS_VXD, INPUT);
  pinMode(JS_VYD, INPUT);
  pinMode(JS_BOUTD, INPUT_PULLUP);

  // LCD
  _lcd.begin();
  _lcd.fillScreen(ILI9341_BLACK);
  _lcd.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  _lcd.setRotation(1);
  _lcd.setTextSize(2);
  _lcd.setCursor(0, 0);
  _lcd.setClock(60000000); // Jusqu'à 120MHz

  // SPI
#ifndef UART
  SPI_BUS.setMOSI(SPI_MOSI);
  SPI_BUS.setMISO(SPI_MISO);
  SPI_BUS.setSCK(SPI_SCK);
  SPI_BUS.begin();
#else
    /*********** Serial1 ***************/
    Serial1.setRX(RX);
    Serial1.setTX(TX);
  
    Serial1.begin(BAUD_RATE);
    Serial.begin(BAUD_RATE);
    _sync_mode = true ;
    _lost_while_running = false;
#endif

  _debug_GPIO = false;
  _grip = false ; 
}

GPIOManette::GPIOManette(bool debug) : 
  _lcd(LCD_CS, LCD_DC, LCD_RST, LCD_MOSI, LCD_SCLK, LCD_MISO),
  _spi("_lcd",SPI_CS , &SPI_BUS)
{
  // Mode
  pinMode(SWITCH_MODE1, INPUT_PULLUP);
  pinMode(SWITCH_MODE2, INPUT_PULLUP);

  // Bouton Gauche
  pinMode(BOUTON_G1, INPUT_PULLUP);
  pinMode(BOUTON_G2, INPUT_PULLUP);

  // Bouton Droit
  pinMode(BOUTON_D1, INPUT_PULLUP);
  pinMode(BOUTON_D2, INPUT_PULLUP);
  pinMode(BOUTON_D3, INPUT_PULLUP);
  pinMode(BOUTON_D4, INPUT_PULLUP);

  // JoyStick Gauche
  pinMode(JS_VXG, INPUT);
  pinMode(JS_VYG, INPUT);
  pinMode(JS_BOUTG, INPUT_PULLUP);

  // JoyStick Droit
  pinMode(JS_VXD, INPUT);
  pinMode(JS_VYD, INPUT);
  pinMode(JS_BOUTD, INPUT_PULLUP);

  // LCD
  _lcd.begin();
  _lcd.fillScreen(ILI9341_BLACK);
  _lcd.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  _lcd.setRotation(1);
  _lcd.setTextSize(2);
  _lcd.setCursor(0, 0);
  _lcd.setClock(60000000); // Jusqu'à 120MHz

  // SPI
#ifndef UART
  SPI_BUS.setMOSI(SPI_MOSI);
  SPI_BUS.setMISO(SPI_MISO);
  SPI_BUS.setSCK(SPI_SCK);
  SPI_BUS.begin();
#else
  Serial1.setRX(RX);
  Serial1.setTX(TX);

  Serial1.begin(BAUD_RATE);
  Serial.begin(BAUD_RATE);
  _sync_mode = true ;
  _lost_while_running = false;
#endif

  _debug_GPIO = debug;
  _grip = false;
}

GPIOManette::~GPIOManette(){
  if (threads.getState(_threadId1) == Threads::RUNNING)  {
        threads.kill(_threadId1);
    }
    if (threads.getState(_threadId2) == Threads::RUNNING)  {
        threads.kill(_threadId2);
    }
}

int GPIOManette::getInput(int numPin){

  switch(numPin) {
    case SWITCH_MODE1 :
      return _switch_Mode1;
      break;

    case SWITCH_MODE2 :
      return _switch_Mode2;
      break;
    
    case BOUTON_G1 :
      return _bouton_G1;
      break;
    
    case BOUTON_G2 :
      return _bouton_G2;
      break;
        
    case BOUTON_D1 :
      return _bouton_D1;
      break;
        
    case BOUTON_D2 :
      return _bouton_D2;
      break;
        
    case BOUTON_D3 :
      return _bouton_D3;
      break;
        
    case BOUTON_D4 :
      return _bouton_D4;
      break;
        
    case JS_VXG :
      return _js_Vxg;
      break;

    case JS_VYG :
      return _js_Vyg;
      break;

    case JS_BOUTG :
      return _js_Boutg;
      break;
    
    case JS_VXD :
      return _js_Vxd;
      break;
      
    case JS_VYD :
      return _js_Vyd;
      break;
        
    case JS_BOUTD :
      return _js_Boutd;
      break;

    default :
      return -1;      
  }
}

void GPIOManette::readInput(){
  
  _switch_Mode1 = digitalRead(SWITCH_MODE1);
  _switch_Mode2 = digitalRead(SWITCH_MODE2);

  _bouton_G1 = digitalRead(BOUTON_G1);
  _bouton_G2 = digitalRead(BOUTON_G2);

  _bouton_D1 = digitalRead(BOUTON_D1);
  _bouton_D2 = digitalRead(BOUTON_D2);
  _bouton_D3 = digitalRead(BOUTON_D3);
  _bouton_D4 = digitalRead(BOUTON_D4);

  _js_Vxg = analogRead(JS_VXG);
  _js_Vyg = analogRead(JS_VYG);
  _js_Boutg = digitalRead(JS_BOUTG);

  _js_Vxd = analogRead(JS_VXD);
  _js_Vyd = analogRead(JS_VYD);
  _js_Boutd = digitalRead(JS_BOUTD);
}

void GPIOManette::printInput(){
  if (_debug_GPIO){
    Serial.print("Bouton : ");
    Serial.print(_switch_Mode1);
    Serial.print(_switch_Mode2);
    Serial.print(" ");
    Serial.print(_bouton_G1);
    Serial.print(_bouton_G2);
    Serial.print(" ");
    Serial.print(_bouton_D1);
    Serial.print(_bouton_D2);
    Serial.print(_bouton_D3);
    Serial.println(_bouton_D4);

    Serial.print("JS G : ");
    Serial.print(_js_Vxg);
    Serial.print(" ");
    Serial.print(_js_Vyg);
    Serial.print(" ");
    Serial.println(_js_Boutg);

    Serial.print("JS D : ");  
    Serial.print(_js_Vxd);
    Serial.print(" ");
    Serial.print(_js_Vyd);
    Serial.print(" ");
    Serial.print(_js_Boutd);
    Serial.println("\n");
  }

  _lcd.setCursor(0, 0);

  _lcd.print("Bouton : ");
  _lcd.print(_switch_Mode1);
  _lcd.print(_switch_Mode2);
  _lcd.print(" ");
  _lcd.print(_bouton_G1);
  _lcd.print(_bouton_G2);
  _lcd.print(" ");
  _lcd.print(_bouton_D1);
  _lcd.print(_bouton_D2);
  _lcd.print(_bouton_D3);
  _lcd.print(_bouton_D4);
  _lcd.print("\n");

  _lcd.print("JS G : ");
  _lcd.print(_js_Vxg);
  _lcd.print(" ");
  _lcd.print(_js_Vyg);
  _lcd.print(" ");
  _lcd.print(_js_Boutg);
  _lcd.print("      \n");

  _lcd.print("JS D : ");  
  _lcd.print(_js_Vxd);
  _lcd.print(" ");
  _lcd.print(_js_Vyd);
  _lcd.print(" ");
  _lcd.print(_js_Boutd);
  _lcd.print("       \n\n");

}
#ifndef UART

  void GPIOManette::setSendSPI(float *send, size_t size){
      if(size/sizeof(send[0]) == SPI_SIZE_MANETTE){
          for(int i = 0; i < SPI_SIZE_MANETTE; i++){
            _spiSend[i] = send[i];
          }
      }

      else{
        _lcd.println("ERROR");
        _lcd.print("Setting Sending SPI : SPI_SIZE_MANETTE != ");
        _lcd.println(size/sizeof(send[0]));
      }
  }

  SPI_MSTransfer GPIOManette::getSPI(){
      return _spi;
  }

  volatile float* GPIOManette::getReceive(){
      return _spiReceive;
  }

  void GPIOManette::sendSPI(int packetID){
      _spi.transfer16((uint16_t *) _spiSend, sizeof(_spiSend), packetID, 1);
  }

  void GPIOManette::sendSPI(float *send, size_t size, int packetID){
      if(size/sizeof(send[0]) == SPI_SIZE_MANETTE){
          for(int i = 0; i < SPI_SIZE_MANETTE; i++)
            _spiSend[i] = send[i];
              
          _spi.transfer16((uint16_t *) _spiSend, size, packetID, 1);
      }

      else{
          _lcd.println("ERROR");
          _lcd.print("Sending SPI : SPI_SIZE_MANETTE != ");
          _lcd.println(size/sizeof(send[0]));
      }
  }

  void GPIOManette::listenSPI(){
      _spi.events();
  }

  void GPIOManette::interruptSPI(uint16_t *buffer, uint16_t length, AsyncMST info){
      if (length/sizeof(buffer) == SPI_SIZE_CONTROLLEUR){
        _lcd.setCursor(0,0);
        _lcd.print("PacketID: "); _lcd.println(info.packetID);
        _lcd.print("Length: "); _lcd.println(length);
        for ( uint16_t i = 0; i < length/sizeof(buffer); i++ ) {
          float *temp = (float *)buffer;
          _spiReceive[i] = temp[i];
          _lcd.print(_spiReceive[i]); _lcd.print(" ");
        }
        _lcd.println();
      }
      else{
        _lcd.println("ERROR");
        _lcd.print("Reception SPI : SPI_SIZE_CONTROLLEUR != ");
        _lcd.println(length/sizeof(buffer));
      }
  }
#else

/*
* lorsque la connexion est perdue pendant que le thread est entrian  
* de rouler ce flag est activé
*/
void GPIOManette::setLost(bool state){
   _lost_while_running = state;
}

  /*
* Get le id du thread specifier
* whichThread : choix en premier thread (sending thread) ou 
* deuxieme (recieving thread)
*/
int GPIOManette::getThreadId(int whichThread){
   if (whichThread == 1) {
        return _threadId1;
    }else if(whichThread == 2){
        return _threadId2;
    }
}

/*
* Get la variable contenant le message à envoyer
*/
float* GPIOManette::getMessageToSend(){
    return _messageToSend ;
}

/*
* Get la variable contenant le message à recu
*/
float* GPIOManette::getMessageRecu(){
    return _messages ;
}

/*
* Set la variable sync_mode qui montre si le système est mode sybnchronisation
* pour l'instant on sait pas si c'est encore utile
* state : state de le synchronisation 
* true si en mode sync sinon false 
*/
void GPIOManette::setSyncMode(bool state){
   _sync_mode = state;
}

/*
* Set la variable sync_mode qui montre si le système est mode sybnchronisation
* pour l'instant on sait pas si c'est encore utile
* value : value avec laquelle remplacer le message à envoyer
* value est de la forme : {ID, DATA(_)}
*/
void GPIOManette::setMessageToSend(float* value){
    for(int i = 0 ; i < NOMBRE_DE_MESSAGE_ENOVYER; i++){
        _messageToSend[i] = value[i];
    }
}

/*
* Check if message received is corrupted
*/
bool GPIOManette::isCorrupted(float valeur){
  bool state = false;
  if (isnan(valeur)) state= true;
  if (isinf(valeur)) state= true;
  if (valeur > 1000) state= true;  // constant determined empirically
  if (valeur < -1000) state= true;
  // add if possible limit angle
  return state;
}

/*
* return if connexion is lost
*/
bool GPIOManette::isLost(){
    return _lost_while_running;
}

/*
* retourn l'état de synchronisation
*/
bool GPIOManette::isSyncMode(){
    return _sync_mode;
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
void GPIOManette::sendUART(void *arg){
  
    GPIOManette *thisobject = static_cast<GPIOManette*> (arg);

    int id2 = thisobject->getThreadId(THREAD2);
    float* messaget = thisobject->getMessageToSend();
    
    while(1){
      threads.wait(id2, PERIOD_OF_THREAD);
      if (!thisobject->isSyncMode()){
        
        Serial1.write(SYNC_BIT);
        for (int i = 0; i < NOMBRE_DE_MESSAGE_ENOVYER ; i++){
            byte *b = (byte *) &messaget[i] ;
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
void GPIOManette::receiveUART(void *arg){
    
    GPIOManette *thisobject = static_cast<GPIOManette*> (arg);
    int id1 = thisobject->getThreadId(THREAD1);
    float* messager = thisobject->getMessageRecu();
    char rec[BUFFER_SIZE];
    float count = 0;
    while(1){
      threads.wait(id1, PERIOD_OF_THREAD);
      count = 0 ;  
        bool corrupted = false;
        if (Serial1.available() && !thisobject->isSyncMode() && !thisobject->isLost()) {
            byte c = Serial1.read();

            Serial.print("Sync bit : ");
            Serial.println(c);
            
            while(c != ID_RECU && c != IM_LOST &&   !thisobject->isLost()){
                if (Serial1.available())
                    c = Serial1.read();
                if (count == TICK_WAIT_FOR_AVAILIBILY){
                    thisobject->setLost(true);
                    Serial.println("Lost now : ");
                    }
                }
            if(c == IM_LOST){thisobject->setLost(true);}
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
              if(thisobject->isSyncMode() || thisobject->isLost() ){
                thisobject->setLost(true);
                thisobject->setSyncMode(true);
                
                Serial.println("Resynchronixing ....");
                thisobject->sendHandshake();                
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
void GPIOManette::sendHandshake(){
      byte shake = 0 ;
      Serial.println("Start");
      Serial.print(" ");
      while (shake != COMFIRMATION_DONE){
          Serial1.write(ENVOI_HANDSHAKE_BIT);
          if (Serial1.available())
              shake = Serial1.read();
      }
      Serial.println("ok commence thread");
      Serial.print(" ");
      for (int i = 0; i < NOMBRE_ESSAIE; i++){
          Serial1.write(OK_SHAKE);
          delay(5);
          
      }
      setSyncMode(false);
      Serial.println("Je fais mon thread maintenant");
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
