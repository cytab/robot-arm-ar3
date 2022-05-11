#ifndef _GPIOManette_H
#define _GPIOManette_H

#include "Arduino.h"
#include <ILI9341_t3.h>
#include <SPI_MSTransfer.h>
#include <..\..\Manette\config_manette.h>
#include <stdint.h>
#include "HardwareSerial.h"
#include <TeensyThreads.h>

/************************ NUMPIN **************************************************************/
// Mode
#define SWITCH_MODE1   36   // Switch - Changer de mode Cartésien à Joint
#define SWITCH_MODE2   35   // Switch - Changer de q1,q2,q3 à q4,q5,q6 / x,y,z à phi,theta,psi

// Bouton Gauche
#define BOUTON_G1     34  // Bouton Gauche #1 - Ouvrir pince
#define BOUTON_G2     33   // Bouton Gauche #2 - Fermer pince

// Bouton Droit
#define BOUTON_D1     24   // Bouton Droit #1 -  À DÉFINIR
#define BOUTON_D2     27   // Bouton Droit #2 -  À DÉFINIR
#define BOUTON_D3     26   // Bouton Droit #3 -  À DÉFINIR
#define BOUTON_D4     25   // Bouton Droit #4 -  À DÉFINIR

// JoyStick Gauche
#define JS_VYG        A22   // JoyStick Gauche Vx - Déplacement X
#define JS_VXG        A21   // JoyStick Gauche Vy - Déplacement Y
#define JS_BOUTG       39  // JoyStick Gauche Bouton - Vitesse Up

// JoyStick Droit
#define JS_VYD        A19   // JoyStick Droit Vx - RIEN
#define JS_VXD        A18   // JoyStick Droit Vy - Déplacement Z
#define JS_BOUTD        6   // JoyStick Droit Bouton - Vitesse down

// Écran LCD
#define LCD_CS       10 
#define LCD_DC        9
#define LCD_MOSI     11
#define LCD_SCLK     14
#define LCD_MISO     12
#define LCD_RST     255 


// SPI
#ifndef UART 
    #define SPI_BUS     SPI1
    #define SPI_CS       31 // Blanc - 5
    #define SPI_SCK      32 // Vert - 6
    #define SPI_MOSI      0 // Jaune - 7
    #define SPI_MISO      1 // Orange - 4
#else
    #define NOMBRE_DE_MESSAGE_RECU 6
    #define NOMBRE_DE_MESSAGE_ENOVYER 7
    #define ENVOI_HANDSHAKE_BIT 144
    #define COMFIRMATION_DONE 145
    #define OK_SHAKE 146
    #define NOMBRE_ESSAIE 5
    #define SYNC_BIT 240
    #define ID_RECU 200
    #define BAUD_RATE 38400
    #define BUFFER_SIZE 100
    #define RX 0
    #define TX 1
    #define PERIOD_OF_THREAD 200
    #define THREAD1 1
    #define THREAD2 2
    #define TICK_WAIT_FOR_AVAILIBILY 200000
    #define IM_LOST 100
#endif
// 3.3 V - Rouge - 2
// GND   - Noire - 3
/*********************************************************************************************/

#define SPI_SIZE_MANETTE 6
#define SPI_SIZE_CONTROLLEUR 6

class GPIOManette{
public:
    GPIOManette();
    GPIOManette(bool debug);
    ~GPIOManette();

    int getInput(int numPin);
    void readInput();
    void printInput();
#ifdef SPI
    SPI_MSTransfer getSPI();
    volatile float* getReceive();
    void setSendSPI(float *send, size_t size);
    void sendSPI(int packetID);
    void sendSPI(float *send, size_t size, int packetID);
    void listenSPI();
    void interruptSPI(uint16_t *buffer, uint16_t length, AsyncMST info);
#endif
#ifdef UART
    static void sendUART(void *arg);
    static void receiveUART(void *arg);
    void sendHandshake();
    float* getMessageRecu();
    /**************  GETTER and setter *****/
    int getThreadId(int whichThread);
    float* getMessageToSend();
    bool isSyncMode();
    bool isLost();
    bool isCorrupted(float valeur);
    void setMessageToSend(float* value); // For debugging purpose. In MAIN, use Manette::formulateMessage() to set MessageToSend 
    void setSyncMode(bool state );
    void setLost(bool state);
    /*****************************************/
#endif

protected:
    ILI9341_t3 _lcd;

#ifdef SPI 
    SPI_MSTransfer _spi;
    float _spiSend[SPI_SIZE_MANETTE];
    volatile float _spiReceive[SPI_SIZE_CONTROLLEUR];
    
    float _spiSend[SPI_SIZE_MANETTE];
    volatile float _spiReceive[SPI_SIZE_CONTROLLEUR];
#endif
#ifdef UART
    float _messages[NOMBRE_DE_MESSAGE_RECU];
    float _messageToSend[NOMBRE_DE_MESSAGE_ENOVYER];
    /************** Thread *******************/
    //std::thread *_thread1;
    //std::thread *_thread2;
    int _threadId1, _threadId2 ; 
    //utile ou pas 
    bool _corruptedData;
    bool _sync_mode;
    bool _lost_while_running;
#endif
    SPI_MSTransfer _spi;



    bool _switch_Mode1;
    bool _switch_Mode2;

    bool _bouton_G1;
    bool _bouton_G2;
    bool _grip ; 

    bool _bouton_D1;
    bool _bouton_D2;
    bool _bouton_D3;
    bool _bouton_D4;

    int _js_Vxg;
    int _js_Vyg;
    bool _js_Boutg;

    int _js_Vxd;
    int _js_Vyd;
    bool _js_Boutd;

    bool _debug_GPIO;

};

#endif // _GPIOManette_H
