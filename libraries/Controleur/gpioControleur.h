#ifndef _GPIOControleur_H
#define _GPIOControleur_H

#include "Arduino.h"
#include "SPI_MSTransfer.h"
#include "BasicLinearAlgebra.h"    // BasicLinearAlgebra by Tom Stewart for Matrix
#include "TeensyStep.h"
#include <memory>
#include <..\..\Controleur\config_controleur.h>
#include <stdint.h>
#include <TeensyThreads.h>


#define NB_MOTEUR 6
#define SPEED_MAX 10000

const int JREPOS[NB_MOTEUR] ={0,0,0,0,0,0};

/************************ NUMPIN **************************************************************/
// SPI for manette
#ifndef UART 
    #define SPI_SIZE_CONTROLEUR 6
    #define SPI_SIZE_MANETTE 6
    #define SPI_CS       31 // Blanc - 5
    #define SPI_SCK      32 // Vert - 6
    #define SPI_MOSI      0 // Jaune - 7
    #define SPI_MISO      1 

#else
    #define NOMBRE_DE_MESSAGE_RECU 7
    #define NOMBRE_DE_MESSAGE_ENOVYER 6
    #define RECEPTION_HANDSHAKE_BIT 144
    #define CONNECTION_DONE 145
    #define COMFIRMATION_SHAKE 146
    #define NOMBRE_ESSAIE 50
    #define SYNC_BIT 200
    #define ID_RECU 240
    #define BUFFER_SIZE 100
    #define BAUD_RATE 38400
    #define RX 0
    #define TX 1
    #define PERIOD_OF_THREAD 200
    #define THREAD1 1
    #define THREAD2 2
    #define TICK_WAIT_FOR_AVAILIBILY 200000
    #define IM_LOST 100
#endif

//Motor
const int JSTEP_PIN[NB_MOTEUR] = {28,26,24,8,6,4};
const int JDIR_PIN[NB_MOTEUR]  = {29,27,25,9,7,5};
const float MAX_SPEED[NB_MOTEUR] = {10,10,100,100,100,100};

//Limit switch
const int JLIM_PIN[NB_MOTEUR] = {33, 34, 35, 36, 37, 38};
/*********************************************************************************************/

class GPIOControleur{
public:
    GPIOControleur();
    ~GPIOControleur();

    /*********** SPI *****************************************************/
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
        void waitHandshake();
        
        /**************  GETTER and setter *****/
        float* getMessageRecu();
        int getThreadId(int whichThread);
        float* getMessageToSend();
        bool isSyncMode();
        bool isLost();
        bool isCorrupted(float valeur);
        void setMessageToSend(float* value);
        void setSyncMode(bool state );
        void setLost(bool state);
        void formulateMessage();
        /*****************************************/

    #endif 
    /*********************************************************************/

    /************** LIMIT SWITCH *****************************************/
    bool getLimit(int n);
    void checkLimitSwitch();
    /*********************************************************************/

    /************** STEPPER **********************************************/
    void setPosition(int32_t *position);
    void setTarget(int32_t *target, bool absolute = false);

    int32_t getPosition(int nMoteur);
    /*********************************************************************/

protected:
    #ifndef UART
        SPI_MSTransfer _spi;
        float _spiSend[SPI_SIZE_CONTROLEUR];
        volatile float _spiReceive[SPI_SIZE_MANETTE];
    #else
        float _messages[NOMBRE_DE_MESSAGE_RECU];
        float _messageToSend[NOMBRE_DE_MESSAGE_ENOVYER];
        /************** Thread *******************/
        int _threadId1, _threadId2 ; 
        bool _sync_mode;
        bool _lost_while_running;
    #endif

    bool _limits[NB_MOTEUR];
    Stepper *_pMoteurs[NB_MOTEUR];
    

};

#endif // _GPIOControleur_H
