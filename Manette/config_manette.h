/*****************************************************
 Fichier de configuration pour Manette
 *****************************************************/

#define DEBUG 1 // Comment for MAIN. Uncomment for DEBUG.
    #ifdef DEBUG 
       // #define SPI 1
        #ifndef SPI
           // #define UART_DEBUG 1   . 
            #define UART 1 // SPI need to be comment. Comment for disabling UART
        #endif
        //#define DIRECT 1
        //#define INVERSE 1
        #define MANETTE 1
#elif //MAIN

#endif 
