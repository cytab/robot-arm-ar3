/*****************************************************
 Fichier de configuration pour le contrôleur 
 *****************************************************/

#define DEBUG 1 // Comment for MAIN 
#define GUI 0  // Comment for enabling using interface graphique

#ifdef DEBUG 
   // #define SPI 1
    #ifndef SPI
        #define UART 1   // SPI need to be comment. Comment for disabling UART. 
    #endif
    
     // #define MOVETOHOMEINSTEPS 1
      //#define GETANGLES 1
      //#define GETSTEPS 1
      //#define MOVE 1
      // #define SAFETY_ANGLE 1
#elif //MAIN
    #define SPI 1   // Comment for disabling SPI/Enabling UART 

#endif 


