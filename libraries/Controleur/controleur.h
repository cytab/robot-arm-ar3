#ifndef _Controleur_H
#define _Controleur_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"    // BasicLinearAlgebra by Tom Stewart for Matrix

#include "math.h"

#include "gpioControleur.h"

using namespace BLA;

/*********** Paramètres des moteurs et drivers **********************************************/


// Ratio de gearbox de chaque moteur
const float MOTOR_GEARBOX[NB_MOTEUR] = {10,47,50,14,1,19};

// Vitesse max de chaque moteur en RPM
const float MOTOR_MAX_RPM_SPEED[NB_MOTEUR] = {10,10,10,10,60,10};

// Ratio pour varier la vitesse de chaque moteur 
const float MOTOR_RPM_SPEED_RATIO[4] = {0.25,0.50,0.75,1};

// Résolution des drivers (en step/rev)
const float DRIVER_RESOLUTION[NB_MOTEUR] = {800,800,800,800,800,800};

// Facteur de conversion angle to step (en step/deg)
const float FACTOR_ANGLE_2_STEP[NB_MOTEUR] = {4*DRIVER_RESOLUTION[0]*MOTOR_GEARBOX[0]/(360),
                                              DRIVER_RESOLUTION[1]*MOTOR_GEARBOX[1]/(360),
                                              DRIVER_RESOLUTION[2]*MOTOR_GEARBOX[2]/(360),
                                              2.8*DRIVER_RESOLUTION[3]*MOTOR_GEARBOX[3]/(360),
                                              DRIVER_RESOLUTION[4]*MOTOR_GEARBOX[4]/(360),
                                              DRIVER_RESOLUTION[5]*MOTOR_GEARBOX[5]/(360)};

// Facteur de conversion step to angle (en deg/step)
const float FACTOR_STEP_2_ANGLE[NB_MOTEUR] = {1/FACTOR_ANGLE_2_STEP[0],
                                              1/FACTOR_ANGLE_2_STEP[1],
                                              1/FACTOR_ANGLE_2_STEP[2],
                                              1/FACTOR_ANGLE_2_STEP[3],
                                              1/FACTOR_ANGLE_2_STEP[4],
                                              1/FACTOR_ANGLE_2_STEP[5]};

// Structure encapsulant les valeurs max et min d'angles ou step
struct axisLimits 
{
  float positive; // Valeur max
  float negative;  // Valeur min
  float absolute; // max - min
}; 

// Valeurs min, max et plage de rotation de chaque joint en degrés
const axisLimits JOINTAXISLIMITS[NB_MOTEUR] = {{170,-170,340},
                                               {90,-42,132},
                                               {52,-89,141},
                                               {165,-165,330},
                                               {105,-105,210},
                                               {155,-155,310}};

//Distance maximale que peut parcourir chaque joint 
const float MAXDISTANCETOMOVE[NB_MOTEUR] = {JOINTAXISLIMITS[0].absolute*FACTOR_ANGLE_2_STEP[0],
                                            JOINTAXISLIMITS[1].absolute*FACTOR_ANGLE_2_STEP[1],
                                            JOINTAXISLIMITS[2].absolute*FACTOR_ANGLE_2_STEP[2],
                                            JOINTAXISLIMITS[3].absolute*FACTOR_ANGLE_2_STEP[3],
                                            JOINTAXISLIMITS[4].absolute*FACTOR_ANGLE_2_STEP[4],
                                            JOINTAXISLIMITS[5].absolute*FACTOR_ANGLE_2_STEP[5]};                                           



class Controleur: public GPIOControleur
{
public:

/****************** CONSTRUCTOR & DESTRUCTOR ******************************/

  Controleur();
  ~Controleur();

/****************** CALIBRATION FUNCTION ******************************/
bool moveToHomeInSteps(int directionTowardHome = 1);

/****************** SPEED INITIALIZER ******************************/
void initStepperSpeed();

/********************* STANDARD FUNCTIONS ********************************/

  void angle2step(float *angle);
  void step2angle();

/*************************** CHECKING ************************************/  

  bool checkAngleLimits(float* angle);
  bool checkStepLimits(int32_t* step);
  bool checkCollision();

  /*********** STEP MOVMENT CONTROL **********************************************/

  void move();
  void moveAsync();
  bool isStepRunning();
    
  /*********** CONTINUOUS MOVMENT CONTROL **********************************************/
  void rotMoveAsync(int moteur );
  void rotateStop();
  bool isRotateRunning();

  /*************************** UART ************************************/  
  void formulateMessage();
  
  /*********** SETTER **********************************************/

  void setNextPos(float* angularPosition);
  void setNewMaxSpeed(int index);
  void setAbsoluteParam(bool state);

  /*********** GETTERS **********************************************/

  float* getNextPos();
  float* getCurrentPos();
  bool getAbsoluteParam();

 
protected:
  StepControl _stepHandler;         // Controller for a step by step mouvement
  RotateControl _rotationHandler;  // Controller for continuous rotation
  float _nextAnglePos[6];         // Futur target in degree for each joint
  float _currentAnglePos[6];     // Current angular position for each axis
  bool _isAbsolute = true;      // Activate absolute (default) or relative movement

private:
  float SpeedTo_LimitSwitch;
  int32_t _nextStepPos[6];      // Futur value of the step counter for each axis
  int32_t _currentStepPos[6];  // Current value of the step counter for each axis

};

#endif // _Controleur_H
