#include "controleur.h"

using namespace std;
using namespace BLA;

/**
 * @brief Construct a new Controleur:: Controleur object
 * initialize GpioControleur , _stepHandler
 * 
 */
Controleur::Controleur():
GPIOControleur(),
_stepHandler()
{
    initStepperSpeed();
    //moveToHomeInSteps();
}

/**
 * @brief Destroy the Controleur:: Controleur object
 * 
 */
Controleur::~Controleur()
{
    
}

/**
 * @brief Converts positions from degree to step for each joint
 * Used to update the target for each motor with _nextStepPos attribute
 * @param angle[in]: array of positions in degrees
 * @return void
 *
 */
void Controleur::angle2step(float *angle)
{
    
    for(int i=0;i<NB_MOTEUR;i++)
    {
        _nextStepPos[i] = (int32_t) (angle[i]*FACTOR_ANGLE_2_STEP[i]);
    }
}



/**
 * @brief Converts positions from steps to degrees
 * Used to recover the position of each axis in degrees from internal stepper counters 
 * The attribute _currentAnglePos is then udpdated
 * @return void
 */
void Controleur::step2angle()
{
    for(int i=0;i<NB_MOTEUR;i++)
    {
        _currentAnglePos[i] = (float) (_currentStepPos[i]*FACTOR_STEP_2_ANGLE[i]);
    }
}


/**
 * @brief Initializes the speed of the stepper motors to their maximum authorized value
 * @return void
 * 
 */
void Controleur::initStepperSpeed()
{
    for (int i = 0 ; i<NB_MOTEUR ; i++)
    {
        _pMoteurs[i]->setMaxSpeed((360/60)*MOTOR_MAX_RPM_SPEED[i]*FACTOR_ANGLE_2_STEP[i]);
        _pMoteurs[i]->setAcceleration((360/60)*MOTOR_MAX_RPM_SPEED[i]*FACTOR_ANGLE_2_STEP[i]*4);
    }
}

/**
 * @brief Modifie la vitesse de moteurs en appliquant un ratio à tous les moteurs
 * @param index[in]: 
 * @return void
 * 
 */
void Controleur::setNewMaxSpeed(int index)
{
    for (int i = 0 ; i<NB_MOTEUR ; i++)
    {
        _pMoteurs[i]->setMaxSpeed((360/60)*MOTOR_MAX_RPM_SPEED[i]*MOTOR_RPM_SPEED_RATIO[index]*FACTOR_ANGLE_2_STEP[i]);
        _pMoteurs[i]->setAcceleration((360/60)*MOTOR_MAX_RPM_SPEED[i]*MOTOR_RPM_SPEED_RATIO[index]*FACTOR_ANGLE_2_STEP[i]*4);
    }
}

/**
 * @brief Vérifie que les angles issus de la cinématique inverse 
 * sont dans la plage de sécurité
 * 
 * @param angle[in]: Orientation future de chaque axe en dégrés 
 * @return bool : True si tous les angles sont dans la plage de sécurité, False sinon 
 */
 bool Controleur::checkAngleLimits(float* angle)
 {
     bool allValid = true;
     for(int i=0;i<NB_MOTEUR;i++)
     {
         if ((angle[i]<JOINTAXISLIMITS[i].negative) || (angle[i]>JOINTAXISLIMITS[i].positive)) 
         {
             Serial.println("Axe "); Serial.print(i); Serial.print(" d'angle ");
             Serial.print(angle[i]); Serial.print("° est hors de la plage ["); Serial.print(JOINTAXISLIMITS[i].negative);
             Serial.print(";"); Serial.print(JOINTAXISLIMITS[i].positive); Serial.println("]°");
             allValid = false;
         }
     }
     Serial.println("Vérification des angles OK!");
     return allValid;
 }


/**
 * @brief 
 * 
 * @param angularPosition[in]: 
 * @return void 
 */
void Controleur::setNextPos(float* angularPosition)
{
    for (int i = 0; i < NB_MOTEUR; i++)
    {
        _nextAnglePos[i] = angularPosition[i];
    }
}


/**
 * @brief Set the type of movement for all motors
 * Can be relative or absolute
 * 
 * @param state[in]: True for absolute steps or false for relative
 * @return void 
 */
void Controleur::setAbsoluteParam(bool state)
{
    _isAbsolute = state;
}


/**
 * @brief return de type of movement for all motors.
 * Can be relative or absolute
 * 
 * @return bool : the _isAbsolute attribute
 */
bool Controleur::getAbsoluteParam()
{
    return _isAbsolute;
}



/*
* Get l'attribut _nextPos
* return : _nextPos
*/
float* Controleur::getNextPos()
{
    return _nextAnglePos;
}


/*
* Get l'attribut _angles
* return : _angles
*/
float* Controleur::getCurrentPos()
{
    for(int i =0; i<NB_MOTEUR;i++)
    {
        _currentStepPos[i]= getPosition(i);
    }
    step2angle();
    return _currentAnglePos;
}

/*
* Initie un mouvement le mouvement des moteurs de façon non bloquant
* Donc le code ne s'arrête meme si le target n'est pas reach
* return: void
*/
void Controleur::moveAsync()
{
    angle2step(_nextAnglePos);
    setTarget(_nextStepPos, _isAbsolute);
    _stepHandler.moveAsync(*_pMoteurs[0], *_pMoteurs[1], *_pMoteurs[2], 
                           *_pMoteurs[3], *_pMoteurs[4], *_pMoteurs[5]);
}

/*
* Initie un mouvement le mouvement des moteurs de façon  bloquant
* return: void
*/
void Controleur::move()
{
    angle2step(_nextAnglePos);
    setTarget(_nextStepPos, _isAbsolute);
    _stepHandler.move(*_pMoteurs[0], *_pMoteurs[1], *_pMoteurs[2], 
                           *_pMoteurs[3], *_pMoteurs[4], *_pMoteurs[5]);
}

/*
* Initie un mouvement de rotation continue des moteurs de façon  non bloquant
* int moteur : quel moteur tourner
* return: void
*/
void Controleur::rotMoveAsync(int moteur)
{
    _rotationHandler.rotateAsync(*_pMoteurs[moteur]);
}


void Controleur::rotateStop()
{
    _rotationHandler.stop();
}

/*
* Fonction qui vérifie si les moteurs sont en mouvement
* return: bool
*/
bool Controleur::isStepRunning()
{
    return _stepHandler.isRunning();
}

/*
* Fonction qui vérifie si les moteurs sont en mouvement
* return: bool
*/
bool Controleur::isRotateRunning()
{
    return _rotationHandler.isRunning();
}



bool Controleur::moveToHomeInSteps(int directionTowardHome)
{
    #ifdef DEBUG
        checkLimitSwitch();
        for (int i=0; i<NB_MOTEUR; i++)
        {
            Serial.print("Moteur ");
            Serial.print(i+1);
            Serial.println(" en cours de calibration ...");

            Serial.print("MAXDISTANCETOMOVE : ");
            Serial.println(FACTOR_STEP_2_ANGLE[i]);
            
            float homingPositions[6] = { 0, 0, 0, 0, 0, 0 };
            homingPositions[i] = 1000;
            setNextPos(homingPositions);
            int count = 0;
            if(!isStepRunning()){
                moveAsync(i);
            } 

            while(getLimit(i) && (MAXDISTANCETOMOVE[i]>= getPosition(i)))
            {
                //setNextPos(homingPositions);
                checkLimitSwitch();
            }
            rotateStop();
            if (getLimit(i)){
                Serial.print("Moteur ");
                Serial.print(i+1);
                Serial.println(" n'as pas atteint");
            } 
        }
        // -------------------- move to DH position --------------------------------//
        move(); // A faire! moveback to pos repos
        return true;
    #else // MAIN
        // A faire
    #endif
}

void Controleur::formulateMessage(){
    checkLimitSwitch();
  for(int i = 0 ; i < NOMBRE_DE_MESSAGE_ENOVYER; i++){
    _messageToSend[i] = getLimit(i);
  }

}