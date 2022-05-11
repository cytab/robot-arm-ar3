#ifndef _MANETTE_H
#define _MANETTE_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"    // BasicLinearAlgebra by Tom Stewart for Matrix

#include "GPIOManette.h"
#include "cinematique.h"
#include "Servo.h"
using namespace BLA;

#define JS_STATIC_MIN 450
#define JS_STATIC_MAX 600

#define ARTICULATION_MIN -3.1416f
#define ARTICULATION_MAX  3.1416f

#define VITESSE_MAX 0.001f
#define VITESSE_MIN 0.000f
#define K_XYZ 100


class Manette: public GPIOManette, public Cinematique{
public:
  Manette();
  Manette(bool debug);
  ~Manette();

  void lookVitesseChange();

  void limitArticulations();
  void limitOrientation();

  void js2Move();
  void printInput();
  void gripOn_Off();
  void formulateMessage();

private:
  float _vitesse;
  bool _flag_Vitesse_Up;
  bool _flag_Vitesse_Down;

  bool _flagLimitArticulations;
  bool _flagLimitOrientations;

  bool _debug;
};

#endif // _MANETTE_H
