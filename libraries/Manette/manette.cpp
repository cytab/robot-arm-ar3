#include "manette.h"

using namespace std;
using namespace BLA;

Manette::Manette():
GPIOManette(), Cinematique()
{
  _vitesse = VITESSE_MAX*0.5;
  _flag_Vitesse_Up = true;
  _flag_Vitesse_Down = true;

  _debug = false;
}

Manette::Manette(bool debug):
GPIOManette(debug), Cinematique(debug)
{
  _vitesse = VITESSE_MAX*0.5;
  _flag_Vitesse_Up = true;
  _flag_Vitesse_Down = true;

  _debug = debug;
}

Manette::~Manette(){
}

void Manette::lookVitesseChange(){
  if (_js_Boutg == false){
    if (_flag_Vitesse_Up){
      _flag_Vitesse_Up = false;
      _vitesse += VITESSE_MAX*0.1;
      if (_vitesse > VITESSE_MAX){
        _vitesse = VITESSE_MAX;
      }
    }
  }
  else{
    _flag_Vitesse_Up = true;
  }

  if (_js_Boutd == false){
    if (_flag_Vitesse_Down){
      _flag_Vitesse_Down = false;
      _vitesse -= VITESSE_MAX*0.1;
      if (_vitesse < VITESSE_MIN){
        _vitesse = VITESSE_MIN;
      }
    }
  }
    else{
    _flag_Vitesse_Down = true;
  }
}

void Manette::limitArticulations(){
    _flagLimitArticulations = false;
    for (int i = 0; i<6;i++){
      if (_articulations(i) < _limit[i].neg){
        _articulations(i) = _limit[i].neg;
        _flagLimitArticulations = true;
      }
      else if (_articulations(i) > _limit[i].pos){
        _articulations(i) = _limit[i].pos;
        _flagLimitArticulations = true;
      }
    }
  }

  void Manette::limitOrientation(){
    _flagLimitArticulations = false;
    for (int i = 3; i<6;i++){
      if (_positions(i) < -M_PI){
        _positions(i) = - M_PI;
        _flagLimitOrientations = true;
      }
      else if (_positions(i) > M_PI){
        _positions(i) = M_PI;
        _flagLimitOrientations = true;
      }
    }
  }

void Manette::js2Move(){
  lookVitesseChange();

  // Mode Cartesien
 Matrix<6> positions = _positions;
 if (!_switch_Mode1){
    if (!_switch_Mode2){
       if(JS_STATIC_MIN > _js_Vxg){
         _positions(0) += static_cast<float>(_js_Vxg - JS_STATIC_MIN)*_vitesse*K_XYZ;
      }
      else if(JS_STATIC_MAX < _js_Vxg){
        _positions(0) += static_cast<float>(_js_Vxg - JS_STATIC_MAX)*_vitesse*K_XYZ;
      }

      if(JS_STATIC_MIN > _js_Vyg){
        _positions(1) += static_cast<float>(_js_Vyg - JS_STATIC_MIN)*_vitesse*K_XYZ;
      }
      else if(JS_STATIC_MAX < _js_Vyg){
        _positions(1) += static_cast<float>(_js_Vyg - JS_STATIC_MAX)*_vitesse*K_XYZ;
      }

      if(JS_STATIC_MIN > _js_Vyd){
        _positions(2) += static_cast<float>(_js_Vyd - JS_STATIC_MIN)*_vitesse*K_XYZ;
      }
      else if(JS_STATIC_MAX < _js_Vyd){
         _positions(2) += static_cast<float>(_js_Vyd - JS_STATIC_MAX)*_vitesse*K_XYZ;
      }
    }
    else{
      if(JS_STATIC_MIN > _js_Vxg){
        _positions(3) += static_cast<float>(_js_Vxg - JS_STATIC_MIN)*_vitesse;
      }
      else if(JS_STATIC_MAX < _js_Vxg){
        _positions(3) += static_cast<float>(_js_Vxg - JS_STATIC_MAX)*_vitesse;
      }

      if(JS_STATIC_MIN > _js_Vyg){
        _positions(4) += static_cast<float>(_js_Vyg - JS_STATIC_MIN)*_vitesse;
      }
      else if(JS_STATIC_MAX < _js_Vyg){
         _positions(4) += static_cast<float>(_js_Vyg - JS_STATIC_MAX)*_vitesse;
      }

      if(JS_STATIC_MIN > _js_Vyd){
        _positions(5) += static_cast<float>(_js_Vyd - JS_STATIC_MIN)*_vitesse;
      }
      else if(JS_STATIC_MAX < _js_Vyd){
        _positions(5) += static_cast<float>(_js_Vyd - JS_STATIC_MAX)*_vitesse;
      }
    }
    limitOrientation();

    Cinematique::cinematiqueInverse();
    if(_flagNanDetected || _flagArticulationTooFar){
      _positions = positions;
    }
    gripOn_Off();

  }
  // Mode Articulaire 
  else{
    // q1, q2, q3
    if(!_switch_Mode2){
      if(JS_STATIC_MIN > _js_Vxg){
         _articulations(0) += static_cast<float>(_js_Vxg - JS_STATIC_MIN)*_vitesse;
      }
      else if(JS_STATIC_MAX < _js_Vxg){
        _articulations(0) += static_cast<float>(_js_Vxg - JS_STATIC_MAX)*_vitesse;
      }

      if(JS_STATIC_MIN > _js_Vyg){
        _articulations(1) += static_cast<float>(_js_Vyg - JS_STATIC_MIN)*_vitesse;
      }
      else if(JS_STATIC_MAX < _js_Vyg){
        _articulations(1) += static_cast<float>(_js_Vyg - JS_STATIC_MAX)*_vitesse;
      }

      if(JS_STATIC_MIN > _js_Vyd){
        _articulations(2) += static_cast<float>(_js_Vyd - JS_STATIC_MIN)*_vitesse;
      }
      else if(JS_STATIC_MAX < _js_Vyd){
         _articulations(2) += static_cast<float>(_js_Vyd - JS_STATIC_MAX)*_vitesse;
      }
    }
    
    // q4, q5, q6
    else{
      if(JS_STATIC_MIN > _js_Vxg){
        _articulations(3) += static_cast<float>(_js_Vxg - JS_STATIC_MIN)*_vitesse;
      }
      else if(JS_STATIC_MAX < _js_Vxg){
        _articulations(3) += static_cast<float>(_js_Vxg - JS_STATIC_MAX)*_vitesse;
      }

      if(JS_STATIC_MIN > _js_Vyg){
        _articulations(4) += static_cast<float>(_js_Vyg - JS_STATIC_MIN)*_vitesse;
      }
      else if(JS_STATIC_MAX < _js_Vyg){
         _articulations(4) += static_cast<float>(_js_Vyg - JS_STATIC_MAX)*_vitesse;
      }

      if(JS_STATIC_MIN > _js_Vyd){
        _articulations(5) += static_cast<float>(_js_Vyd - JS_STATIC_MIN)*_vitesse;
      }
      else if(JS_STATIC_MAX < _js_Vyd){
        _articulations(5) += static_cast<float>(_js_Vyd - JS_STATIC_MAX)*_vitesse;
      }
    }
    limitArticulations();

    Cinematique::cinematiqueDirect(); 
    gripOn_Off();
  }
}

void Manette::printInput(){
  if(_debug){
    Serial.print("Articulations : \n");  
    Serial.print(_articulations(0));
    Serial.print(" ");
    Serial.print(_articulations(1));
    Serial.print(" ");
    Serial.println(_articulations(2));
    Serial.print(_articulations(3));
    Serial.print(" ");
    Serial.print(_articulations(4));
    Serial.print(" ");
    Serial.print(_articulations(5));
    Serial.println("\n");

    Serial.print("Positions : \n");  
    Serial.print((int)_positions(0));
    Serial.print(" ");
    Serial.print((int)_positions(1));
    Serial.print(" ");
    Serial.println((int)_positions(2));
    Serial.print(_positions(3));
    Serial.print(" ");
    Serial.print(_positions(4));
    Serial.print(" ");
    Serial.print(_positions(5));
    Serial.println("\n");

    Serial.print("Vitesse: ");
    Serial.print(int(_vitesse/VITESSE_MAX*100));
    Serial.println("%\n");
  }

  _lcd.print("Articulations : \n");  
  _lcd.print(_articulations(0));
  _lcd.print(" ");
  _lcd.print(_articulations(1));
  _lcd.print(" ");
  _lcd.print(_articulations(2));
  _lcd.print("      \n");
  _lcd.print(_articulations(3));
  _lcd.print(" ");
  _lcd.print(_articulations(4));
  _lcd.print(" ");
  _lcd.print(_articulations(5));
  _lcd.print("      \n\n");


  _lcd.print("Positions : \n");  
  _lcd.print((int)_positions(0));
  _lcd.print(" ");
  _lcd.print((int)_positions(1));
  _lcd.print(" ");
  _lcd.print((int)_positions(2));
  _lcd.print("      \n");
  _lcd.print(_positions(3));
  _lcd.print(" ");
  _lcd.print(_positions(4));
  _lcd.print(" ");
  _lcd.print(_positions(5));
  _lcd.print(" ");
  _lcd.print(_grip);
  _lcd.print("    \n\n");

  _lcd.print("Vitesse: ");
  _lcd.print(int(_vitesse/VITESSE_MAX*100));
  _lcd.print("%      \n\n");

  if(_flagNanDetected){
    _lcd.print("Cinematique inverse failed");
    _lcd.print("Not in range\n");
  }
  else if(_flagArticulationTooFar){
    _lcd.print("Cinematique inverse failed");
    _lcd.print("Articulation too far\n");
  }
  else if(_flagLimitArticulations){
    _lcd.print("Articulation too far\n");
  }
  else if(_flagLimitOrientations){
    _lcd.print("Orientation too far\n");
  }
  else{
    _lcd.print("                          ");
    _lcd.print("                     ");
  }
  
  
  _lcd.setCursor(0, 0);
}

void Manette::gripOn_Off(){
  readInput();
  if (!getInput(BOUTON_G2))
    _grip = false;
  else if (!getInput(BOUTON_G1))
    _grip = true ; 
}

void Manette::formulateMessage(){
  for(int i = 0 ; i < NOMBRE_DE_MESSAGE_ENOVYER-1; i++){
    _messageToSend[i] = _articulations(i);
  }
  _messageToSend[NOMBRE_DE_MESSAGE_ENOVYER-1] = _grip;

}