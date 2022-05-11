#include "cinematique.h"

using namespace std;
using namespace BLA;


/**
 * Constructeur 
 */
CinematiqueAlgebra::CinematiqueAlgebra(){
}

/**
 * Destructeur 
 */
CinematiqueAlgebra::~CinematiqueAlgebra(){

}

/**
 * Renvoie l'angle dans l'intervalle [-pi, pi] 
 */
float CinematiqueAlgebra::wrapAngle(float angle_rad){
    return angle_rad + 2*M_PI*(1- ceil((angle_rad+M_PI)/(2*M_PI)));
}

/**
 * Matrice de rotation centré à l'origine suivant l'Axe X 
 * return : Maitrice 3x3
 */
Matrix<3,3> CinematiqueAlgebra::getRotateX(float angle){
    Matrix<3,3> R ;
        R = {1, 0, 0,
            0, cosf(angle), -sinf(angle),
            0, sinf(angle), cosf(angle) };

    return R;
}

/**
 * Matrice de rotation centré à l'origine suivant l'Axe y
 * return : Maitrice 3x3
 */
Matrix<3,3> CinematiqueAlgebra::getRotateY(float angle){
    Matrix<3,3> R ;
        R = {cosf(angle), 0, sinf(angle),
            0, 1, 0,
            -sinf(angle), 0, cosf(angle) };
    return R;
}

/**
 * Matrice de rotation centré à l'origine suivant l'Axe Z
 * return : Maitrice 3x3
 */
Matrix<3,3> CinematiqueAlgebra::getRotateZ(float angle){
    Matrix<3,3> R ;
        R = {cosf(angle), -sinf(angle), 0,
            sinf(angle), cosf(angle), 0,
            0, 0, 1 };

    return R;
}


/**
 * Transforme la matrice de rotation R 3x3 en angle d'euler pour XYZ.
 * return : Matrice 3x1 contenant les anlges phi, theta et psi défini pour XYZ.
 */
Matrix<3> CinematiqueAlgebra::getDCM2EulXYZ(Matrix<3,3> matrixR){
  float phi,theta, psi ; 
  Matrix<3> euler ;
  if((matrixR(0,2) > 0.99 && matrixR(0,2) < 1.01) || (matrixR(0,2) < -0.99 && matrixR(0,2) > -1.01)){
    phi = atan2(matrixR(2,1),matrixR(1,1)); 
    theta = asin(matrixR(0,2));
    psi = 0;
  }
  else
  {
    phi = atan2(-matrixR(1,2),matrixR(2,2)); 
    theta = atan2(matrixR(0,2),(sqrt(pow(matrixR(1,2),2)+pow(matrixR(2,2),2)))); 
    psi = atan2(-matrixR(0,1),matrixR(0,0)); 
  }

  phi = CinematiqueAlgebra::wrapAngle(phi);
  theta = CinematiqueAlgebra::wrapAngle(theta);
  psi = CinematiqueAlgebra::wrapAngle(psi);

  euler = {phi, theta, psi};
  return euler;
}

/**
 * Transforme la matrice de positions 4x4 en angle d'euler pour XYZ.
 * return : Matrice 3x1 contenant les anlges phi, theta et psi défini pour XYZ.
 **/
Matrix<3> CinematiqueAlgebra::getDCM2EulXYZ(Matrix<4,4> matrixT){
  float phi,theta, psi ; 
  Matrix<3> euler ;
  if((matrixT(0,2) > 0.99 && matrixT(0,2) < 1.01) || (matrixT(0,2) < -0.99 && matrixT(0,2) > -1.01)){
    phi = atan2(matrixT(2,1),matrixT(1,1)); 
    theta = asin(matrixT(0,2));
    psi = 0;
  }
  else
  {
    phi = atan2(-matrixT(1,2),matrixT(2,2)); 
    theta = atan2(matrixT(0,2),(sqrt(pow(matrixT(1,2),2)+pow(matrixT(2,2),2)))); 
    psi = atan2(-matrixT(0,1),matrixT(0,0)); 
  }

  phi = CinematiqueAlgebra::wrapAngle(phi);
  theta = CinematiqueAlgebra::wrapAngle(theta);
  psi = CinematiqueAlgebra::wrapAngle(psi);

  euler = {phi, theta, psi};
  return euler;
}

/**
 * Transforme la matrice de rotation R 3x3 du poignet du bras robotique en angle d'euler pour XYX
 * return : Matrice 3x1 contenant les anlges phi, theta et psi défini pour XYX
 */
Matrix<3> CinematiqueAlgebra::getDCM2EulXYX(Matrix<3,3> matrixR){
  float phi,theta, psi ; 
  Matrix<3> euler ;
  if((matrixR(0,0) > 0.99 && matrixR(0,0) < 1.01) || (matrixR(0,0) < -0.99 && matrixR(0,0) > -1.01)){
    phi = atan2(matrixR(2,1),matrixR(1,1));  // Possibilité de mettre phi à 0 et trouver psi. J6 tourne plutot que J4.
    theta = acos(matrixR(0,0));
    psi = 0; 
  }

  else
  {
    phi = atan2(matrixR(1,0),-matrixR(2,0)); 
    theta = atan2(sqrt(pow(matrixR(1,0),2)+pow(matrixR(2,0),2)),matrixR(0,0)); 
    psi = atan2(matrixR(0,1),matrixR(0,2));
  }

  phi = CinematiqueAlgebra::wrapAngle(phi);
  theta = CinematiqueAlgebra::wrapAngle(theta);
  psi = CinematiqueAlgebra::wrapAngle(psi);

  euler = {phi, theta, psi};
  return euler;
}

/**
 * Détermine la position des articulations (angles) de l'épaule qui vont satisfaire la position 
 * du Wdes (point de rencontre des articulations du poignet)
 * 
 * param : Wdes - Position du point Wdes 
 * return : Maitrice 3x1
 */
Matrix<3> CinematiqueAlgebra::getAnthroRRR(Matrix<3> Wdes){
    Matrix<3> thetas = {0,0,0};

    // Retrait de l'offset en  pour changement de repère
    float Px = sqrt(pow(Wdes(0),2)+pow(Wdes(1),2)) - A1;
    float Pz = Wdes(2) - D1;

    float d = sqrt(pow(Px,2) + pow(Pz,2));
    float alpha = atan2(Pz, Px);

    thetas(0) = - atan2(Wdes(1),Wdes(0));
    thetas(1) = - alpha - acos((pow(A2,2) + pow(d,2) -  pow(D4,2))/(2*d*A2));
    thetas(2) =   acos((pow(d,2)  - pow(A2,2) - pow(D4,2))/(2*D4*A2));

/*
    switch(typesol) {
      case EPAULE_DROITE_COUDE_BAS :
        thetas(0) = - (atan2(Wdes(1),Wdes(0)) + asin(var_theta1));
        thetas(1) =   alpha - acos(var_theta2);
        thetas(2) = acos(var_theta3);
        break;

      case EPAULE_DROITE_COUDE_HAUT :
        thetas(0) = - (atan2(Wdes(2),Wdes(1)) + asin(var_theta1));
        thetas(1) =    alpha + acos(var_theta2);
        thetas(2) = -acos(var_theta3);
        break;

      case EPAULE_GAUCHE_COUDE_BAS :
        thetas(0) = - (atan2(Wdes(2),Wdes(1)) - asin(var_theta1) + M_PI);
        thetas(1) =   M_PI - alpha - acos(var_theta2);
        thetas(2) =   acos(var_theta3);
        break;

      case EPAULE_GAUCHE_COUDE_HAUT :
        thetas(0) = - (atan2(Wdes(2),Wdes(1)) - asin(var_theta1) + M_PI);
        
        thetas(1) =  M_PI - alpha + acos(var_theta2);
        thetas(2) = -acos(var_theta3);

        break;
    
      default :
        thetas(0) = -1;
        thetas(1) = -1;
        thetas(2) = -1;
        return thetas;    
  }
*/

  thetas(0) = CinematiqueAlgebra::wrapAngle(thetas(0));
  thetas(1) = CinematiqueAlgebra::wrapAngle(thetas(1));
  thetas(2) = CinematiqueAlgebra::wrapAngle(thetas(2));
  
  return thetas;
}




/****************************************** Cinematique ****************************/

Cinematique::Cinematique() :
CinematiqueAlgebra()
{
  _articulations = {0,0,0, 0,0,0};
  _positions     = {756.44, -0.00, 151.27, -M_PI, M_PI/2, 0.00};

    // Valeur à changer après calibration
    _limit[0].pos = 3.14 ;
    _limit[0].neg = -3.14;
    _limit[1].pos = 3.14 ;
    _limit[1].neg = -3.14;
    _limit[2].pos = 3.14 ;
    _limit[2].neg = -3.14;
    _limit[3].pos = 3.14 ;
    _limit[3].neg = -3.14;
    _limit[4].pos = 3.14 ;
    _limit[4].neg = -3.14;
    _limit[5].pos = 3.14 ;
    _limit[5].neg = -3.14;

  _matrixT = {1,0,0,0,
              0,1,0,0,
              0,0,1,0,
              0,0,0,1};
  _matrixT_inv = {1,0,0,0,
                  0,1,0,0,
                  0,0,1,0,
                  0,0,0,1};

 _flagNanDetected = 0;
 _flagArticulationTooFar = 0 ;

 _debug_Cinematique = false;
}

Cinematique::Cinematique(bool debug) :
CinematiqueAlgebra()
{
  _articulations = {0,0,0, 0,0,0};
  _positions     = {756.44, -0.00, 151.27, -M_PI, M_PI/2, 0.00};

    // Valeur à changer après calibration
    _limit[0].pos = 3.14 ;
    _limit[0].neg = -3.14;
    _limit[1].pos = 3.14 ;
    _limit[1].neg = -3.14;
    _limit[2].pos = 3.14 ;
    _limit[2].neg = -3.14;
    _limit[3].pos = 3.14 ;
    _limit[3].neg = -3.14;
    _limit[4].pos = 3.14 ;
    _limit[4].neg = -3.14;
    _limit[5].pos = 3.14 ;
    _limit[5].neg = -3.14;

  _matrixT = {1,0,0,0,
              0,1,0,0,
              0,0,1,0,
              0,0,0,1};
  _matrixT_inv = {1,0,0,0,
                  0,1,0,0,
                  0,0,1,0,
                  0,0,0,1};

 _flagNanDetected = 0;
 _flagArticulationTooFar = 0 ;

 _debug_Cinematique = debug;
}

Cinematique::~Cinematique(){
}

Matrix<6> Cinematique::getPositions(){
  return _positions;
}

Matrix<6> Cinematique::getArticulations(){
  return _articulations;
}

Matrix<4,4> Cinematique::getMatrixT(){
  return _matrixT;
}

void Cinematique::setPositions(Matrix<6> positions){
 _positions = {positions(0), positions(1), positions(2), positions(3), positions(4), positions(5)};
}

void Cinematique::setArticulations(Matrix<6> articulations){
 _articulations = {articulations(0), articulations(1), articulations(2), articulations(3), articulations(4), articulations(5),};
}

/*
* Set limit for angle
* int joint : numero de joint [1,2,3,4,5,6]
* float limitNeg: limite negative
* float limitPos: limite positive
*/
void Cinematique::setLimitAngle(int joint, float limitNeg, float limitPos){
 _limit[joint-1].pos = limitPos;
 _limit[joint-1].neg = limitNeg;
}

bool Cinematique::sanityCheckNAN(Matrix<6> articulations){
  _flagNanDetected = false;
  for (int i = 0; i <6 ; i++){
    if (isnan(articulations(i))){
      _flagNanDetected = true;
      Serial.println("Cinematique inverse failed - NAN detected");
      i = 6;
    }
  }
  return !_flagNanDetected;
}

/*
* Vérifie si les angles se trouvent dans les limites physiques du robot
* Matrix<6> articulations: Vecteur contenant les valeurs d'angle de chaque joint/moteur
* return: bool _flagArticulationTooFar
* 1 pour les articulations sont inclus dans les limites
* 0 pour au moins une articulation hors limite
*/
bool Cinematique::sanityCheckArticulations(Matrix<6> articulations){
  _flagArticulationTooFar = false;
  for (int i = 0; i <6 ; i++){
    if (articulations(i) < (_limit[i].neg + TRESHOLD)){
      _flagArticulationTooFar = true;
      i = 6;
      Serial.println("Cinematique inverse failed - Articulation is too far");
    }
    else if (articulations(i) > (_limit[i].pos - TRESHOLD)){
      _flagArticulationTooFar = true;
      i = 6;
      Serial.println("Cinematique inverse failed - Articulation is too far");
    } 
  }
  return !_flagArticulationTooFar;
}

void Cinematique::cinematiqueDirect(){
  _matrixT = {1,0,0,0,
              0,-1,0,0,
              0,0,-1,0,
              0,0,0,1};

  for (int i = 0; i<6 ; i++){
    Matrix<4,4> DH = {cosf(_articulations(i)+q(i)), -cosf(alpha(i))*sinf(_articulations(i)+q(i)),  sinf(alpha(i))*sinf(_articulations(i)+q(i)), a(i)*cosf(_articulations(i)+q(i)),
                      sinf(_articulations(i)+q(i)),  cosf(alpha(i))*cosf(_articulations(i)+q(i)), -sinf(alpha(i))*cosf(_articulations(i)+q(i)), a(i)*sinf(_articulations(i)+q(i)),
                      0,                        sinf(alpha(i)),                          cosf(alpha(i)),                         d(i),
                      0,                        0,                                       0,                                      1};
      
    _matrixT = _matrixT*DH;
    }

  Matrix<3> euler = CinematiqueAlgebra::getDCM2EulXYZ(_matrixT);

  _positions(0) = _matrixT(0,3);
  _positions(1) = _matrixT(1,3);
  _positions(2) = _matrixT(2,3);

  _positions(3) = euler(0);
  _positions(4) = euler(1);
  _positions(5) = euler(2);

  if(_debug_Cinematique){
        Serial.println("Articulations :");
    for(int i=0; i<6;i++){
      Serial.print(_articulations(i));
      Serial.print(" ");
    }
    Serial.println("\n");

    Serial.println("Matrix T :");
    for(int i=0; i<4;i++){
      for(int j=0; j<4; j++){
        Serial.print(_matrixT(i,j));
        Serial.print(" ");
      }
      Serial.println("");
    }
    Serial.println("");

    Serial.println("Positions :");
    for(int i=0; i<6;i++){
      Serial.print(_positions(i));
      Serial.print(" ");
    }
    Serial.println("");
  }
}

void Cinematique::cinematiqueInverse(){
 // Déterminer la matrice 4x4 T des positions
  Matrix<3,3> rot1 = CinematiqueAlgebra::getRotateX(_positions(3));
  Matrix<3,3> rot2 = CinematiqueAlgebra::getRotateY(_positions(4));
  Matrix<3,3> rot3 = CinematiqueAlgebra::getRotateZ(_positions(5));
  Matrix<3,3> Rdes  = rot1 * rot2 * rot3 ; 

  _matrixT_inv = { Rdes(0,0), Rdes(0,1), Rdes(0,2), _positions(0),
                   Rdes(1,0), Rdes(1,1), Rdes(1,2), _positions(1),
                   Rdes(2,0), Rdes(2,1), Rdes(2,2), _positions(2),
                   0,         0,         0,         1};


  // Déterminer le vecteur 3x1 Wdes correspondant à la position x,y,z désirée de l'épaule
  Matrix<3> Wdes;
  Wdes(0) = _positions(0) - D6*_matrixT_inv(0,2) ; 
  Wdes(1) = _positions(1) - D6*_matrixT_inv(1,2) ; 
  Wdes(2) = _positions(2) - D6*_matrixT_inv(2,2) ; 

  Matrix<3> theta123 = CinematiqueAlgebra::getAnthroRRR(Wdes) ;

  rot1 = CinematiqueAlgebra::getRotateZ(-theta123(0));
  rot2 = CinematiqueAlgebra::getRotateY(theta123(1));
  rot3 = CinematiqueAlgebra::getRotateY(theta123(2));
  Matrix<3,3> R123 = rot1 * rot2* rot3;
  
  Matrix<3,3> Res = {0, 0, 1,
                     0, -1, 0,
                     1, 0, 0};

  Matrix<3,3> R456 = ~R123 * Rdes * ~Res; 
  
  Matrix<3> theta456 = CinematiqueAlgebra::getDCM2EulXYX(R456);

  Matrix<6> articulations;
  articulations(0) = theta123(0);
  articulations(1) = theta123(1);
  articulations(2) = theta123(2);

  articulations(3) = theta456(0);
  articulations(4) = theta456(1);
  articulations(5) = theta456(2);

  
  if(sanityCheckNAN(articulations) && sanityCheckArticulations(articulations)){
    _articulations(0) = theta123(0);
    _articulations(1) = theta123(1);
    _articulations(2) = theta123(2);

    _articulations(3) = theta456(0);
    _articulations(4) = theta456(1);
    _articulations(5) = theta456(2);

    if(_debug_Cinematique){
      Serial.println("Positions :");
      for(int i=0; i<6;i++){
        Serial.print(_positions(i));
        Serial.print(" ");
      }
      Serial.println("\n");

    Serial.println("Matrix T :");
      for(int i=0; i<4;i++){
        for(int j=0; j<4; j++){
          Serial.print(_matrixT_inv(i,j));
          Serial.print(" ");
        }
        Serial.println("");
      }
      Serial.println("");

      Serial.println("Articulations :");
      for(int i=0; i<6;i++){
        Serial.print(_articulations(i));
        Serial.print(" ");
      }
      Serial.println("");
    }
  }
}