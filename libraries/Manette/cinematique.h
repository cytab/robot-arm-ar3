#ifndef _CINEMATIQUE_H
#define _CINEMATIQUE_H

#include "Arduino.h"
#include <math.h>
#include "BasicLinearAlgebra.h"    // BasicLinearAlgebra by Tom Stewart for Matrix

using namespace std;
using namespace BLA;

// Mesure du AR3
#define D1 151.269f
#define A1  64.201f
#define A2 250.000f
#define D4 221.341f
#define D6 220.900f

//sanity
#define TRESHOLD 0.02f

//type de solution 
#define  EPAULE_DROITE_COUDE_BAS   00
#define  EPAULE_DROITE_COUDE_HAUT  01
#define  EPAULE_GAUCHE_COUDE_BAS   10
#define  EPAULE_GAUCHE_COUDE_HAUT  11

//limite d'Angle pour un joint
struct LimitJoint
{
  float pos;
  float neg;
};




class CinematiqueAlgebra{
public:
    CinematiqueAlgebra();
    ~CinematiqueAlgebra();

    Matrix<3,3> getRotateX(float angle);
    Matrix<3,3> getRotateY(float angle);
    Matrix<3,3> getRotateZ(float angle);

    Matrix<3> getDCM2EulXYZ(Matrix<3,3> matrixR);
    Matrix<3> getDCM2EulXYZ(Matrix<4,4> matrixT);

    Matrix<3> getDCM2EulXYX(Matrix<3,3> matrixR);

    Matrix<3> getAnthroRRR(Matrix<3> Wdes);

    float wrapAngle(float angle_rad);

  protected:

};


class Cinematique: public CinematiqueAlgebra{
public:
    Cinematique();
    Cinematique(bool debug);
    ~Cinematique();
    
    Matrix<6> getPositions();
    Matrix<6> getArticulations();
    Matrix<4,4> getMatrixT();

    void setPositions(Matrix<6> positions);
    void setArticulations(Matrix<6> articulations);
    // for debug purpose *never use this function otherwise*//
    void setLimitAngle(int joint, float limitNeg, float limitPos);

    bool sanityCheckNAN(Matrix<6> articulations);
    bool sanityCheckArticulations(Matrix<6> articulations);

    void cinematiqueDirect();
    void cinematiqueInverse();

  protected:
    // Paramètre DH
    const Matrix<6> a = {64.201, 250, 0, 0, 0, 0};
    const Matrix<6> alpha = {M_PI/2.0, 0, -M_PI/2.0, M_PI/2.0, -M_PI/2.0, 0}; 
    const Matrix<6> d = {-151.269, 0, 0, 221.341, 0, 220.9};
    const Matrix<6> q = {0, 0, -M_PI/2.0, 0, 0, 0}; 
   
    Matrix<6> _positions;
    Matrix<6> _articulations;

    Matrix<4,4> _matrixT;
    Matrix<4,4> _matrixT_inv;

    LimitJoint _limit[6]; 
    bool _flagNanDetected;
    bool _flagArticulationTooFar;

    bool _debug_Cinematique;
};

#endif // _CINEMATIQUE_H
