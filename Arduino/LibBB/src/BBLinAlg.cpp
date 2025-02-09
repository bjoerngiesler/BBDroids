#include "BBLinAlg.h"
#include "LibBB.h"
#include "BBConsole.h"

BLA::Matrix<3,3> bb::eulerToRot(float p, float r, float h) {
  BLA::Matrix<3,3> Ax, Ay, Az;

  Ax.Fill(0);
  Ax(0, 0) = 1; Ax(0, 1) = 0;                 Ax(0, 2) = 0;
  Ax(1, 0) = 0; Ax(1, 1) = cos(DEG_TO_RAD*p); Ax(1, 2) = -sin(DEG_TO_RAD*p);
  Ax(2, 0) = 0; Ax(2, 1) = sin(DEG_TO_RAD*p); Ax(2, 2) = cos(DEG_TO_RAD*p);

  Ay.Fill(0);
  Ay(0, 0) = cos(DEG_TO_RAD*r);  Ay(0, 1) = 0; Ay(0, 2) = sin(DEG_TO_RAD*r);
  Ay(1, 0) = 0;                  Ay(1, 1) = 1; Ay(1, 2) = 0;
  Ay(2, 0) = -sin(DEG_TO_RAD*r); Ay(2, 1) = 0; Ay(2, 2) = cos(DEG_TO_RAD*r);

  Az.Fill(0);
  Az(0, 0) = cos(DEG_TO_RAD*h); Az(0, 1) = -sin(DEG_TO_RAD*h); Az(0, 2) = 0;
  Az(1, 0) = sin(DEG_TO_RAD*h); Az(1, 1) = cos(DEG_TO_RAD*h);  Az(1, 2) = 0;
  Az(2, 0) = 0;                 Az(2, 1) = 0;                  Az(2, 2) = 1;

  return Az*Ay*Ax;
}

void bb::rotToEuler(const BLA::Matrix<3, 3>& A, float &p, float &r, float &h) {
  if(!EPSILON(fabs(A(2, 0))-1.0)) {
    float p1 = -asin(A(2, 0));
    float p2 = M_PI - p1;
    float r1 = atan2(A(2,1)/cos(p1), A(2,2)/cos(p1));
    float r2 = atan2(A(2,1)/cos(p2), A(2,2)/cos(p2));
    float h1 = atan2(A(1,0)/cos(p1), A(0,0)/cos(p1));
    float h2 = atan2(A(1,0)/cos(p2), A(0,0)/cos(p2));
    
    // make compiler happy
    (void) r2;
    (void) h2;

    r = RAD_TO_DEG*r1; p = RAD_TO_DEG*p1; h = RAD_TO_DEG*h1;
  } else {
    h = 0;
    if(EPSILON(fabs(A(2,0))+1)) {
      p = RAD_TO_DEG*M_PI_2;
      r = RAD_TO_DEG*atan2(A(0,1), A(0,2));
    } else {
      p = RAD_TO_DEG*-M_PI_2;
      r = RAD_TO_DEG*atan2(-A(0,1), -A(0,2));
    }
  }
}

void bb::transformRotation(float pIn, float rIn, float hIn, float pXf, float rXf, float hXf,
                           float& rOut, float& pOut, float& hOut, bool inverse) {
  BLA::Matrix<3,3> A, B, C;

  A = eulerToRot(pIn, rIn, hIn);
  B = eulerToRot(pXf, rXf, hXf);
  if(inverse) B = BLA::Inverse(B);
  C = B*A;
  float r, p, h;
  rotToEuler(C, r, p, h);
  rOut = r; pOut = p; hOut = h;
}

void bb::transformVector(float xIn, float yIn, float zIn, float pXf, float rXf, float hXf,
                         float &xOut, float &yOut, float& zOut, bool inverse) {
    BLA::Matrix<3,3> B; 
    BLA::Matrix<3> a, c;

    a(0) = xIn; a(1) = yIn; a(2) = zIn;
    B = eulerToRot(pXf, rXf, hXf);
    if(inverse) B = BLA::Inverse(B);
    c = B * a;

    xOut = c(0); yOut = c(1); zOut = c(2);
}
