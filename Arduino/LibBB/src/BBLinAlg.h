#if !defined(BBLINALG_H)
#define BBLINALG_H

#include <BasicLinearAlgebra.h>

namespace bb {
    BLA::Matrix<3,3> eulerToRot(float r, float p, float h);
    void rotToEuler(const BLA::Matrix<3, 3>& A, float &r, float &p, float &h);

    void transformRotation(float rIn, float pIn, float hIn, float rXf, float pXf, float hXf,
                           float& rOut, float& pOut, float& hOut, bool inverse);
    void transformVector(float xIn, float yIn, float zIn, float rXf, float pXf, float hXf,
                         float &xOut, float &yOut, float& zOut, bool inverse);
};

#endif // BBLINALG_H