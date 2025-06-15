#ifndef MODERN_ROBOTIC_LIB_H
#define MODERN_ROBOTIC_LIB_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;

    MatrixXd Adjoint(Matrix4d T);
    Matrix4d VecTose3(VectorXd V);
    Matrix4d MatrixExp6(Matrix4d se3mat);
    Matrix3d VecToso3(Vector3d omg);
    void TransToRp(Matrix4d T, Matrix3d& R, Vector3d& p);
    Vector3d so3ToVec(Matrix3d  omg);
    bool NearZear(double z);
    void AxisAng3(Vector3d expc3, Vector3d& unitV, double& theta);
    Vector3d  Normalize(Vector3d  V);
    Matrix3d MatrixExp3(Matrix3d so3mat);


#endif // MODERN_ROBOTIC_LIB_H
