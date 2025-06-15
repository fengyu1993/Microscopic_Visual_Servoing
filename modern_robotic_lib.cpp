#include "modern_robotic_lib.h"

MatrixXd Adjoint(Matrix4d T)
{
    MatrixXd Ad(6, 6);
    Ad = MatrixXd::Zero(6, 6);
    Matrix3d R; Vector3d p;
    TransToRp(T, R, p);
    Ad.block<3, 3>(0, 0) = R;
    Ad.block<3, 3>(3, 0) = VecToso3(p) * R;
    Ad.block<3, 3>(3, 3) = R;
    return Ad;
}

Matrix4d VecTose3(VectorXd V)
{
    Matrix4d se3 = MatrixXd::Zero(4, 4);
    Matrix3d so3 = VecToso3(V.block<3, 1>(0, 0));
    se3.block<3, 4>(0, 0) << so3, V.block<3, 1>(3, 0);
    return se3;
}


Matrix4d MatrixExp6(Matrix4d se3mat)
{
    Matrix4d T = MatrixXd::Identity(4, 4);
    Matrix3d  so3mat = se3mat.block<3, 3>(0, 0);
    Vector3d omgtheta = so3ToVec(so3mat);
    if (NearZear(omgtheta.norm()))
        T.block<3, 1>(0, 3) = se3mat.block<3, 1>(0, 3);
    else {
        Vector3d omghat; double theta;
        AxisAng3(omgtheta, omghat, theta);
        Matrix3d omgmat = so3mat / theta;
        T.block<3, 3>(0, 0) << MatrixExp3(so3mat);
        T.block<3, 1>(0, 3) << (MatrixXd::Identity(3, 3) * theta + (1 - cos(theta)) * omgmat + (theta - sin(theta)) * omgmat * omgmat) * se3mat.block<3, 1>(0, 3) / theta;
    }
    return T;
}

Matrix3d VecToso3(Vector3d omg)
{
    Matrix3d so3 = Matrix3d::Zero(3, 3);
    so3(0, 1) = -omg(2);   so3(0, 2) = omg(1);
    so3(1, 0) = omg(2);    so3(1, 2) = -omg(0);
    so3(2, 0) = -omg(1);   so3(2, 1) = omg(0);
    return so3;
}

void TransToRp(Matrix4d T, Matrix3d& R, Vector3d& p)
{
    R << T.block<3, 3>(0, 0);
    p << T.block<3, 1>(0, 3);
}

Vector3d so3ToVec(Matrix3d  omg)
{
    Vector3d v(omg(2, 1), omg(0, 2), omg(1, 0));
    return v;
}

bool NearZear(double z)
{
    return abs(z) < 1e-6;
}

void AxisAng3(Vector3d expc3, Vector3d& unitV, double& theta)
{
    unitV = Normalize(expc3);
    theta = expc3.norm();
}

Vector3d Normalize(Vector3d  V)
{
    return V / V.norm();
}

Matrix3d MatrixExp3(Matrix3d so3mat)
{
    Vector3d omgtheta = so3ToVec(so3mat);
    if (NearZear(omgtheta.norm()))
        return Matrix3d::Identity();
    else {
        double theta = omgtheta.norm();
        Matrix3d omgmat = so3mat / theta;
        return  Matrix3d::Identity() + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
    }
}
