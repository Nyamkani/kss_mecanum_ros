#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

struct State {
    Vector3d position;
    Vector3d velocity;
    Quaterniond orientation;
    Vector3d gyro_bias;
};

class EKF {
public:
    EKF();

    void predict(const Vector3d& odom_velocity, const Vector3d& gyro, double dt);
    void updateIMU(const Vector3d& accel, const Vector3d& gyro);

    State getState() const { return state_; }

private:
    State state_;
    Matrix<double, 13, 13> P_; // Covariance matrix (13x13: pos(3)+vel(3)+quat(4)+bias(3))

    // Process and measurement noise
    Matrix<double, 13, 13> Q_;
    Matrix<double, 6, 6> R_; // accel (3) + gyro (3)

    Quaterniond integrateQuat(const Quaterniond& q, const Vector3d& omega, double dt);
};


double getYawFromQuaternion(const Eigen::Quaterniond& q);

