#include "kssekf/kssekf.hpp"

EKF::EKF() {
    state_.position.setZero();
    state_.velocity.setZero();
    state_.orientation.setIdentity();
    state_.gyro_bias.setZero();

    P_.setIdentity();

    P_.block<3,3>(0,0) *= 1e-2;   // position
    P_.block<3,3>(3,3) *= 1e-1;   // velocity
    P_.block<3,3>(6,6) *= 1e-2;   // orientation
    P_.block<3,3>(10,10) *= 1e-4; // gyro bias



    Q_.setIdentity();
    Q_ *= 1e-3;

    R_.setIdentity();
    R_ *= 1e-2;
}

Quaterniond EKF::integrateQuat(const Quaterniond& q, const Vector3d& omega, double dt) {
    Vector3d theta = omega * dt;
    double angle = theta.norm();

    Quaterniond dq ;

    if (angle < 1e-6) {
        // Small Angle Estmate: dq ≈ [1, 0.5 * theta]
        dq.w() = 1.0;
        dq.vec() = 0.5 * theta;
    } else {
        dq = Quaterniond(AngleAxisd(angle, theta.normalized()));
    }

    // Quaterniond dq = Quaterniond(AngleAxisd(angle, theta.normalized()));
    dq.normalize();

    return (q * dq).normalized();
}

void EKF::predict(const Vector3d& odom_velocity, const Vector3d& gyro, double dt) 
{
    // ===== 1. 상태 예측 =====
    // (1) 속도 예측 (오도메트리 속도는 body frame)
    Vector3d v_world = state_.orientation * odom_velocity;

    // (2) 위치 갱신
    state_.position += v_world * dt;

    // (3) 속도 갱신
    state_.velocity = v_world;

    // (4) orientation 예측 (yaw 회전만 고려)
    // Vector3d omega(0.0, 0.0, omega_z);

    Vector3d omega = gyro - state_.gyro_bias;

    // omega -= state_.gyro_bias; // 바이어스 보정

    Quaterniond dq = integrateQuat(Quaterniond::Identity(), omega, dt);
    state_.orientation = (state_.orientation * dq).normalized();

    // ===== 2. 공분산 예측 =====

    // 상태전이 자코비안 F (13x13)
    Matrix<double,13,13> F = Matrix<double,13,13>::Identity();

    // (1) 위치 w.r.t orientation
    Matrix3d R = state_.orientation.toRotationMatrix();
    Matrix3d v_skew;
    v_skew <<             0, -v_world.z(),  v_world.y(),
                      v_world.z(),       0, -v_world.x(),
                     -v_world.y(),  v_world.x(),        0;

    F.block<3,3>(0,3) = Matrix3d::Identity() * dt;               // pos ← vel
    F.block<3,3>(0,6) = -R * v_skew * dt;                        // pos ← orientation

    // (2) orientation 부분 (yaw 회전만 간주)
    Matrix3d omega_skew;
    omega_skew <<      0,        -omega.z(),  omega.y(),
                omega.z(),         0,      -omega.x(),
                -omega.y(),    omega.x(),         0;
    F.block<3,3>(6,6) += -omega_skew * dt; // rot ← rot

    // Eigen::Matrix3d F_rot = Sophus::SO3d::exp(-omega * dt).matrix();
    // F.block<3,3>(6,6) = F_rot;
    // F.block<3,3>(6,10) = -state_.orientation.toRotationMatrix() * dt;  // rot ← bias

    // (3) bias 변화 없음 → 항등

    // G: 잡음 자코비안 (13x13 간단화)
    Matrix<double,13,13> G = Matrix<double,13,13>::Identity();

    // P 예측
    P_ = F * P_ * F.transpose() + G * Q_ * G.transpose();



}

void EKF::updateIMU(const Vector3d& accel, const Vector3d& gyro) {
    // 측정 예상값
    Vector3d gravity(0, 0, 9.81);
    Vector3d accel_pred = state_.orientation.inverse() * gravity * -1;
    Vector3d gyro_pred = gyro - state_.gyro_bias;

    // 측정값과 예측값
    VectorXd z(6), z_pred(6), y(6);
    z << accel, gyro;
    z_pred << accel_pred, gyro_pred;
    y = z - z_pred;

    
    // 측정 자코비안 H (6x13)
    Matrix<double, 6, 13> H = Matrix<double, 6, 13>::Zero();

    // -------------------------------
    // Orientation Jacobian for accel
    // accel_pred = R^T * g
    // d(accel_pred)/d(θ) = -R^T * [g]_x
    Matrix3d R = state_.orientation.toRotationMatrix();
    // Matrix3d g_skew;
    // g_skew <<         0,  9.81,     0,
    //               -9.81,     0,     0,
    //                   0,     0,     0;


    Matrix3d g_skew;
    g_skew <<         0,  -gravity.z(),     gravity.y(),
            gravity.z(),        0,    -gravity.x(),
            -gravity.y(),  gravity.x(),        0;


    H.block<3,3>(0,6) = -R.transpose() * g_skew;

    // -------------------------------
    // Gyro bias Jacobian
    H.block<3,3>(3,10) = -Matrix3d::Identity();

    // 칼만 이득
    Matrix<double, 6, 6> S = H * P_ * H.transpose() + R_;
    Matrix<double, 13, 6> K = P_ * H.transpose() * S.inverse();

    // 상태 보정량
    VectorXd dx = K * y;

    // 상태 업데이트 
    state_.position += dx.segment<3>(0);
    state_.velocity += dx.segment<3>(3);

    // ---- Nonlinear quaternion correction ----
    Vector3d dtheta = dx.segment<3>(6);
    double angle = dtheta.norm();

    // if (angle > 0.2) {
    //     dtheta *= 0.2 / angle;  // Limit max correction to ~11 degrees
    //     angle = 0.2;
    // }

    Quaterniond dq;
    if (angle > 1e-5)
        dq = Quaterniond(AngleAxisd(angle, dtheta.normalized()));
    else
        dq = Quaterniond(1, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z()).normalized();

    dq.normalize();
    // orientation 업데이트
    state_.orientation = (state_.orientation * dq).normalized();

    state_.gyro_bias += dx.segment<3>(10);

    // 공분산 갱신
    P_ = (Matrix<double,13,13>::Identity() - K * H) * P_;
}


double getYawFromQuaternion(const Eigen::Quaterniond& q) {
    // Eigen: ZYX (yaw → pitch → roll)
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // yaw, pitch, roll
    return euler[0]; // yaw
}

// void EKF::updateIMU(const Vector3d& accel, const Vector3d& gyro) {
//     // 측정 예상값
//     Vector3d gravity(0, 0, -9.81);
//     Vector3d accel_pred = state_.orientation.inverse() * gravity;
//     Vector3d gyro_pred = gyro - state_.gyro_bias;

//     // 측정값과 예측값
//     VectorXd z(6), z_pred(6), y(6);
//     z << accel, gyro;
//     z_pred << accel_pred, gyro_pred;
//     y = z - z_pred;

//     // 잔차 크기 제한 (임계값)
//     const double residual_threshold = 3.0; // 필요에 따라 조절
//     if (y.norm() > residual_threshold) {
//         // 잔차가 너무 크면 업데이트 스킵 (스파이크 방지)
//         #ifdef EKF_DEBUG_MODE
//             RCLCPP_WARN(this->logger, "IMU residual too large: %f, skipping update", y.norm());
//         #endif
//         return;
//     }

//     // 측정 자코비안 H (6x13)
//     Matrix<double, 6, 13> H = Matrix<double, 6, 13>::Zero();

//     // Orientation Jacobian for accel
//     Matrix3d R = state_.orientation.toRotationMatrix();

//     Matrix3d g_skew;
//     g_skew <<         0,  -gravity.z(),     gravity.y(),
//             gravity.z(),        0,    -gravity.x(),
//             -gravity.y(),  gravity.x(),        0;

//     H.block<3,3>(0,6) = -R.transpose() * g_skew;

//     // Gyro bias Jacobian
//     H.block<3,3>(3,10) = -Matrix3d::Identity();

//     // 칼만 이득 계산
//     Matrix<double, 6, 6> S = H * P_ * H.transpose() + R_;
//     Matrix<double, 13, 6> K = P_ * H.transpose() * S.inverse();

//     // 상태 보정량
//     VectorXd dx = K * y;

//     // 상태 업데이트
//     state_.position += dx.segment<3>(0);
//     state_.velocity += dx.segment<3>(3);

//     // 쿼터니언 보정 (회전 벡터)
//     Vector3d dtheta = dx.segment<3>(6);
//     double angle = dtheta.norm();

//     // 각도 제한 (예: 0.1 rad = 약 5.7도)
//         const double max_angle = 0.1;
//         if (angle > max_angle) {
//             dtheta = dtheta.normalized() * max_angle;
//             angle = max_angle;
//         }

//     Quaterniond dq;
//     if (angle > 1e-5)
//         dq = Quaterniond(AngleAxisd(angle, dtheta.normalized()));
//     else
//         dq = Quaterniond(1, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z()).normalized();

//     dq.normalize();

//     // 쿼터니언 정상성 검사
//     if (!dq.coeffs().allFinite()) {
//         #ifdef EKF_DEBUG_MODE
//             RCLCPP_WARN(this->logger, "Non-finite quaternion correction detected, skipping update");
//         #endif
//         return;
//     }

//     state_.orientation = (state_.orientation * dq).normalized();

//     state_.gyro_bias += dx.segment<3>(10);

//     // 공분산 갱신
//     P_ = (Matrix<double,13,13>::Identity() - K * H) * P_;
// }


