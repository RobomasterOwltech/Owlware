#include "chassisMove.hpp"

float chassisMove::normalizeSpeed(float speed) {
    if (speed > maxMotorSpeed) return maxMotorSpeed;
    if (speed < -maxMotorSpeed) return -maxMotorSpeed;
    return speed;
}

chassisMove::chassisMove(IntfMotor* leftFrontMotor, IntfMotor* rightFrontMotor,
                         IntfMotor* leftBackMotor, IntfMotor* rightBackMotor, 
                         float maxMotorSpeed)
    : leftFrontMotor(leftFrontMotor), rightFrontMotor(rightFrontMotor),
      leftBackMotor(leftBackMotor), rightBackMotor(rightBackMotor),
      maxMotorSpeed(maxMotorSpeed) {}


void chassisMove::joystickToMotors(float x, float y, float w) {
    //v=M*u
    // u
    Eigen::Vector3f joystick_input(x, y, w);

    // M
    Eigen::MatrixXf control_matrix(4, 3);
    control_matrix << -1, -1,  CHASSIS_RADIUS,  // Delantera izquierda
                       1, -1,  CHASSIS_RADIUS,  // Delantera derecha
                       1,  1,  CHASSIS_RADIUS,  // Trasera derecha
                      -1,  1,  CHASSIS_RADIUS;  // Trasera izquierda

    // v
    Eigen::VectorXf wheel_speed = control_matrix * joystick_input;
    wheel_speed = wheel_speed.unaryExpr([this](float speed) { return normalizeSpeed(speed); });

    leftFrontMotor->actuate(wheel_speed[0]);
    rightFrontMotor->actuate(wheel_speed[1]);
    rightBackMotor->actuate(wheel_speed[2]);
    leftBackMotor->actuate(wheel_speed[3]);
}

void chassisMove::stop() {
    leftFrontMotor->stop(0);
    rightFrontMotor->stop(0);
    leftBackMotor->stop(0);
    rightBackMotor->stop(0);
}
