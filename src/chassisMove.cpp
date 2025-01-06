#include "chassisMove.hpp"

// Constructor
chassisMove::chassisMove(IntfMotor* leftFrontMotor, IntfMotor* rightFrontMotor,
                         IntfMotor* leftBackMotor, IntfMotor* rightBackMotor, float maxMotorSpeed)
    : leftFrontMotor(leftFrontMotor), rightFrontMotor(rightFrontMotor),
      leftBackMotor(leftBackMotor), rightBackMotor(rightBackMotor), maxMotorSpeed(maxMotorSpeed) {}

// Normalizar la velocidad para que esté dentro del rango permitido
float chassisMove::normalizeSpeed(float speed) {
    if (speed > maxMotorSpeed) return maxMotorSpeed;
    if (speed < -maxMotorSpeed) return -maxMotorSpeed;
    return speed;
}

// Traducir joystick a velocidades de las ruedas
void chassisMove::joystickToMotors(float x, float y, float w) {
    float wheel_speed[4];

    // Calcular velocidades de las ruedas mecanum
    wheel_speed[0] = -x - y + w; // Delantera izquierda
    wheel_speed[1] = x - y + w;  // Delantera derecha
    wheel_speed[2] = x + y + w;  // Trasera derecha
    wheel_speed[3] = -x + y + w; // Trasera izquierda

    // Normalizar y actuar las velocidades en los motores
    leftFrontMotor->actuate(normalizeSpeed(wheel_speed[0]));
    rightFrontMotor->actuate(normalizeSpeed(wheel_speed[1]));
    rightBackMotor->actuate(normalizeSpeed(wheel_speed[2]));
    leftBackMotor->actuate(normalizeSpeed(wheel_speed[3]));
}

void getJoystickRotation(){
    w = (-wheel_speed[0] - wheel_speed[1] - wheel_speed[2] - wheel_speed[3]) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
}

// Actualizar el movimiento basado en joystick
void chassisMove::update() {
    float x = getJoystickX();
    float y = getJoystickY();
    float w = getJoystickRotation();

    joystickToMotors(x, y, w);
}

void chassisMove::stop() {
    leftMotor->stop(0); 
    rightMotor->stop(0); 
    leftBackMotor->stop(0); 
    rightBackMotor->stop(0); 
}
