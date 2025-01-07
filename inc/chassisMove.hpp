#ifndef CHASSIS_MOVE_HPP
#define CHASSIS_MOVE_HPP

#include <Eigen/Dense> 
#include "IntfMotor.hpp" 

#define CHASSIS_RADIUS 0.3f  // Radio del chasis (distancia del centro a una rueda) en metros
#define MAX_MOTOR_SPEED 1.0f // Velocidad máxima del motor

class chassisMove {
private:
    IntfMotor* leftFrontMotor;
    IntfMotor* rightFrontMotor;
    IntfMotor* leftBackMotor;
    IntfMotor* rightBackMotor;

    float maxMotorSpeed; 

    float normalizeSpeed(float speed);

public:
    chassisMove(IntfMotor* leftFrontMotor, IntfMotor* rightFrontMotor,
                IntfMotor* leftBackMotor, IntfMotor* rightBackMotor, 
                float maxMotorSpeed = MAX_MOTOR_SPEED);

    void joystickToMotors(float x, float y, float w);

    void stop();
};

#endif // CHASSIS_MOVE_HPP
