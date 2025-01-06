#ifndef CHASSIS_MOVE_HPP
#define CHASSIS_MOVE_HPP

#include "IntfMotor.hpp"

#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ
#define MOTOR_DISTANCE_TO_CENTER


class chassisMove {
private:
    IntfMotor* leftFrontMotor;
    IntfMotor* rightFrontMotor;
    IntfMotor* leftBackMotor;
    IntfMotor* rightBackMotor;

    float maxMotorSpeed; 

    float normalizeSpeed(float speed);

public:
    // Constructor
    chassisMove(IntfMotor* leftFrontMotor, IntfMotor* rightFrontMotor,
                IntfMotor* leftBackMotor, IntfMotor* rightBackMotor, float maxMotorSpeed = 100.0f);

    // Traducir joystick a movimiento de motores
    void joystickToMotors(float x, float y, float w);

    // Método para actualizar las velocidades basándose en el joystick
    void update();

    //Metodo para detener motores 
    void stop();
};

#endif /* CHASSIS_MOVE_HPP */
