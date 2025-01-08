#include "chassisMove.hpp"

/**
 * @brief Normaliza la velocidad del motor para asegurarse de que no exceda la velocidad máxima.
 * 
 * @param speed Velocidad a normalizar.
 * @return La velocidad normalizada.
*/
float chassisMove::normalizeSpeed(float speed) {
    if (speed > maxMotorSpeed_rpm) return maxMotorSpeed_rpm;
    if (speed < -maxMotorSpeed_rpm) return -maxMotorSpeed_rpm;
    return speed;
}

/**
 * @brief Normaliza la velocidad angular (torsión) en función de los valores de entrada del joystick.
 * 
 * @param theta_joy Ángulo del joystick que indica la dirección de la torsión.
 * @param theta_robot Ángulo actual del robot.
 * @return La velocidad angular normalizada en radianes por segundo.
 */
float chassisMove::normalizeW(float theta_joy_rads) {
    float angle_error = theta_joy_rads - theta_robot_rads;
    if (angle_error > M_PI) angle_error -= 2 * PI;
    if (angle_error < -M_PI) angle_error += 2 * PI;
    float w_rs = K_TWIST * angle_error;
    //TODO: si se actualiza?
    //theta_robot_rads=w_rs; 
    return w_rs;
}

chassisMove::chassisMove(IntfMotor* leftFrontMotor, IntfMotor* rightFrontMotor,
                         IntfMotor* leftBackMotor, IntfMotor* rightBackMotor, 
                         float maxMotorSpeed)
    : leftFrontMotor(leftFrontMotor), rightFrontMotor(rightFrontMotor),
      leftBackMotor(leftBackMotor), rightBackMotor(rightBackMotor),
      maxMotorSpeed(maxMotorSpeed_rpm) {}

/**
 * @brief Convierte las entradas de los joysticks en velocidades de los motores.
 * 
 * Este método toma las entradas de los dos joysticks (para movimiento y torsión) y las convierte en 
 * velocidades para cada rueda del chasis mecanum.
 * 
 * @param x1 Entrada del joystick 1 (eje X para desplazamiento en el plano horizontal).
 * @param y1 Entrada del joystick 1 (eje Y para desplazamiento en el plano vertical).
 * @param x2 Entrada del joystick 2 (eje X para control de torsión).
 * @param y2 Entrada del joystick 2 (eje Y para control de torsión).
 * @param theta_robot Ángulo de orientación del robot (en radianes).
 * @param theta_joy Ángulo de orientación del joystick2 (en radianes).
 */
void chassisMove::joystickToMotors(float x1, float y1, float x2, float y2) {
    // Cálculo del ángulo deseado 
    float theta_joy_rads = atan2(y2, x2);
    // Cálculo de la torsión (velocidad angular)
    float w_rs = normalizeW(theta_joy_rads)

    // u
    Eigen::Vector3f joystick_input(x1, y1, w);
    // M
    Eigen::MatrixXf control_matrix(4, 3);
    control_matrix << -1, -1,  CHASSIS_RADIUS,  // Delantera izquierda
                       1, -1,  CHASSIS_RADIUS,  // Delantera derecha
                       1,  1,  CHASSIS_RADIUS,  // Trasera derecha
                      -1,  1,  CHASSIS_RADIUS;  // Trasera izquierda

    //v=M*u
    Eigen::VectorXf wheel_speed = control_matrix * joystick_input;
    wheel_speed = wheel_speed.unaryExpr([this](float speed) { return normalizeSpeed(speed); });

    leftFrontMotor->actuate(wheel_speed[0]);   // Delantera izquierda
    rightFrontMotor->actuate(wheel_speed[1]);  // Delantera derecha
    rightBackMotor->actuate(wheel_speed[2]);   // Trasera derecha
    leftBackMotor->actuate(wheel_speed[3]);    // Trasera izquierda  

    //PARTE ANA - leer referencia de velovidad en el IMU
    //postitionr1... es la velocidad detectada en el imu para cada uno de los motores. 
    //planeados1... son la velocidades calculadas en la matriz

    float m1= positionr1-planeado1;
    float m2= positionr2-planeado2;
    float m3= positionr3-planeado3;
    float m4= positionr4-planeado4;

    leftFrontMotor->actuate(wheel_speed[0]+m1);   // Delantera izquierda
    rightFrontMotor->actuate(wheel_speed[1]+m2);  // Delantera derecha
    rightBackMotor->actuate(wheel_speed[2]+m3);   // Trasera derecha
    leftBackMotor->actuate(wheel_speed[3]+m4);    // Trasera izquierda
    
}


void chassisMove::stop() {
    leftFrontMotor->stop(0);
    rightFrontMotor->stop(0);
    leftBackMotor->stop(0);
    rightBackMotor->stop(0);
}
