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

// Función para recibir las velocidades de los motores
void queueReceive() {
    float actualMotorSpeeds[4];
    if (xQueueReceive(motorSpeedQueue, &actualMotorSpeeds, portMAX_DELAY) == pdPASS) {
        for (int i = 0; i < 4; i++) {
            currentMotorSpeeds[i] = actualMotorSpeeds[i]; // Almacena las velocidades actuales
        }
    } else {
        printf("Error.\n");
    }
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
 * @param theta_joy Ángulo de orientación del joystick2 (en radianes).
 */
void chassisMove::joystickToMotors(float x1, float y1, float x2, float y2) {
    // Cálculo del ángulo deseado 
    float w=atan2(y2, x2);

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

        // Restar las velocidades actuales (obtenidas de la cola)
    Eigen::VectorXf adjusted_speed(4);
    for (int i = 0; i < 4; i++) {
        adjusted_speed[i] = currentMotorSpeeds[i]-wheel_speed[i] ;
    }

    leftFrontMotor->actuate(adjusted_speed[0]);   // Delantera izquierda
    rightFrontMotor->actuate(adjusted_speed[1]);  // Delantera derecha
    rightBackMotor->actuate(adjusted_speed[2]);   // Trasera derecha
    leftBackMotor->actuate(adjusted_speed[3]);    // Trasera izquierda
}

void chassisMove::stop() {
    leftFrontMotor->stop(0);
    rightFrontMotor->stop(0);
    leftBackMotor->stop(0);
    rightBackMotor->stop(0);
}
