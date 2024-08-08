/*
 * Motor 2305.h
 *
 *  Created on: Apr 12, 2024
 *      Author: @JorgePerC
 *
 * To know more about the motor, check:
 * https://github.com/Telpochcalli/wiki/blob/main/PDFs/RM_MTR_2305-Brushless_User-Guide.pdf
 */
#ifndef Motor2305_HPP
#define Motor2305_HPP

#include "IntfMotor.hpp"

class Motor_2305 : IntfMotor {
   private:
    // ===== Operational ranges =====
    /*
     * speed value unit: rpm
     * send frequency: 1 KHz
     */
    // Resolution for position control:
    static const uint8_t maxAngle = NAN;
    // Resolution for velocity control:
    static const int16_t maxVoltage = 17.44;
    static const int16_t minVoltage = -17.44;
    // Resolution for torque control:
    static const float maxCurrent = 30;
    static const float minCurrent = 0.4;
    // Operational temperature (Celsius):
    static const int16_t maxTemp = 40;
    static const int16_t minTemp = -10;

    static const uint8_t cntrlId = CONTROL_ID_A;

   public:
    Motor_2305(TIM_HandleTypeDef htimer, OperationModes mode, uint8_t motorId);
    Motor_2305(CAN_HandleTypeDef hcan, OperationModes mode, uint8_t motorId);

    void setReference(float w);
    void setControlType(OperationModes mode);

    ~Motor_2305();
};

#endif