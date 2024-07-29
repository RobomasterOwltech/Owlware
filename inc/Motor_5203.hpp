/*
 * Motor 5203.h
 *
 *  Created on: Apr 12, 2024
 *      Author: @JorgePerC
 *
 * To know more about the motor, check:
 * https://github.com/Telpochcalli/wiki/blob/main/PDFs/RM_MTR_5203-Brushless_User-Guide.pdf
 */
#ifndef Motor5203_HPP
#define Motor5203_HPP

#include "BaseMotor.hpp"

class Motor_5203 : BaseMotor {
   private:
    // ===== Operational ranges =====
    /*
     * speed value unit: rpm
     * send frequency: 1 KHz
     */
    // Resolution for position control:
    static const uint8_t maxAngle = NAN;
    // Resolution for velocity control:
    static const int16_t maxVoltage = 32;
    static const int16_t minVoltage = -32;
    // Resolution for torque control:
    static const float maxCurrent = 9.2;
    static const float minCurrent = 0.25;
    // Operational temperature (Celsius):
    static const int16_t maxTemp = NAN;
    static const int16_t minTemp = NAN;

    static const uint8_t cntrlId = CONTROL_ID_A;

   public:
    Motor_5203(TIM_HandleTypeDef htimer, OperationModes mode, uint8_t motorId);
    Motor_5203(CAN_HandleTypeDef hcan, OperationModes mode, uint8_t motorId);

    void setReference(float w);
    void setControlType(OperationModes mode);

    ~Motor_5203();
};

#endif