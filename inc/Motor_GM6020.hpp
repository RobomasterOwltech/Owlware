ls/*
 * Motor GM6020.h
 *
 *  Created on: Apr 12, 2024
 *      Author: @JorgePerC
 *
 * To know more about the motor, check:
 * https://github.com/Telpochcalli/wiki/blob/main/PDFs/RM_MTR_GM6020-Brushless_User-Guide.pdf
 */
#ifndef MotorGM6020_HPP
#define MotorGM6020_HPP

#include "IntfMotor.hpp"

    // ===== Operational ranges =====
    /*
     * speed value unit: rpm
     * send frequency: 1 KHz
     * TODO: CHECK TYPES ARE CORRECT
     * TODO: SET AS DEFINES RATHER THAN CONSTANTS
     */


class Motor_GM6020 : IntfMotor {
   private:
    // Resolution for position control:
    static const uint8_t maxAngle = 8191;
    // Resolution for velocity control:
    static const int16_t maxVoltage = 30000;
    static const int16_t minVoltage = -30000;
    // Resolution for torque control:
    static const float maxCurrent = 0.90;
    static const float minCurrent = 2.45;
    // Operational temperature (Celsius):
    static const int16_t maxTemp = 55;
    static const int16_t minTemp = 0;

    bool isCAN = false;
    uint8_t motorId;

    void initMotor (uint8_t motorID);

    // Update msg
    static uint8_t txBuff1[RM_DLC];
    static uint8_t txBuff2[RM_DLC];
    // TODO: THIS IS GOING TO RETURN A POINTER
    // MANAGE IT WISELY
    // DELETE IT AFTER USING IT
    static CAN_TxHeaderTypeDef* getTxHeader(uint8_t cntrlId);
   public:

    Motor_GM6020(TIM_HandleTypeDef htimer, OperationModes mode, uint8_t motorId);
    Motor_GM6020(ControllerCAN* hcan, OperationModes mode, uint8_t motorId);

    void setReference(float w);
    // Check that the motor can be controlled in that mode
    // also determines it based on it's controller
    void setControlType(OperationModes mode);
    void actuateVelocity();
    
    // TODO: DELETE THIS METHOD
    // This should move all motors at a time
    void moveCAN();


    ~Motor_GM6020();
};

#endif