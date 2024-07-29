/*
 * BaseMotor.hpp
 *
 *  Created on: Apr 10, 2024
 *      Author: @JorgePerC
 * This is based on the following:
 * https://github.com/STMicroelectronics/STM32CubeF4/blob/master/Projects/STM324xG_EVAL/Examples/CAN/CAN_Networking/Src/main.c
 */
#ifndef BaseMotor_HPP
#define BaseMotor_HPP

#include "ControllerCAN.hpp"
#include "ControllerPWM.hpp"

typedef enum { POS, VEL, TOR } OperationModes;

typedef struct {
    float maxVelocity;
    float minVelocity;
    float maxTorque;
    float minTorque;
    float maxPosition;
    float minPosition;
    float maxTemp;
    float minTemp;
} OperationalRanges;

typedef union {
    ControllerCAN* can;
    ControllerPWM* pwm;
} Controller;

//TODO: Create a new union for selecting the ID
// bc it can be a channel, or for can
class BaseMotor {
   protected:
    Controller contr;
    OperationalRanges* attr;

    uint8_t mode;
    uint8_t dir;
    uint16_t runFreq;

    virtual void actuateVelocity();
    virtual void actuatePosition();
    virtual void actuateTorque();

   public:
    BaseMotor();
    BaseMotor(ControllerCAN* controller, OperationalRanges* attr, OperationModes mode, uint8_t direction);
    BaseMotor(ControllerPWM* controller, OperationalRanges* attr, OperationModes mode, uint8_t direction);
    virtual float getFeedback();
    void actuate();
    // The input value is an angular velocity
    virtual void setReference(float w){};
    virtual void setControlType(OperationModes mode){};
    void invert();
    void stop();

    ~BaseMotor();
};

#endif /* BaseMotor */
