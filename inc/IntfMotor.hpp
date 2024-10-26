/*
 * IntfMotor.hpp
 *
 *  Created on: Apr 10, 2024
 *      Author: @JorgePerC
 * This is based on the following:
 * https://github.com/STMicroelectronics/STM32CubeF4/blob/master/Projects/STM324xG_EVAL/Examples/CAN/CAN_Networking/Src/main.c
 */
#ifndef IntfMotor_HPP
#define IntfMotor_HPP

#include "ControllerCAN.hpp"
#include "ControllerPWM.hpp"

typedef union {
     ControllerCAN* can;
    ControllerPWM* pwm;
} Controller;
class motor{
    public:
        void setReference();
        void setControlType();
        float getFeedback();
        void stop();
        void invert();
    private:
        Controller* comunicationType;
        int16_t maxVel;
        int16_t minVel;
        int8_t Direction;
        int16_t maxAngle;
        int16_t minAngle;
        int16_t maxCurrent;
        int16_t minCurrent;
        uint8_t ID;       
};

// typedef enum { POS, VEL, TOR } OperationModes;

// typedef struct {
//     float maxVelocity;
//     float minVelocity;
//     float maxTorque;
//     float minTorque;
//     float maxPosition;
//     float minPosition;
//     float maxTemp;
//     float minTemp;
// } OperationalRanges;

// typedef union {
//     ControllerCAN* can;
//     ControllerPWM* pwm;
// } Controller;

// //TODO: Create a new union for selecting the ID
// // bc it can be a channel, or for can
// class IntfMotor {
//    protected:
//     Controller contr;
//     OperationalRanges* attr;

//     uint8_t mode;
//     uint8_t dir;
//     uint16_t runFreq;

//     virtual void actuateVelocity();
//     virtual void actuatePosition();
//     virtual void actuateTorque();

//    public:
//     IntfMotor();
//     IntfMotor(ControllerCAN* controller, OperationalRanges* attr, OperationModes mode, uint8_t direction);
//     IntfMotor(ControllerPWM* controller, OperationalRanges* attr, OperationModes mode, uint8_t direction);
//     virtual float getFeedback();
//     void actuate();
//     // The input value is an angular velocity
//     virtual void setReference(float w){};
//     virtual void setControlType(OperationModes mode){};
//     void invert();
//     void stop();

//     ~IntfMotor();
// };

#endif /* IntfMotor */
