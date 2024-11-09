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

typedef enum { POS, VEL, TOR } OperationModes;

// typedef union {
//     ControllerCAN* can;
//     ControllerPWM* pwm;
// } Controller;

class IntfMotor {
    protected:
        Controller* contr; //polymorphism 

        int16_t maxVel;
        int16_t minVel;
        int16_t maxTorque;
        int16_t minTorque;
        int16_t maxPosition;
        int16_t minPosition;
        int8_t Direction;
        int16_t maxAngle;
        int16_t minAngle;
        int16_t maxCurrent;
        int16_t minCurrent;
        uint8_t ID; 

        int16_t ref;
        uint8_t mode;
        uint8_t dir;
        uint16_t runFreq;

        virtual void actuateVelocity(int16_t ref);
        virtual void actuatePosition(int16_t ref);
        virtual void actuateTorque(int16_t ref);

        void setVelocity();
        void setTorque();
        void setPosition();
        // void setTemp();

    public:
        IntfMotor();
        //IntfMotor(ControllerCAN* controller,  OperationModes mode, uint8_t direction);
        //IntfMotor(ControllerPWM* controller, OperationModes mode, uint8_t direction);
        IntfMotor(Controller* controller, OperationModes mode, uint8_t direction);
        virtual float getFeedback()=0;
        void actuate(int16_t ref);
        // The input value is an angular velocity
        virtual void setControlType(OperationModes mode){};
        void invert(uint8_t direction);
        void stop(int16_t ref);

        ~IntfMotor();
};

#endif /* IntfMotor */
