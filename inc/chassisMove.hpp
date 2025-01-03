/*
 * ControllerCAN.hpp
 *
 *  Created on: April 12, 2024
 *      Author: @JorgePerC
 * 
 * For mor information about how to use this driver: 
 * um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics-1
 */

#ifndef chassisMove_HPP
#define chassisMove_HPP

// ===== Includes =====
#include "IntfMotor.hpp" 
#include "stm32f4xx_hal.h"


class chassisMove {
    private:
        IntfMotor* leftFrontMotor;   
        IntfMotor* rightFrontMotor;  
        IntfMotor* leftBackMotor;   
        IntfMotor* rightBackMotor;  

        int16_t speed;  

    public:
        chassisMove();
        chassisMove(IntfMotor* leftFrontMotor, IntfMotor* rightFrontMotor,
                    IntfMotor* leftBackMotor, IntfMotor* rightBackMotor, int16_t speed = 100);

        void moveFront();
        void moveBack();
        void moveRight();
        void moveLeft();

        void stop();
};

#endif /*chassisMove_HPP*/