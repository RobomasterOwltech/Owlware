/*
 * IntfMotor.cpp
 *
 *  Created on: Apr 12, 2024
 *      Author: @JorgePerC
 */
#include "IntfMotor.hpp"

IntfMotor::IntfMotor(){
    //Empty
}

IntfMotor::IntfMotor(Controller* controller, OperationModes mode, uint8_t direction) {

    // Asignaciones
    this->contr = controller;  
    this->direction = direction;
    this->mode = mode;
}

void IntfMotor::actuate(int16_t ref) {
    this->ref = ref;

    if (mode == VEL) {
        actuateVelocity(ref);
    } else if (mode == TOR) {
        actuateTorque(ref);
    } else if (mode == POS) {
        actuatePosition(ref);
    }
}

void IntfMotor::invert(uint8_t direction){
    this -> direction *= -1;
}

void IntfMotor::stop(int16_t ref){
    this -> ref=0;
}
