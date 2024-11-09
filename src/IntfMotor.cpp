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

IntfMotor::IntfMotor(Controller* controller, OperationalRanges* attr, OperationModes mode, uint8_t direction) {

    // Asignaciones
    this->controller = controller;  
    this->attr = attr;
    this->dir = direction;
    this->mode = mode;
}

void IntfMotor::actuate(int16_t ref) {
    if (mode == VEL) {
        controller->velocity(ref);
    } else if (mode == TOR) {
        controller->torque(ref);
    } else if (mode == POS) {
        controller->pos(ref);
    }
}

void IntfMotor::invert(uint8_t direction){
    this -> direction *= -1;
}

void IntfMotor::stop(int16_t ref){
    this -> ref=0;
}

/*
virtual void actuateVelocity(int16_t ref);
virtual void actuatePosition(int16_t ref);
virtual void actuateTorque(int16_t ref);

void setVelocity();
void setTorque();
void setPosition();

TODO: implementar funciones???

*/