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

virtual float IntfMotor::getFeedback(){

}

void IntfMotor::actuate(int16_t ref){

}

virtual void IntfMotor::setControlType(OperationModes mode){

}

void IntfMotor::invert(uint8_t direction){
    this -> ref*=-1;
}

void IntfMotor::stop(int16_t ref){
    this -> ref=0;
}