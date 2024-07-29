/*
 * BaseMotor.cpp
 *
 *  Created on: Apr 12, 2024
 *      Author: @JorgePerC
 */
#include "BaseMotor.hpp"

BaseMotor::BaseMotor() {
    // Empty
}
BaseMotor::BaseMotor(ControllerCAN* controller, OperationalRanges* attr, OperationModes mode, uint8_t direction) {
    // TODO: Check pointers
    // TODO: validate parameters before successful obj creation
    this->contr.can = controller;
    this->attr = attr;
    this->dir = direction;
    this->mode = mode;
}

BaseMotor::BaseMotor(ControllerPWM* controller, OperationalRanges* attr, OperationModes mode, uint8_t direction) {
    this->contr.pwm = controller;
    this->attr = attr;
    this->dir = direction;
    this->mode = mode;
}

void BaseMotor::actuate() {
    switch (mode) {
        case POS:
            actuatePosition();
            break;
        case VEL:
            actuateVelocity();
            break;
        case TOR:
            actuateTorque();
            break;
        default:
            // TODO: Throw an error
            break;
    }
}

void BaseMotor::invert() { this->dir *= -1; }

void BaseMotor::stop() { this->setReference(0); }