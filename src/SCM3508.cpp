#include "SCM3508.hpp"

SCM3508::SCM3508(TIM_HandleTypeDef timer, OperationModes mode, uint8_t motorId) {
    // Setting the properties
    this->attr->maxVelocity = 400;   // rpm
    this->attr->minVelocity = -400;  // rpm

    this->attr->maxVelocity = NAN;
    this->attr->minVelocity = NAN;

    this->attr->minTemp = minTemp;
    this->attr->maxTemp = maxTemp;

    this->attr->maxTorque = NAN;
    this->attr->minTorque = NAN;

    this->contr.pwm = new ControllerPWM(timer, mode);
}

void SCM3508::setSpeed(float _Speed) { Speed = _Speed; }

float SCM3508::actuateVelocity(uint16_t duty_Cycle, float maxVel) { return (duty_Cycle * maxVel); }

void SCM3508::setReference(float _reference) {}
void SCM3508::setControlType(OperationModes _mode) { mode = _mode; }