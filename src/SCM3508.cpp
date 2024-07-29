#include "SCM3508.hpp"

SpeedControllerPWM::SpeedControllerPWM(TIM_HandleTypeDef timer, OperationModes mode, uint8_t motorId) {
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

void SpeedControllerPWM::setSpeed(float _Speed) { Speed = _Speed; }

float SpeedControllerPWM::actuateVelocity(uint16_t duty_Cycle, float maxVel) { return (duty_Cycle * maxVel); }

void SpeedControllerPWM::setReference(float _reference) {}
void SpeedControllerPWM::setControlType(OperationModes _mode) { mode = _mode; }