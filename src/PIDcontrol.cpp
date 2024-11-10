/*
 * PIDControl.cpp
 *
 *  Created on: Apr 13, 2024
 *      Author: @carloseag1234
 */
#include "PIDcontrol.hpp"

PIDcontrol::PIDcontrol(float kp_, float ki_, float kd_, float setpoint_, float (*func_ptr)())
    : Kp(kp_), Ki(ki_), Kd(kd_), setPoint(setpoint_), plantaCallback(func_ptr), error1(0), integral(0) {}

float PIDcontrol::output(float feedback) {
    float error = setPoint - feedback;
    integral += error;
    float derivative = error - error1;
    float output = Kp * error + Ki * integral + Kd * derivative;
    error1 = error;
    return output;
}

void PIDcontrol::setSetpoint(float setpoint_) { this->setPoint = setpoint_; }