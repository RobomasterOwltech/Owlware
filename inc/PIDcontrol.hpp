/*
 * PIDControl.hpp
 *
 *  Created on: Apr 13, 2024
 *      Author: @carloseag1234
 */

#ifndef PIDcontrol_HPP
#define PIDcontrol_HPP


class PIDcontrol {
private:
    float setPoint;
    float Kp;
    float Ki;
    float Kd;
    float error1;
    float integral;
    float(*plantaCallback)();

public:
    PIDcontrol(float kp_, float ki_, float kd_, float setpoint_, float (*func_ptr)());
    float output(float);
    void setSetpoint(float);
    ~PIDcontrol();
};

#endif /* PIDcontrol */