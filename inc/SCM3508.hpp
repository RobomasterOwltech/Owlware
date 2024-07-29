#include <cmath>
#include <iostream>
#ifndef SCM3508_H
#define SCM3508_H
using namespace std;

#include "BaseMotor.hpp"

class SpeedControllerPWM : BaseMotor {
   public:
    SpeedControllerPWM(TIM_HandleTypeDef timer, OperationModes mode, uint8_t motorId);
    //~SpeedControllerPWM();

    void setSpeed(float);
    virtual float actuateVelocity(uint16_t, float);

    virtual void setReference(float w) override;
    virtual void setControlType(OperationModes) override;

   private:
    float Speed;

    float frecuenciaMax = 500;
    float frecuenciaMin = 50;
    float voltage = 24;
    float current = 20;
    float maxTemp = 50;
    float minTemp = 0;
    float maxPulseWidth = 2000;
    float minPulseWidth = 1000;
};
#endif
