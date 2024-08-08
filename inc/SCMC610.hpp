#include <cmath>
#include <iostream>
#ifndef SCMC610 _H
#define SSCMC610 _H
using namespace std;

#include "IntfMotor.hpp"

class SCMC610 : IntfMotor {
   public:
    SCMC610(CAN_HandleTypeDef hcan, OperationModes mode, uint8_t motorId);

   private:
    static const uint8_t maxAngle = 8191;
    // Resolution for velocity control:
    static const int16_t maxVoltage = NAN;
    static const int16_t minVoltage = NAN;
    // Resolution for torque control:
    static const float maxCurrent = 10;
    static const float minCurrent = 10;
    // Operational temperature (Celsius):
    static const int16_t maxTemp = 55;
    static const int16_t minTemp = 0;

    static const uint8_t cntrlId = CONTROL_ID_A;
};
#endif