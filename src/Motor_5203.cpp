// For more information about the motor:
// https://github.com/RobomasterOwltech/wiki/blob/main/PDFs/RM_MTR_GM6020-Brushless_User-Guide.pdf
#include "Motor_5203.hpp"

Motor_5203::Motor_5203(CAN_HandleTypeDef hcan, OperationModes mode, uint8_t motorId) {
    this->attr->maxPosition = maxAngle;
    this->attr->minPosition = 0;

    this->attr->maxVelocity = maxVoltage;
    this->attr->minVelocity = minVoltage;

    this->attr->minTemp = minTemp;
    this->attr->maxTemp = maxTemp;

    this->attr->maxTorque = maxCurrent;
    this->attr->minTorque = minCurrent;

    this->contr.can = new ControllerCAN(hcan, uint8_t motorId);
}

Motor_5203::Motor_5203(TIM_HandleTypeDef htimer) {}