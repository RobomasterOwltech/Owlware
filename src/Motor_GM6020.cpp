
// For more information about the motor:
// https://github.com/RobomasterOwltech/wiki/blob/main/PDFs/RM_MTR_GM6020-Brushless_User-Guide.pdf
#include "Motor_GM6020.hpp"

void Motor_GM6020::initMotor(uint8_t motorId) {
    // Setting the properties specific to this model
    this->attr->maxPosition = maxAngle;
    this->attr->minPosition = 0;

    this->attr->maxVelocity = maxVoltage;
    this->attr->minVelocity = minVoltage;

    this->attr->minTemp = minTemp;
    this->attr->maxTemp = maxTemp;

    this->attr->maxTorque = maxCurrent;
    this->attr->minTorque = minCurrent;
    // Set the id
    this->motorId = motorId;
}

Motor_GM6020::Motor_GM6020(ControllerCAN* ctrl, OperationModes mode, uint8_t motorId) {
    // Calling the base class constructor
    // super(name, profession);

    // Start
    // We settled on a method, bc we can't call
    // a constructor within another prior to c++11
    initMotor(motorId);

    // There's no need to create a new controller for each motor, since they'll
    // be shared among several motors.
    this->contr.can = ctrl;

    isCAN = true;
}

Motor_GM6020::Motor_GM6020(TIM_HandleTypeDef htimer, OperationModes mode, uint8_t channel) {
    if (channel > MAX_CHNNLS_IN_TIMER) {
        // FAIL
    }
    initMotor(channel);
    // We create a new controller
    // Tho, after thinking simmilarly to the CAN Controller, it
    // can be shared and just specify the channel
    this->contr.pwm = new ControllerPWM(htimer, channel);
    isCAN = false;
}

// This is static
CAN_TxHeaderTypeDef* Motor_GM6020::getTxHeader(uint8_t cntrlId) {
    // shoudl I set this handler as static?
    CAN_TxHeaderTypeDef* TxHeader;
    TxHeader->StdId = cntrlId;
    // Specifies the type of identifier for the message
    TxHeader->IDE = CAN_ID_STD;
    // Specifies the length of the frame that will be transmitted
    TxHeader->DLC = RM_DLC;
    // Specifies the type of frame for the message
    TxHeader->RTR = CAN_RTR_DATA;
    // To avoid overriding the last values
    TxHeader->TransmitGlobalTime = DISABLE;
    return TxHeader;
}

// We should start at least returning values to indicate
// that everything went all right
void Motor_GM6020::setControlType(OperationModes mode) {
    if (isCAN) {
        if (mode == VEL) this->mode = mode;
    } else {
    }
}

void Motor_GM6020::actuateVelocity() {
    // This should set the new reference based on the
    // Control function that is calculated
    int vel = 0;
    if (isCAN) {
        // Control motor with CAN
        // Note how the control_ID changes
        if (motorId > CAN_ID_4) {
            contr.can->updateMessage(txBuff2, motorId, vel);
        } else {
            contr.can->updateMessage(txBuff1, motorId, vel);
        }

    } else {
        // Control motor with PWM
    }
}
void Motor_GM6020::moveCAN() {
    // TODO: Find out it my
    if (motorId > CAN_ID_4) {
        contr.can->sendMessage(getTxHeader(CONTROL_ID_2FF), txBuff2);
    }
    // We don't put an else clause, bc we can send for all motors at the same type
    contr.can->sendMessage(getTxHeader(CONTROL_ID_2FF), txBuff1);
}
