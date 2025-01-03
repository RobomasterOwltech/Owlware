#include "chassisMove.hpp"

chassisMove::chassisMove(IntfMotor* leftFrontMotor, IntfMotor* rightFrontMotor,
                    IntfMotor* leftBackMotor, IntfMotor* rightBackMotor, int16_t speed = 100);

void chassisMove::moveFront() {

}


void chassisMove::moveBack() {

}

void chassisMove::moveRight() {
    leftFrontMotor->actuate(speed);   
    rightFrontMotor->actuate(-speed); 
    leftBackMotor->actuate(-speed);
    rightBackMotor->actuate(speed);
}

void chassisMove::moveLeft() {
    leftMotor->actuate(-speed);  
    rightMotor->actuate(speed); 
    leftBackMotor->actuate(speed); 
    rightBackMotor->actuate(-speed);
}

void chassisMove::stop() {
    leftMotor->stop(0); 
    rightMotor->stop(0); 
    leftBackMotor->stop(0); 
    rightBackMotor->stop(0); 
}
