
/**
 * @brief   Interface for motor speed controllers
 * @author  Erick Daniel Ortiz Cervantes
 * @date    October 26, 2024
 */
#ifndef Controller_HPP
#define Controller_HPP

#include "stm32f4xx_hal.h"

typedef enum { PWM, CAN } ControllerType;

class Controller {
    private:
        ControllerType type;

    public:
        Controller();
        ~Controller();

        void setType(ControllerType);

        virtual void sendSignal(uint32_t);
        virtual uint32_t readSignal();
        virtual void stopActuator(uint32_t);
};

#endif
