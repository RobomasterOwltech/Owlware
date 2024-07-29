/*
 * ReciverChanel.hpp
 *
 *  Created on: Apr 13, 2024
 *      Author: @carloseag1234
 */


#ifndef ReciverChanel
#define ReciverChanel

class ReciverChanel

// ===== Includes =====
#include "stm32f4xx_hal.h"

{
private:
    TIM_HandleTypeDef* htim;
    uint32_t channel1;
    uint32_t channel2;
    uint32_t pulse_width;

    static void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
public:
    ReciverChanel(TIM_HandleTypeDef* htim, uint32_t channel1, uint32_t channel2);
    void begin();
    uint32_t getPWM();
    ~ReciverChanel();


};

#endif /* ReciverChanel */