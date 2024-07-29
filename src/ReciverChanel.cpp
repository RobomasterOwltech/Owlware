/*
 * ReciverChanel.cpp
 *
 *  Created on: Apr 13, 2024
 *      Author: @carloseag1234
 */

 #include "ReciverChanel.hpp"

ReciverChanel::ReciverChanel(TIM_HandleTypeDef* htim_, uint32_t channel1_, uint32_t channel2_) :
    htim(htim), channel1(channel1_), channel2(channel2_), pulse_width(0) {}

void ReciverChanel::begin() {
    HAL_TIM_IC_Start_IT(htim, channel1);
    HAL_TIM_IC_Start_IT(htim, channel2);
}

uint32_t ReciverChanel::getPulseWidth() {
    return pulse_width;
}

void ReciverChanel::HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        capturedValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        if(capturedValue){
            frecuency = systemCoreCloclk / capturedValue;
            pulse_width = 1000* HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2)/capturedValue;
            
        }
    }
}