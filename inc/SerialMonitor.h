/*
 * SerialMonitor.h
 *
 *  Created on: Ago 7, 2024
 *      Author: @JorgePerC
 */

#ifndef SerialMonitor_H
#define SerialMonitor_H

#include <stdio.h>

#include "stm32f4xx.h"

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;

bool is_USART_enabled();
void USART_write(int ch);

#endif