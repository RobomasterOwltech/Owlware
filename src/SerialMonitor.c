/*
 * SerialMonitor.cpp
 *
 * UART Driver
 *  Created on: Apr 12, 2024
 *      Author: @JorgePerC
 */
#ifndef SerialMonitor_C
#define SerialMonitor_C

#include "SerialMonitor.h"
// TODO: Set this to a variable when selecting the chip

// Enable clock access
bool is_USART_enabled() {
    // TODO: Check that the registers are configured properly
}
bool is_USART_DMA_enabled() {
    // TODO: Check that the registers are configured properly
}

// GPIO General Purpose Input/Output

// SPIO Special Purpose Input/Output
// For this perspective, you active alternate functions

void USART_write(int ch) {
    while (!(USART2->SR & 0x0080)) {
        /* code */
    }
    USART2->DR = (ch & 0xFF);
}

// Interface to the C std I/O library

struct __FILE {
    int handle;
};
FILE __stdint = {0};
FILE __stdout = {1};
FILE __stderr = {2};

int fgetc(FILE* f) {
    int c;

    c = USART2_read();

    if (c == '\r') {
        USART_write(c);
        c = '\n';
    }

    return c;
}

int fputc(int c, FILE* f) { return USART2_write(c); }

#endif