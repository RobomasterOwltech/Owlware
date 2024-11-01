/*
 * ControllerCAN.cpp
 *
 *  Created on: April 12, 2024
 *      Author: @JorgePerC
 */
#include "ControllerFDCAN.hpp"

// Remember MSB: Most significant bit

// This is a function that will be called in case something failed
// TODO: make it global but put it somewhere usefull
static void Error_Handler(void) {
    while (1) {
        // comment
        for (int i = 0; i < 10; i++) {
            i *= 1;
        }
    }
}

ControllerCAN::ControllerCAN(FDCAN* handler) {
    // Check there are no duplicate IDs
    hcan = handler;
    // Turn off interruption mode
    //  HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    //  HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);

    // HAL_CAN_DeactivateNotification(hcan, );
    // Check that the handler has been initialized
    if (!HAL_CAN_GetState(hcan) != CAN_INITSTATUS_SUCCESS) {
        Error_Handler();
    }
}

void ControllerCAN::updateMessage(uint8_t* bufferArr, uint8_t motorId, uint16_t value) {
    // This should only be updated from the same type of motor.
    // Also, if the IDs are in the ranges 1-4 || 5-7
    // TODO: limit the resource to comply
    // UPDT: That's why i'm passing as an argument the pointer to the buffer
    // see if we can return it, instead of passing it as a reference.

    // Set postion to be written
    uint8_t writePos = motorId == CAN_ID_4 ? 6 : (motorId % 4 - 1) * 2;

    bufferArr[writePos] = value;
}

void ControllerCAN::sendMessage(CAN_TxHeaderTypeDef* TxHeader, uint8_t* TxData) {
    // The example never actually sets a value to the mailbox
    // if (HAL_CAN_AddTxMessage(&CanHandle, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    // Prior to sending the message, we must
    // check that all prev messages have been sent
    // we do so wit the tx boxes
    uint32_t selectedMailBox = NULL;
    // Monitor the Tx mailboxes availability until at least one Tx mailbox is free
    if ((hcan->Instance->TSR & CAN_TSR_TME0) != 0U) {
        selectedMailBox = CAN_TX_MAILBOX0;
    } else if ((hcan->Instance->TSR & CAN_TSR_TME1) != 0U) {
        selectedMailBox = CAN_TX_MAILBOX1;
    } else if ((hcan->Instance->TSR & CAN_TSR_TME2) != 0U) {
        selectedMailBox = CAN_TX_MAILBOX2;
    }
    if (selectedMailBox != NULL) {
        if (HAL_CAN_AddTxMessage(hcan, TxHeader, TxData, &selectedMailBox) != HAL_OK) {
            // uart_buf_len = sprintf(uart_buf, "Error");
            // HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, HAL_MAX_DELAY);
            Error_Handler();
        }
    }
    // Timeout so that is has time to send the data
    HAL_Delay(10);
}

/* This method is configured to read messages through polling.
Note that the implementation is rather different from using interrupts
Since these functions NEED TO BE CONSTANTLY CALLED.
On one side, this is easier, since will be using RTOS later
*/
void ControllerCAN::readMessagePolling() {
    // TODO: How to pass it to the motors?

    // Check there's a message on the fifo
    if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_FILTER_FIFO0) > 1) {
        /* Get RX message */
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, RxHeader, RxData) != HAL_OK) {
            /* Reception Error */
            Error_Handler();
        }
    }
    // Timeout so that is has time to read the data

    HAL_Delay(10);
}

// TODO: Complete
void ControllerCAN::getMotorInfo(uint8_t feedbackID, uint8_t motorId) {
    // Decode message
    RxHeader->StdId = FEEDBACK_ID + motorId;
}

void ControllerCAN::setChannelFilter() {
    // TODO: FIGURE OUT HOW THIS WORKS
    //  Use interrtuptions or no
    //   HAL_CAN_ActivateNotification()

    //   // Filter messages and freq
    //   HAL_CAN_ConfigFilter();
}
