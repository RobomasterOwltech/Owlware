/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң����������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж���������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef FlySky_i6X_C
#define FlySky_i6X_C

#include "FlySky_i6X.h"

#define RC_CHANNAL_ERROR_VALUE 700  // 2048(funcino masomenos) //700 (original)

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

static int16_t RC_abs(int16_t value);
/**
 * @brief          remote control protocol resolution
 * @param[in]      sbus_buf: raw data point
 * @param[out]     rc_ctrl: remote control data struct point
 * @retval         none
 */
static void sbus_to_rc(volatile const uint8_t* sbus_buf, RC_ctrl_t* rc_ctrl);

// remote control data
RC_ctrl_t rc_ctrl;
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
 * @brief          remote control init
 * @param[in]      none
 * @retval         none
 */
void remote_control_init(void) { RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM); }
/**
 * @brief          get remote control data point
 * @param[in]      none
 * @retval         remote control data point
 */
const RC_ctrl_t* get_remote_control_point(void) { return &rc_ctrl; }

// TODO: SEE IF WE CAN SET THIS TO THE DEFAULT
uint8_t RC_data_is_error(void) {
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE) {
        if (rc_ctrl.rc.ch[0] > 0) {
            rc_ctrl.rc.ch[0] = RC_CHANNAL_ERROR_VALUE;
        } else {
            rc_ctrl.rc.ch[0] = -RC_CHANNAL_ERROR_VALUE;
        }
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE) {
        if (rc_ctrl.rc.ch[1] > 0) {
            rc_ctrl.rc.ch[1] = RC_CHANNAL_ERROR_VALUE;
        } else {
            rc_ctrl.rc.ch[1] = -RC_CHANNAL_ERROR_VALUE;
        }
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE) {
        if (rc_ctrl.rc.ch[2] > 0) {
            rc_ctrl.rc.ch[2] = RC_CHANNAL_ERROR_VALUE;
        } else {
            rc_ctrl.rc.ch[2] = -RC_CHANNAL_ERROR_VALUE;
        }
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE) {
        if (rc_ctrl.rc.ch[3] > 0) {
            rc_ctrl.rc.ch[3] = RC_CHANNAL_ERROR_VALUE;
        } else {
            rc_ctrl.rc.ch[3] = -RC_CHANNAL_ERROR_VALUE;
        }
    }
    if (RC_abs(rc_ctrl.rc.ch[4]) > RC_CHANNAL_ERROR_VALUE) {
        if (rc_ctrl.rc.ch[4] > 0) {
            rc_ctrl.rc.ch[4] = RC_CHANNAL_ERROR_VALUE;
        } else {
            rc_ctrl.rc.ch[4] = -RC_CHANNAL_ERROR_VALUE;
        }
    }
    if (RC_abs(rc_ctrl.rc.ch[5]) > RC_CHANNAL_ERROR_VALUE) {
        if (rc_ctrl.rc.ch[5] > 0) {
            rc_ctrl.rc.ch[5] = RC_CHANNAL_ERROR_VALUE;
        } else {
            rc_ctrl.rc.ch[5] = -RC_CHANNAL_ERROR_VALUE;
        }
    }

    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.ch[5] = 0;
    rc_ctrl.rc.s[0] = RC_SW_MID;
    rc_ctrl.rc.s[1] = RC_SW_MID;
    rc_ctrl.rc.s[2] = RC_SW_UP;
    rc_ctrl.rc.s[3] = RC_SW_MID;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void slove_RC_lost(void) { RC_restart(SBUS_RX_BUF_NUM); }
void slove_data_error(void) { RC_restart(SBUS_RX_BUF_NUM); }

void USART3_IRQHandler(void) {
    if (huart3.Instance->SR & UART_FLAG_RXNE)  //���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    } else if (USART3->SR & UART_FLAG_IDLE) {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // get receive data length, length = set_data_length - remain_length
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // reset set_data_lenght
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            // TODO: GET CORRECT FRAME LENGTH, SO WE CAN CHECK FOR ERRORS
            // if(this_time_rx_len == RC_FRAME_LENGTH)
            //{
            sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            detect_hook(DBUS_TOE);
            sbus_to_usart1(sbus_rx_buf[0]);
            //}
        } else {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // get receive data length, length = set_data_length - remain_length
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // reset set_data_lenght
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            // if(this_time_rx_len == RC_FRAME_LENGTH)
            //{
            sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            detect_hook(DBUS_TOE);
            sbus_to_usart1(sbus_rx_buf[1]);
            // }
        }
    }
}

//ȡ������
static int16_t RC_abs(int16_t value) {
    if (value > 0) {
        return value;
    } else {
        return -value;
    }
}
/**
 * @brief          remote control protocol resolution
 * @param[in]      sbus_buf: raw data point
 * @param[out]     rc_ctrl: remote control data struct point
 * @retval         none
 */
/**
 * @brief          ң����Э�����
 * @param[in]      sbus_buf: ԭ������ָ��
 * @param[out]     rc_ctrl: ң��������ָ
 * @retval         none
 */

static void sbus_to_rc(volatile const uint8_t* sbus_buf, RC_ctrl_t* rc_ctrl) {
    if (sbus_buf == NULL || rc_ctrl == NULL) {
        return;
    }

    rc_ctrl->rc.ch[0] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x0ff;  //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 1
                         (sbus_buf[4] << 10)) &
                        0x0ff;
    rc_ctrl->rc.ch[2] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x0ff;  //!< Channel 2
    rc_ctrl->rc.ch[3] = ((sbus_buf[5] >> 4) | (sbus_buf[6] << 4)) & 0x0ff;  //!< Channel 3

    // Knobs
    rc_ctrl->rc.ch[5] = ((sbus_buf[6] >> 7) | (sbus_buf[7] << 1)) & 0x0ff;  // Knob left
    // We decided to not map knob right :D
    // rc_ctrl->rc.ch[5] = ((sbus_buf[7]) |(sbus_buf[8])) &0x0ff;        // Knob right

    rc_ctrl->rc.s[0] = ((sbus_buf[9] >> 1) & 0x07) >> 2;  //!< Switch A
    rc_ctrl->rc.s[1] = ((sbus_buf[10]) & 0x20) >> 5;      //!< Switch B
    rc_ctrl->rc.s[2] = ((sbus_buf[12] >> 3) & 0x03);      //!< Switch C
    rc_ctrl->rc.s[3] = ((sbus_buf[13] & 0x08) >> 3);      //!< Switch D

    rc_ctrl->rc.ch[6] = sbus_buf[13] | (sbus_buf[15] << 8);  // NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;

    rc_ctrl->rc.ch[0] *= 2;
    rc_ctrl->rc.ch[1] *= 2;
    rc_ctrl->rc.ch[2] *= 2;
    rc_ctrl->rc.ch[3] *= 2;
    // TODO: SEE IF NOT HAVING THE CORRECT OFFSET ON CH4 AFFECTS OR NOT
    // Original
    rc_ctrl->rc.ch[4] -= 1024;

    // rc_ctrl->rc.ch[4] -= RC_KNOB_VALUE_OFFSET;
    rc_ctrl->rc.ch[5] -= RC_KNOB_VALUE_OFFSET;  // 0;
}

/**
 * @brief          send sbus data by usart1, called in usart3_IRQHandle
 * @param[in]      sbus: sbus data, 18 bytes
 * @retval         none
 */
/**
 * @brief          ͨ��usart1����sbus����,��usart3_IRQHandle����
 * @param[in]      sbus: sbus����, 18�ֽ�
 * @retval         none
 */
void sbus_to_usart1(uint8_t* sbus) {
    static uint8_t usart_tx_buf[20];
    static uint8_t i = 0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, sbus, 18);
    for (i = 0, usart_tx_buf[19] = 0; i < 19; i++) {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
    usart1_tx_dma_enable(usart_tx_buf, 20);
}

#endif