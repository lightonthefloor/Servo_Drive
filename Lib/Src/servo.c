/**
 * @file servo.c
 * @brief 
 * @author Luke (AiNuozhou@bupt.edu.cn)
 * @version 1.0
 * @date 2021-10-27
 * 
 * @copyright Copyright (c) 2021  北京邮电大学
 * 
 * @par 修改日志:
 * @date 2021-10-27 
 * @author Luke (AiNuozhou@bupt.edu.cn)
 */

#include "servo.h"
#include "main.h"
#include "gpio.h"
#include "usart.h"

extern DMA_HandleTypeDef hdma_usart6_rx;

uint32_t broadcast_ctrl_pos[12] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};
float broadcast_ctrl_angle[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void Servo_Pos2Angle_Mappping(Servo_s *servo)
{
    // servo->goal_angle = 0;

    switch (servo->ID)
    {
    case 0:
        servo->goal_angle = (float)((servo->goal_pos - 2048) * -0.088028);
        break;
    case 1:
        servo->goal_angle = (float)((servo->goal_pos - 2048) * -0.08787);
        break;
    case 2:
        servo->goal_angle = (float)((servo->goal_pos - 2048) * -0.08787);
        break;
    case 10:
        servo->goal_angle = (float)((servo->goal_pos - 2048) * -0.088028);
        break;
    case 11:
        servo->goal_angle = (float)((servo->goal_pos - 2048) * 0.08787);
        break;
    case 12:
        servo->goal_angle = (float)((servo->goal_pos - 2048) * 0.08787);
        break;
    case 20:
        servo->goal_angle = (float)((servo->goal_pos - 2048) * 0.088028);
        break;
    case 21:
        servo->goal_angle = (float)((servo->goal_pos - 2048) * 0.08787);
        break;
    case 22:
        servo->goal_angle = (float)((servo->goal_pos - 2048) * 0.08787);
        break;
    case 30:
        servo->goal_angle = (float)((servo->goal_pos - 2048) * 0.088028);
        break;
    case 31:
        servo->goal_angle = (float)((servo->goal_pos - 2048) * -0.08787);
        break;
    case 32:
        servo->goal_angle = (float)((servo->goal_pos - 2048) * -0.08787);
        break;
    default:
        break;
    }
}

void Servo_Angle2Pos_Mappping(Servo_s *servo)
{
    // servo->goal_pos = 2048;

    switch (servo->ID)
    {
    case 0:
        servo->goal_pos = (uint32_t)(-11.36 * servo->goal_angle + 2048);
        break;
    case 1:
        servo->goal_pos = (uint32_t)(-11.38 * servo->goal_angle + 2048);
        break;
    case 2:
        servo->goal_pos = (uint32_t)(-11.38 * servo->goal_angle + 2048);
        break;
    case 10:
        servo->goal_pos = (uint32_t)(-11.36 * servo->goal_angle + 2048);
        break;
    case 11:
        servo->goal_pos = (uint32_t)(11.38 * servo->goal_angle + 2048);
        break;
    case 12:
        servo->goal_pos = (uint32_t)(11.38 * servo->goal_angle + 2048);
        break;
    case 20:
        servo->goal_pos = (uint32_t)(11.36 * servo->goal_angle + 2048);
        break;
    case 21:
        servo->goal_pos = (uint32_t)(11.38 * servo->goal_angle + 2048);
        break;
    case 22:
        servo->goal_pos = (uint32_t)(11.38 * servo->goal_angle + 2048);
        break;
    case 30:
        servo->goal_pos = (uint32_t)(11.36 * servo->goal_angle + 2048);
        break;
    case 31:
        servo->goal_pos = (uint32_t)(-11.38 * servo->goal_angle + 2048);
        break;
    case 32:
        servo->goal_pos = (uint32_t)(-11.38 * servo->goal_angle + 2048);
        break;
    default:
        break;
    }
}

void ServoSyncResetLeg1(uint8_t leg)
{
    uint32_t inst[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    uint32_t pos[12] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};

    switch (leg)
    {

    case 0:
    {
        uint8_t ids0[12] = {0, 1, 2};
        dxl2_sync_write(0x40, 4, ids0, inst, 3);
        dxl2_tx_packet();
        HAL_Delay(30);
        dxl2_sync_write(0x41, 4, ids0, inst, 3);
        dxl2_tx_packet();
        HAL_Delay(30);
        dxl2_sync_write(0x74, 4, ids0, pos, 3);
        dxl2_tx_packet();
        HAL_Delay(30);
        break;
    }
    case 1:
    {
        uint8_t ids1[12] = {10, 11, 12};
        dxl2_sync_write(0x40, 4, ids1, inst, 3);
        dxl2_tx_packet();
        HAL_Delay(30);
        dxl2_sync_write(0x41, 4, ids1, inst, 3);
        dxl2_tx_packet();
        HAL_Delay(30);
        dxl2_sync_write(0x74, 4, ids1, pos, 3);
        dxl2_tx_packet();
        HAL_Delay(30);
        break;
    }
    case 2:
    {
        uint8_t ids2[12] = {20, 21, 22};
        dxl2_sync_write(0x40, 4, ids2, inst, 3);
        dxl2_tx_packet();
        HAL_Delay(30);
        dxl2_sync_write(0x41, 4, ids2, inst, 3);
        dxl2_tx_packet();
        HAL_Delay(30);
        dxl2_sync_write(0x74, 4, ids2, pos, 3);
        dxl2_tx_packet();
        HAL_Delay(30);
        break;
    }
    case 3:
    {
        uint8_t ids3[12] = {30, 31, 32};
        dxl2_sync_write(0x40, 4, ids3, inst, 3);
        dxl2_tx_packet();
        HAL_Delay(30);
        dxl2_sync_write(0x41, 4, ids3, inst, 3);
        dxl2_tx_packet();
        HAL_Delay(30);
        dxl2_sync_write(0x74, 4, ids3, pos, 3);
        dxl2_tx_packet();
        HAL_Delay(30);
        break;
    }
    case 4:
    {
        uint8_t ids4[12] = {0, 1, 2, 10, 11, 12, 20, 21, 22, 30, 31, 32};
        dxl2_sync_write(0x40, 4, ids4, inst, 12);
        dxl2_tx_packet();
        HAL_Delay(30);
        dxl2_sync_write(0x41, 4, ids4, inst, 12);
        dxl2_tx_packet();
        HAL_Delay(30);
        dxl2_sync_write(0x74, 4, ids4, pos, 12);
        dxl2_tx_packet();
        HAL_Delay(30);
        break;
    }
    default:
        break;
    }
}

//void Servo_Sync_ResetLeg(uint8_t leg)
//{
//    uint32_t inst[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//    // uint32_t pos[12] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};
//    uint8_t ids[12] = {0, 1, 2, 10, 11, 12, 20, 21, 22, 30, 31, 32};
//    if (leg == 4)
//    {
//        for (int i = 0; i < 4; i++)
//        {
//            Dog.legs[i].servo[0].goal_pos = 2048;
//            Dog.legs[i].servo[1].goal_pos = 2048;
//            Dog.legs[i].servo[2].goal_pos = 2048;
//        }
//    }
//    else
//    {
//        Dog.legs[leg].servo[0].goal_pos = 2048;
//        Dog.legs[leg].servo[1].goal_pos = 2048;
//        Dog.legs[leg].servo[2].goal_pos = 2048;
//    }
//    Dog_servo_pos_angle();
//    dxl2_sync_write(0x40, 4, ids, inst, 12);
//    dxl2_tx_packet();
//    HAL_Delay(30);
//    dxl2_sync_write(0x41, 4, ids, inst, 12);
//    dxl2_tx_packet();
//    HAL_Delay(30);
//    dxl2_sync_write(0x74, 4, ids, broadcast_ctrl_pos, 12);
//    dxl2_tx_packet();
//    HAL_Delay(30);
//}

void Servo_Sync_Torque(bool TrqState)
{
    uint8_t ids[12] = {0, 1, 2, 10, 11, 12, 20, 21, 22, 30, 31, 32};
    uint32_t value[12] = {TrqState, TrqState, TrqState, TrqState, TrqState, TrqState, TrqState, TrqState, TrqState, TrqState, TrqState, TrqState};
    dxl2_sync_write(0x40, 4, ids, value, 12);
    dxl2_tx_packet();
    HAL_Delay(30);
    dxl2_sync_write(0x41, 4, ids, value, 12);
    dxl2_tx_packet();
    HAL_Delay(30);
    // uprintf("all Dynamixel to %d\n\r", TrqState);
}

void Servo_Torque(uint8_t id, bool TrqState)
{
		dxl2_write_byte(id, 0x41, TrqState);
		dxl2_tx_packet();
		HAL_Delay(30);
    dxl2_write_byte(id, 0x40, TrqState);
    dxl2_tx_packet();
    HAL_Delay(30);
//    uprintf("%d Dynamixel to %d\n\r", id, TrqState);
}
/**
 * @brief 同步写入位置指令
 * @param  pos              My Param doc
 */
void Servo_Sync_Goal_Position(uint32_t *pos)
{
    uint8_t ids[12] = {0, 1, 2, 10, 11, 12, 20, 21, 22, 30, 31, 32};
    dxl2_sync_write(0x74, 4, ids, pos, 12);
    dxl2_tx_packet();
    HAL_Delay(30);
}

/**
 * @brief 写入位置指令
 * @param  id               My Param doc
 * @param  pos              My Param doc
 */
void Servo_Goal_Position(uint8_t id, uint32_t pos)
{
    dxl2_write_dword(id, 0x74, pos);
    dxl2_tx_packet();
    HAL_Delay(30);
//    uprintf("id %d pos %d\n\r", id, pos);
}

void Servo_Led(uint8_t id, bool LedState)
{
    dxl2_write_byte(id, 0x41, LedState);
    dxl2_tx_packet();
    HAL_Delay(30);
}

//void show_broadcast()
//{
//    uprintf("broadcast_ctrl_pos: ");
//    for (int i = 0; i < 12; i++)
//    {
//        uprintf("%d ", broadcast_ctrl_pos[i]);
//    }
//    uprintf("\nbroadcast_ctrl_angle: ");
//    for (int i = 0; i < 12; i++)
//    {
//        uprintf("%f ", broadcast_ctrl_angle[i]);
//    }
//    uprintf("\n\r");
//}

unsigned char usart6_rxbuff[USART6_BUFFER_SIZE];
uint8_t usart6_rx_flag = 0;
uint8_t usart6_rx_buffer[128];
uint8_t send_buff[100];
unsigned int i = 0;

void usart6_init()
{
    /* USART6接收 */
    if (HAL_UART_Receive_DMA(&huart6, (uint8_t *)&usart6_rxbuff, USART6_BUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
    }
    /* 开启空闲接收中断 */
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
}

void Usart6Receive_IDLE(UART_HandleTypeDef *huart)
{
    //uint16_t i = 0;

    if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET))
    {
        if (huart->Instance == USART6)
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);
            i = huart->Instance->SR;
            i = huart->Instance->DR;
            i = hdma_usart6_rx.Instance->NDTR;
            /* ???é?????????????¨HAL_UART_DMAStop(huart)?????????????????°?????????TX???DMA??????é??????°???????DMA???é??????????°??? */
            /* 2020.10.24 ?????????HAL_UART_AbortReceive(huart)?????????DMA???é?????é???????????????¤???? */
            HAL_UART_AbortReceive(huart);

            /* ??¤?¤??¤??????°?????????????????ˇ????????????????????? */
            // if (usart6_rx_flag == 0)
            // {
            // 	memcpy(vp_txbuff, usart6_rxbuff, (USART6_BUFFER_SIZE - i));
            // 	usart6_rx_flag = 1;
            // }

            /* ???????????????é????°?????? */
            // memset(usart6_rxbuff, 0x00, USART6_BUFFER_SIZE);
            HAL_UART_Receive_DMA(huart, (uint8_t *)&usart6_rxbuff, USART6_BUFFER_SIZE);
        }
    }
}
