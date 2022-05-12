/**
 * @file servo.h
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
#ifndef SERVO_H
#define SERVO_H

#include "dynamixel2.h"
#include <stdbool.h>
#include "main.h"


#define USART6_BUFFER_SIZE 32

extern uint8_t usart6_rx_flag;
extern unsigned char usart6_rxbuff[USART6_BUFFER_SIZE];
extern uint32_t broadcast_ctrl_pos[12];
extern float broadcast_ctrl_angle[12];


typedef struct {
  uint8_t ID;
  uint32_t present_pos;
  uint32_t goal_pos;
  float present_angle;
  float goal_angle;
} Servo_s;



void Servo_Torque(uint8_t id, bool TrqState);
void Servo_Sync_Torque(bool TrqState);
void Servo_Goal_Position(uint8_t id, uint32_t pos);
void Servo_Sync_Goal_Position(uint32_t *pos);
void Servo_Sync_ResetLeg(uint8_t leg);
void Servo_Sync_ResetLeg1(uint8_t leg);
void Servo_Led(uint8_t id, bool LedState);
void Servo_Angle2Pos_Mappping(Servo_s *servo);
void Servo_Pos2Angle_Mappping(Servo_s *servo);
void show_broadcast();
void Usart6Receive_IDLE(UART_HandleTypeDef *huart);
void usart6_init();

#endif // SERVO_H