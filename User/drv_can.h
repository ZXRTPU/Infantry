#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#include "stdint.h"
#include "main.h"

void CAN1_Init(void);
void CAN2_Init( void );
void can_remote(uint8_t sbus_buf[],uint8_t can_send_id);

//��̨����������ͺ���-CAN1
void set_motor_current_can2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);

//���̵���������ͺ���-CAN2
void set_motor_current_chassis(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);

#endif