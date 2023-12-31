#ifndef CHASSIS_TASK_H
#define  CHASSIS_TASK_H

#include "pid.h"
#include  "drv_can.h"
#include "rc_potocal.h"
#include "main.h"
#include "encoder_map.h"
#include "rc_map.h"

typedef struct
{
    uint16_t can_id;		//ID号
    int16_t  set_current;		//发送信息
    uint16_t rotor_angle;		//现在的角度
    int16_t  rotor_speed;		//现在的转速
    int16_t  torque_current;		//实际转矩电流
    uint8_t  temp;		//电机温度
}motor_info_t;


typedef enum {
    CHAS_LF,
    CHAS_RF,
    CHAS_RB,
    CHAS_LB,
} chassis_motor_cnt_t;

//=================================底盘任务变量定义======================================
extern int16_t Drifting_yaw;
extern uint16_t Down_ins_yaw;

//motor_info_t  motor_info_chassis[4];   //电机信息结构体

//================================底盘任务函数定义========================================
void rudder_motor_task(void const * argument);
void RC_to_Vector(void);
void chassis_motol_speed_calculate(void);
void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed);


void chassis_current_give_RC_6020();
void chassis_current_give();


#endif





