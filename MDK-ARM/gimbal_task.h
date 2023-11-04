#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"
#include "pid.h"

typedef struct
{
    uint16_t can_id;		//ID��
    int16_t  set_current;		//������Ϣ
    uint16_t rotor_angle;		//���ڵĽǶ�
    int16_t  rotor_speed;		//���ڵ�ת��
    int16_t  torque_current;		//ʵ��ת�ص���
    uint8_t  temp;		//����¶�
	
	  float gyro_angle;//�����ǽ�����ĵ�ǰ�Ƕ�
	  int16_t  gyro_omega;	//�����ǽ�����ĵ�ǰ���ٶ�
	
	  float target_angle;//Ŀ��Ƕ�
	  int16_t target_speed;  //Ŀ���ٶ�
	  
	  //pid_struct_t gimbal_pid; //������pid����
	
}gimbal_motor_info_t;

//=============================��������=========================
//gimbal_motor_info_t yaw_motor;
//gimbal_motor_info_t pitch_motor;
//gimbal_motor_info_t roll_motor;

//=============================��������ģ��======================
void gimbal_task(void const * argument);

void Gimbal_gyro_speed_current();

void Gimbal_gyro_angle_current();

void lock_gimbal_yaw();

void detel_gimbal(fp32* angle);

fp32 GXY_CXY(fp32 yaw_ground,fp32 chassis_ground);

void chassis_yaw_mode();

#endif





