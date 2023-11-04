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





#endif