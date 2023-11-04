#include "gimbal_task.h"
#include "Chassis_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "rc_map.h"
#include "encoder_map.h"
#include "drv_can.h"

gimbal_motor_info_t yaw_motor;
gimbal_motor_info_t pitch_motor;
gimbal_motor_info_t roll_motor;

extern motor_info_t  motor_info_chassis[4];       //�����Ϣ�ṹ��

fp32 yaw_speed_pid [3]={30,0.5,10};   //�õ�ԭ����pid
fp32 yaw_angle_pid[3]={2,0,0.3};

pid_struct_t yaw_motor_pid;

void gimbal_task(void const * argument)
{
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
		lock_gimbal_yaw();
		
		Gimbal_gyro_speed_current();
		
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}


//=======================================��̨����ϵ���������ϵ=================================================
fp32 GXY_CXY(fp32 yaw_ground,fp32 chassis_ground)
{
	fp32 yaw_chassis;
	yaw_chassis=yaw_ground-chassis_ground;
	
	return yaw_chassis;
}


//==========================================����̨ģʽ===========================================================
fp32 ZERO_yaw=0;  //�ϵ�ʱ�����ǽǶ�,������̨ģʽ�п��Կ�����̨��Ŀ�곯��
extern int16_t ZERO_pos[4];//Ҫ��ȡ��λУ׼ʱ����ı���ֵ

//��ϽǶȻ������yaw������Ŀ���ٶ�
void lock_gimbal_yaw()
{
	fp32 err_yaw=0;
	
	ZERO_yaw=ZERO_yaw+(get_xy_angle_8191(ZERO_pos[2])/4096*180);
	
	detel_gimbal(&ZERO_yaw);
	
	err_yaw=ZERO_yaw- yaw_motor.gyro_angle;
	
	//��Ʈ�����Ŀ��Ƕȼ���
	if(err_yaw>1||err_yaw<-1)
	{
		pid_init(&yaw_motor_pid, yaw_angle_pid, 10000, 10000);  
		
		//�˴����������̨����������ٶ�
		yaw_motor.target_speed=pid_calc(&yaw_motor_pid,yaw_motor.gyro_angle,ZERO_yaw);
	}
	else 
	{
		yaw_motor.target_speed=0;
	}
}

//Խ�紦��
void detel_gimbal(fp32* angle)
{
	if(*angle>180)
	{
		*angle=*angle-360;
	}
	else if(*angle<-180)
	{
		*angle=*angle+360;
	}
}

//=========================================���̸�����̨ģʽ=====================================================
//������yaw�������������ڵ�������ϵ�ľ��ԽǶ�����
//���̸�����̨ģʽ����ͨ���Ե��̵�PID���ƣ�yaw���������������ǴӶ���ʹyaw_chassis����Ϊ��
void chassis_yaw_mode()
{
	
}


//=======================================��̨���PID������㼰���Ƶ�������===============================================
fp32 gimbal_current=0;
//���Ѿ���ȡ�����ʵʱ�ٶȺ�����Ӧ������������Ŀ���ٶȵ�����£��������ٶȵ�PID����ͷ��͵���
void Gimbal_gyro_speed_current()
{
	  pid_init(&yaw_motor_pid, yaw_speed_pid, 6000, 6000);
				
		gimbal_current=pid_calc(&yaw_motor_pid,yaw_motor.rotor_speed,yaw_motor.target_speed);
				
	  //gimbal_current��ֵУ��ϸ񣬲Ÿ�ֵ��yaw�����ṹ��
		if(gimbal_current>yaw_motor.target_speed)
		{
			gimbal_current=0;
		}
				
		yaw_motor.set_current = gimbal_current;	
    		
	  set_motor_current_chassis(0,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current );
}

void Gimbal_gyro_angle_current()
{
			  pid_init(&yaw_motor_pid, yaw_angle_pid, 10000, 10000);  
			
			  yaw_motor.target_speed=pid_pitch_calc(&yaw_motor_pid,yaw_motor.gyro_angle,yaw_motor.target_angle);
			
			  if((get_x_ch1()==0)&&(get_y_ch0()==0))
				{
					yaw_motor.target_speed=0;
				}
			
				pid_init(&yaw_motor_pid, yaw_speed_pid, 6000, 6000);
				
				gimbal_current=pid_calc(&yaw_motor_pid,yaw_motor.rotor_speed,yaw_motor.target_speed);
				
				if(gimbal_current>yaw_motor.target_speed)
				{
					gimbal_current=0;
				}
				
				yaw_motor.set_current = gimbal_current;
				
				set_motor_current_chassis(0,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current );
}



