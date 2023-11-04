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

extern motor_info_t  motor_info_chassis[4];       //电机信息结构体

fp32 yaw_speed_pid [3]={30,0.5,10};   //用的原来的pid
fp32 yaw_angle_pid[3]={2,0,0.3};

pid_struct_t yaw_motor_pid;

void gimbal_task(void const * argument)
{
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}


//=======================================云台坐标系与底盘坐标系=================================================
void GXY_CXY( )
{
	
}


//==========================================锁云台模式===========================================================
fp32 ZERO_yaw=0;  //上电时陀螺仪角度



//=======================================云台电机PID输出计算及控制电流发送======================================
fp32 gimbal_current=0;
//在已经获取到电机实时速度和由相应功能需求解算出目标速度的情况下，计算电机速度的PID输出和发送电流
void Gimbal_gyro_speed_current()
{
	  pid_init(&yaw_motor_pid, yaw_speed_pid, 6000, 6000);
				
		gimbal_current=pid_calc(&yaw_motor_pid,yaw_motor.rotor_speed,yaw_motor.target_speed);
				
	  //gimbal_current的值校验合格，才赋值给yaw轴电机结构体
		if(gimbal_current>yaw_motor.target_speed)
		{
			gimbal_current=0;
		}
				
		yaw_motor.set_current = gimbal_current;	
    		
	  set_motor_current_chassis(0,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current );
}

void Gimbal_gyro_angle_current()
{
			  pid_init(&yaw_motor_pid, yaw_speed_pid, 10000, 10000);  
			
			  yaw_motor.target_speed=pid_pitch_calc(&yaw_motor_pid,yaw_motor.gyro_angle,yaw_motor.target_angle);
			
			  if((get_x_ch1()==0)&&(get_y_ch0()==0))
				{
					yaw_motor.target_speed=0;
				}
			
				pid_init(&yaw_motor_pid, yaw_angle_pid, 6000, 6000);
				
				gimbal_current=pid_calc(&yaw_motor_pid,yaw_motor.rotor_speed,yaw_motor.target_speed);
				
				if(gimbal_current>yaw_motor.target_speed)
				{
					gimbal_current=0;
				}
				
				yaw_motor.set_current = gimbal_current;
				
				set_motor_current_chassis(0,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current );
}



