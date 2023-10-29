#include "rc_map.h"
#include "math.h"

extern  RC_ctrl_t rc_ctrl;

double  get_x_ch1()
{
	double x=0; 
  x=(double)rc_ctrl.rc.ch[1]-1024; //x����-660~660֮��
	
	return x;
}

double  get_y_ch0()
{
	int y=0; 
  y=(double)rc_ctrl.rc.ch[0]-1024; //y����-660~660֮��
	
	return y;
}

//����ң������ȡ�ĽǶ�ֵ��������׼��������,����Ŀ��Ƕ�
double get_xy_angle_8191(uint16_t ZERO_POS)
{
	double angle=0.0f;
	
	if(get_x_ch1()>0)
	{
		angle=atan2(get_y_ch0(),get_x_ch1());
	}
	else if((get_x_ch1()==0)&&(get_y_ch0()>0))
	{
		angle=PI/2;
	}
	else if((get_x_ch1()==0)&&(get_y_ch0()<0))
	{
		angle=-PI/2;
	}
	else if((get_x_ch1()<0)&&(get_y_ch0()>=0))
	{
		angle=atan2(get_y_ch0(),get_x_ch1())+PI;
	}
	else if((get_x_ch1()<0)&&(get_y_ch0()<0))
	{
		angle=atan2(get_y_ch0(),get_x_ch1())-PI;
	}
	else
	{
		angle=0;
	}
	
	//���ϵõ��ĽǶ��ǻ����Ƶģ�����-PI~PI֮�䣬��Ҫת��Ϊ�����Ӧ�ı�������Χ-4096~4096
	
	angle=(angle/PI)*4096;
	
	return angle;
}







