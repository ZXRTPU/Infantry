#include "rc_map.h"
#include "math.h"

extern  RC_ctrl_t rc_ctrl;

double x=0;
double y=0; 

double  get_x_ch1()
{
  x=(double)rc_ctrl.rc.ch[1]; //x����-660~660֮��
	
	return x;
}


double  get_y_ch0()
{
  y=(double)rc_ctrl.rc.ch[0]; //y����-660~660֮��
	
	return y;
}


//����ң������ȡ�ĽǶ�ֵ��������׼��������,����Ŀ��Ƕ�
double get_xy_angle_8191(int_least16_t ZERO_POS)
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
		angle=atan2(get_y_ch0(),get_x_ch1());
	}
	else if((get_x_ch1()<0)&&(get_y_ch0()<0))
	{
		angle=atan2(get_y_ch0(),get_x_ch1());
	}
	else
	{
		//angle=0;
	}
	
	//���ϵõ��ĽǶ��ǻ����Ƶģ�����-PI~PI֮�䣬��Ҫת��Ϊ�����Ӧ�ı�������Χ-4096~4096
	
	angle=(angle/PI)*4096;
	
	return angle;
}







