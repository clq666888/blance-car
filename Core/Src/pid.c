#include "pid.h"                  // Device header
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"

//传感器数据变量
int Encoder_Left,Encoder_Right;   //全速75
float pitch,roll,yaw;
short gyroX,gyroY,gyroZ;
short aacX,aacY,aacZ;

//闭环控制中间变量
int Vertical_out,Velocity_out,Turn_out,Target_Speed,Target_turn,MOTO1,MOTO2;  //直立环，速度,角度环输出，速度环目标输入速度
float Med_Angle=1;//平衡角度偏移量(机械中值)，小车往前倒地极限角，和往后倒地极限角之和除以2

float Vertical_Kp=390,Vertical_Kd=1.5;    //KP:0~1000,Kd:0~10   
float Velocity_Kp=0.4,Velocity_Ki=0.4/200;    //KP:0~1,Ki=KP/200
//float Vertical_Kp=0,Vertical_Kd=0;    //KP:0~1000,Kd:0~10   
//float Velocity_Kp=0,Velocity_Ki=0;    //KP:0~1,Ki=KP/200
float Turn_Kp=20,Turn_Kd=0.4;

extern TIM_HandleTypeDef htim2,htim4;
extern float distance;
extern uint8_t Fore,Back,Left,Right,stop;
static uint8_t l;
 
#define SPEED_Y 30 //俯仰(前后)最大设定速度
#define SPEED_Z 50//偏航(左右)最大设定速度 
//直立环PD控制器
//目标角度值   真实角度值  角速度值
int Vertical(float Med,float Angel,float gyro_Y)
{
	int temp;
	temp=Vertical_Kp*(Angel-Med)+Vertical_Kd*gyro_Y;
	return temp;
}

//速度环PI控制器
//希望速度值   左真实速度值  右真实速度值 （后两个参数对应两个编码器）
int Velocity(int Target,int encoder_L,int encoder_R)
{
	static int Err_LowOut_last,Encoder_S;//上一个低通滤波，积分值
	static float a=0.7;
	Velocity_Ki=Velocity_Kp/200;
	int Err,Err_LowOut,temp;   //误差  误差值在低通滤波后的结果  中间值
	//计算误差值
	Err=(encoder_L+encoder_R)-Target;    
	//低通滤波
	Err_LowOut=(1-a)*Err+a*Err_LowOut_last;
	Err_LowOut_last=Err_LowOut;
	//积分
	Encoder_S+=Err_LowOut;
	//积分限幅（-20000~20000）
	Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
	//if(Back==1)Encoder_S=0,Back=0;
	//速度环计算
	temp=Velocity_Kp*Err_LowOut+Velocity_Ki*Encoder_S;
	return temp;
}

//转向环PD控制器
//角速度 目标转向角度值
int Turn(float gyro_Z,int Target_turn)
{
	int temp;
	temp=Turn_Kp*Target_turn+Turn_Kd*gyro_Z;
	return temp;
}

void Control()   //每隔10ms调用一次，利用MPU6050INT每隔10ms输出低电平和stm32EXTI5中断来定时
{
	int PWM_out;
	//读取编码器和陀螺仪
	Encoder_Left=Read_Speed(&htim2);
	Encoder_Right=-Read_Speed(&htim4);
	mpu_dmp_get_data(&pitch,&roll,&yaw);
	MPU_Get_Gyroscope(&gyroX,&gyroY,&gyroZ);
	MPU_Get_Accelerometer(&aacX,&aacY,&aacZ);
	
	//遥控
	if((Fore==0)&&(Back==0))Target_Speed=0;//未接受到前进后退指令-->速度清零，稳在原地
	if(Fore==1)
	{
		distance=51;  //超声波模块损坏，暂时用此代码屏蔽距离检测保护功能
		if(distance<50)       //超声波测距保护功能
			Target_Speed--;
		else
			Target_Speed++;
	}
	if(Back==1){Target_Speed--;}//
	Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);//限幅

	/*左右*/
	if((Left==0)&&(Right==0))Target_turn=0;
	if(Left==1)Target_turn-=30;	//左转
	if(Right==1)Target_turn+=30;	//右转
	Target_turn=Target_turn>SPEED_Z?SPEED_Z:(Target_turn<-SPEED_Z?(-SPEED_Z):Target_turn);//限幅( (20*100) * 100   )
	
	/*转向约束*/
	if((Left==0)&&(Right==0))Turn_Kd=0.6;//若无左右转向指令，则开启转向约束
	else if((Left==1)||(Right==1))Turn_Kd=0;//若左右转向指令接收到，则去掉转向约束

	//将数据传入pid控制器计算输出结果
	Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);//速度环
	Vertical_out=Vertical(Velocity_out+Med_Angle,roll,gyroX);      //直立环
	Turn_out=Turn(gyroZ,Target_turn);
	PWM_out=Vertical_out;
	MOTO1=PWM_out-Turn_out;
	MOTO2=PWM_out+Turn_out;
	Limit(&MOTO1,&MOTO2);
	
	if(stop==1)Load(0,0);
	else Load(MOTO1,MOTO2);
}










