#include "sr04.h"
#include "pid.h"    

uint16_t count;
float distance;
extern TIM_HandleTypeDef htim3;

void RCCdelay_us(uint32_t udelay)
{
  __IO uint32_t Delay = udelay * 72 / 8;//(SystemCoreClock / 8U / 1000000U)
    //ݻstm32f1xx_hal_rcc.c -- static void RCC_Delay(uint32_t mdelay)
  do
  {
    __NOP();
  }
  while (Delay --);
}

void GET_Distance(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
	RCCdelay_us(12);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  //上升下降沿产生中断
{
	if(GPIO_Pin==GPIO_PIN_2)
	{
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)==GPIO_PIN_SET)//上升沿开始计时
		{
			__HAL_TIM_SetCounter(&htim3,0);   //将定时器3初始值置0
			HAL_TIM_Base_Start(&htim3);	      //打开计时器时钟
		}
		else                                   //下降沿结束计时
		{
			HAL_TIM_Base_Stop(&htim3);
			count=__HAL_TIM_GetCounter(&htim3);
			distance=count*0.017;
		}
	}
	if(GPIO_Pin==GPIO_PIN_5)Control();
	
}