#include "bsp_key.h"   



void bsp_key_init()
{		
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(KEY1_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(KEY2_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    
	GPIO_InitStructure.GPIO_Pin = KEY1_GPIO_PIN;
	GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);	
	GPIO_InitStructure.GPIO_Pin = KEY2_GPIO_PIN;
	GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);
}

uint8_t bsp_key_scan()
{
    uint8_t key_value = 0;

    if(KEY1_GET_STATUS() == 0)
    {
        key_value |= 1<<0;
    }
    if (KEY2_GET_STATUS() == 0)
    {
        key_value |= 1<<1;
    }
    return key_value;
}

/*********************************************END OF FILE**********************/
