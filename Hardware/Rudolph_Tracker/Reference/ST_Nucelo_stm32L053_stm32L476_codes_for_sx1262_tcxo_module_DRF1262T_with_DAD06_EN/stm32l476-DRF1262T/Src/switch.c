// Header:	User select mode
// File Name:	 switch.c
// Author:	zhihong qiu
// Date:	2018/10/9

#include "switch.h"
#include "main.h"
 const uint8_t switch_sf_val[8] = {12,11,10,8,9,7,6,5};
/*
	Droji_SB1----PC4
	Droji_SB2----PC5
	Droji_SB3----PC6
	Droji_SB4----PC8

	Droji_SF1----PC10
	Droji_SF2----PC11
	Droji_SF3----PC12


	 SF1   SF2  SF3       Spreading Factor    Switch Val
	 OFF   OFF  OFF             12          			0
	 ON    OFF  OFF             11								1
	 OFF   ON   OFF             10								2
	 OFF   OFF  ON               9								4
	 ON 	 ON   OFF              8          			3
	 ON    OFF  ON 							 7          			5
	 OFF   ON   ON               6          			6
   ON    ON   ON               5          			7
	 


   BW1   BW2   BW3   BW4        Bandwidth 		Switch Val
   OFF   OFF   OFF   OFF         250KHz           0
   ON    OFF   OFF   OFF         125KHz           1
   OFF   ON    OFF   OFF         60.5KHz          2
   OFF   OFF   ON    OFF         41.67KHz         4
   OFF   OFF   OFF   ON          31.25KHz         8
   ON    ON    OFF   OFF         20.83KHz         3
   ON    OFF   ON    OFF         16.63KHz         5
   ON    OFF   OFF   ON          10.42KHz         9
   OFF   ON    ON    OFF         7.81KHz          6
   others                        500KHz
   *******************************************
*/
SWITCH_STATE_ST Check_switch(void)
{
	SWITCH_STATE_ST select_mode = {0};
	uint16_t sb_val = (GPIOC->IDR&0x70)>>4;  //检查SB选择
	uint16_t sf_val = (GPIOC->IDR&0x1c00)>>10; //检查 SF选择
	
	if(HAL_GPIO_ReadPin(DORJI_SB4_GPIO_Port,DORJI_SB4_Pin))
	{
		sb_val |= 1<<3;
	}
	select_mode.band_width = (~sb_val)&0x0f; //取低四位取反的值
	select_mode.spread_factor = switch_sf_val[(~sf_val)&0x07];//取低3位取反的值
	
	/*检查T/R选择*/
		if(HAL_GPIO_ReadPin(DORJI_T_R_GPIO_Port,DORJI_T_R_Pin))
	{
		select_mode.tx_rx_mode = TX_MODE;
		HAL_GPIO_WritePin(LED_RED_RIGHT_GPIO_Port,LED_RED_RIGHT_Pin,GPIO_PIN_RESET);	
	}
	else
	{
		select_mode.tx_rx_mode = RX_MODE;
		HAL_GPIO_WritePin(LED_BLUE_RIGHT_GPIO_Port,LED_BLUE_RIGHT_Pin,GPIO_PIN_RESET);	
	}
		
		/*检查L/F选择 只测试LORA 功能，不测试FSK功能*/
		if(HAL_GPIO_ReadPin(DORJI_L_F_GPIO_Port,DORJI_L_F_Pin))
	{
		select_mode.use_modem = LORA_MODEM;
//	HAL_GPIO_WritePin(LED_RED_RIGHT_GPIO_Port,LED_RED_RIGHT_Pin,GPIO_PIN_RESET);	
	}
	else
	{
		select_mode.tx_rx_mode = FSK_MODEM;
//	HAL_GPIO_WritePin(LED_BLUE_RIGHT_GPIO_Port,LED_BLUE_RIGHT_Pin,GPIO_PIN_RESET);	
	}
	
	return select_mode;	
}
