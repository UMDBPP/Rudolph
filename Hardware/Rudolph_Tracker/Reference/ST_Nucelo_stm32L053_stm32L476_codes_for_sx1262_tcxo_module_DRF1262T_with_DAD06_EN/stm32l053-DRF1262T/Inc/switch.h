#ifndef _SWITCH_H_
#define _SWITCH_H_

#include "gpio.h"

#define TX_MODE        1
#define RX_MODE        0

#define LORA_MODEM     1
#define FSK_MODEM      0

extern const uint8_t switch_sf_val[8];

typedef struct 
{
	uint8_t use_modem;     	//使用的模式
	uint8_t band_width;    	//使用的带宽
	uint8_t tx_rx_mode;    	//收发模式
	uint8_t spread_factor; 	//分频因子
	
}SWITCH_STATE_ST;

SWITCH_STATE_ST Check_switch(void);  //检查用户选择模式


#endif
