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
	uint8_t use_modem;     	//ʹ�õ�ģʽ
	uint8_t band_width;    	//ʹ�õĴ���
	uint8_t tx_rx_mode;    	//�շ�ģʽ
	uint8_t spread_factor; 	//��Ƶ����
	
}SWITCH_STATE_ST;

SWITCH_STATE_ST Check_switch(void);  //����û�ѡ��ģʽ


#endif
