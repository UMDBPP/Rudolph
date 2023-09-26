/*
******************************************************
	*Soft VERSION:1.1
	*Company:DORJI
	*Email: sales@dorji.com
	*Web: 	http://www.dorji.com
*****************************************************
*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "sx126x-board.h"
#include "radio.h"
#include "switch.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
 SWITCH_STATE_ST user_select_mode = {0};   //user selected mode
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define Get_NIRQ()  HAL_GPIO_ReadPin(RADIO_DIO0_GPIO_Port,RADIO_DIO0_Pin)

/*
  Notes:
  1)The jumper T/R on DRF1262G is connected to SWCLK£¨PA14) so this jumper must be unconnected in order to download the codes into ST-NUCLEO board successfuly.
  2)If no jumpers are connected, the default settings will be: freq=868Mhz,mode=TX,power=22,BW=250kHZ,SF=12
    The freqeuncy selection for USE_BAND_868 and USE_BAND_915 can be done in KEIL MDK5: Options for targets->c/c++->Define global variable
  
   SF1   SF2  SF3       Spreading Factor    Switch Val
	 OFF   OFF  OFF             12          			0
	 ON    OFF  OFF             11								1
	 OFF   ON   OFF             10								2
	 OFF   OFF  ON               9								4
	 ON 	 ON   OFF              8          			3
	 ON    OFF  ON 							 7          			5
	 OFF   ON   ON               6          			6
   ON    ON   ON               5          			7
	 


   SB1   SB2   SB3   SB4        Bandwidth 		Switch Val
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
 */
 #if defined( USE_BAND_868 )

#define RF_FREQUENCY                                868000000 // Hz

#elif defined( USE_BAND_915 )

#define RF_FREQUENCY                                915000000 // Hz

#else
    #error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER                             22        // dBm TX output power

#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              1         // Bandwidth: 125khz,lookfor radio.c line 392 
																															//[250kHZ,125khz,62.5khz,20.83kHZ,41.67KHZ
																															//16.63kHZ,7.81KHZ,500khz,31.25KHZ,10.42kHZ]
#define LORA_SPREADING_FACTOR                       11         // Spreading Factor: SF11 [SF5..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )

#define FSK_FDEV                                    25e3      // Hz 
#define FSK_DATARATE                                50e3      // bps
#define FSK_BANDWIDTH                               100e3     // Hz >> DSB in sx126x
#define FSK_AFC_BANDWIDTH                           100e3     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
    #error "Please define a modem in the compiler options."
#endif

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Define the payload size here

const uint8_t PingMsg[] = "PING";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
		uint8_t i = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("Device is init OK...\r\n");
  HAL_TIM_Base_Start_IT(&htim2);
	
	user_select_mode = Check_switch();  //obtain users' selection
				 
	// Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

#if defined( USE_MODEM_LORA )

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, user_select_mode.band_width,
                                   user_select_mode.spread_factor, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, user_select_mode.band_width, user_select_mode.spread_factor,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

#elif defined( USE_MODEM_FSK )

    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

#else
    #error "Please define a frequency band in the compiler options."
#endif
	 if(RX_MODE == user_select_mode.tx_rx_mode)
	{
			Radio.Rx(0);
	}else
	{
		Radio.Send((uint8_t*)Buffer, 4 );
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	if(RX_MODE == user_select_mode.tx_rx_mode)
		{
		        if(  RX == State )  //Judge if the data is received
        { 
					HAL_GPIO_TogglePin(LED_BLUE_LEFT_GPIO_Port,LED_BLUE_LEFT_Pin);
          HAL_UART_Transmit(&huart1, (uint8_t *)Buffer, BufferSize, 0xFFFF);
					printf("\r\n");
					printf("RssiValue:%d,SnrValue:%d,State:%d\r\n",RssiValue,SnrValue,State);					
          State = LOWPOWER;

        }
		}else if (TX == State) //Judge if data transmission is finished
		{  
			  i++;  //unsigned char
			  sprintf((char*)Buffer,"%s%03d",PingMsg,i);
        Radio.Send((uint8_t*)Buffer, 7 );
			  HAL_GPIO_TogglePin(LED_RED_LEFT_GPIO_Port,LED_RED_LEFT_Pin);
				DelayMs(500);
		}
        // Process Radio IRQ
        Radio.IrqProcess( );				
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void OnTxDone( void )
{
//    Radio.Sleep( );
    State = TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
//    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
}

void OnTxTimeout( void )
{
 //   Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
//    Radio.Sleep( );
    State = RX_TIMEOUT;
}

void OnRxError( void )
{
//    Radio.Sleep( );
    State = RX_ERROR;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
