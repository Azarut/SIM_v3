/**
  ******************************************************************************
  * @file    SIM800+stm32f3
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F3xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_TwoBoards_ComIT
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/

static void RTC_CalendarConfig(void);
static void TimeStamp_First(uint8_t* showtime, uint8_t* showdate);
static void TimeStamp_Second(uint8_t* showtime, uint8_t* showdate);
/* Private define ------------------------------------------------------------*/
#define TRANSMITTER_BOARD
#define CRC16_INITIAL_REMAINDER 0x0000
#define CRC16_FINAL_XOR_VALUE   0x0000

#define CRC16_WIDTH (8 * sizeof(uint16_t))
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* RTC handler declaration */
RTC_HandleTypeDef RtcHandle;

/* Buffers used for displaying Time and Date */
uint8_t aShowTime[50] = {0};
uint8_t aShowDate[50] = {0};

/* UART handler declaration */
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;
  
/* Buffer used for transmission */
uint8_t aTxBuffer0[19] = "AT+CSTT=\"internet\"\r"; //AT+CGATT=1
uint8_t aTxBuffer[9] =  "AT+CIICR\r";
uint8_t aTxBuffer2[9] = "AT+CIFSR\r";
uint8_t aTxBuffer3[42] = "AT+CIPSTART=\"TCP\",\"82.204.241.138\",\"8081\"\r";
uint8_t aTxBuffer4[11] = "AT+CIPSEND\r";
uint8_t aTxBuffer5[10] = "AT+CGATT?\r";
uint8_t aTxBuffer7[11] = "AT+CGATT=1\r";
uint8_t aTxBuffer6[1] = {0x1A};
/* Buffer used for reception */
uint8_t aRxBuffer[12];
uint8_t aMeasBuffer[6];
int16_t  Version = 0x0003;
int32_t  TransmitterID = 123456789;
int16_t  MessageType = 3;
int16_t  DataType = 3082;
uint32_t DeltaTime = 2000;
uint16_t SystolicPressure = 141;
uint16_t DiastolicPressure = 97;
uint16_t Pulse = 88;
uint16_t CRC_calc = 0;
uint8_t aTCP_Buffer[22];

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void)
{
  /* Turn LED5 on */
  BSP_LED_On(LED5);
  while(1)
  {
  }
}

static const uint16_t crcTable[256] =
    { 0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
      0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
      0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
      0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
      0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
      0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
      0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
      0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
      0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
      0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
      0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
      0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
      0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
      0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
      0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
      0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
      0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
      0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
      0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
      0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
      0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
      0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
      0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
      0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
      0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
      0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
      0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
      0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
      0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
      0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
      0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
      0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202 };

uint16_t crc16(const void * const message, const uint16_t nBytes) {
    uint8_t  data;
    uint16_t byte, remainder = CRC16_INITIAL_REMAINDER;

    for (byte = 0; byte < nBytes; ++byte) {
        data = ((const uint8_t * const) message)[byte] ^
               (remainder >> (CRC16_WIDTH - 8));
        remainder = crcTable[data] ^ (remainder << 8);
    }

    return (remainder ^ CRC16_FINAL_XOR_VALUE);
}


void delay_us(unsigned int d){  /* for 168 MHz @ Level3 Opt */
    unsigned int i,j;
    i=d;while(i){i--;j=55;while(j)j--;}
}
 
void delay_ms(unsigned int d){  /* for 168 MHz @ Level3 Opt */
    unsigned int i;
    i=d;while(i){i--;delay_us(998);}
}
void Uart_Wait(void)
{
	//BSP_LED_Toggle(LED6);
  while (UartReady != SET)
  {
  } 
  /* Reset transmission flag */
  UartReady = RESET;
}
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{ uint8_t i = 0;

  HAL_Init();
  
  /* Configure LED3, LED4, LED5 & LED6 */
//  BSP_LED_Init(LED7);
//  BSP_LED_Init(LED8);

  /* Configure the system clock to 168 Mhz */
  SystemClock_Config();

  UartHandle.Instance        = USARTx;
  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED6);
	BSP_LED_Init(LED5);
	BSP_LED_Off(LED4);
  BSP_LED_On(LED6);
	  /*##-1- Configure the RTC peripheral #######################################*/
  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follows:
  - Hour Format    = Format 24
  - Asynch Prediv  = Value according to source clock
  - Synch Prediv   = Value according to source clock
  - OutPut         = Output Disable
  - OutPutPolarity = High Polarity
  - OutPutType     = Open Drain */ 
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  RtcHandle.Instance = RTC;
	
	if(HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
  
  /*##-2- Check if Data stored in BackUp register0: No Need to reconfigure RTC#*/
  /* Read the Back Up Register 0 Data */
  if(HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR0) != 0x32F2)
  {  
    /* Configure RTC Calendar */
    RTC_CalendarConfig();
  }
  else
  {
    /* Check if the Power On Reset flag is set */  
    if(__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
    {
      /* Turn on LED4: Power on reset occured */
      BSP_LED_On(LED4);
    }
    /* Check if Pin Reset flag is set */
    if(__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
    {
      /* Turn on LED6: External reset occured */
      BSP_LED_On(LED6);
    }
    /* Clear source Reset Flag */
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }

  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {

  }
  
  /* Configure User push-button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
  /* Wait for User push-button press before starting the Communication */


	aTCP_Buffer[1] = (uint8_t)Version;
	aTCP_Buffer[0] = Version >> 8;
	aTCP_Buffer[5] = (uint8_t)TransmitterID;
	aTCP_Buffer[4] = TransmitterID >> 8;
	aTCP_Buffer[3] = TransmitterID >> 16;
	aTCP_Buffer[2] = TransmitterID >> 24;
	aTCP_Buffer[7] = (uint8_t)MessageType;
	aTCP_Buffer[6] = MessageType >> 8;
	aTCP_Buffer[9] = (uint8_t)DataType;
	aTCP_Buffer[8] = DataType >> 8;
	aTCP_Buffer[13] = (uint8_t)DeltaTime;
	aTCP_Buffer[12] = DeltaTime >> 8;
	aTCP_Buffer[11] = DeltaTime >> 16;
	aTCP_Buffer[10] = DeltaTime >> 24;
	aTCP_Buffer[15] = (uint8_t)SystolicPressure;
	aTCP_Buffer[14] = SystolicPressure >> 8;
	aTCP_Buffer[17] = (uint8_t)DiastolicPressure;
	aTCP_Buffer[16] = DiastolicPressure >> 8;
	aTCP_Buffer[19] = (uint8_t)Pulse;
	aTCP_Buffer[18] = Pulse >> 8;
//	aTCP_Buffer[20] = (uint8_t)Version >> 8;
//	aTCP_Buffer[21] = (uint8_t)Version >> 8;
  while (BSP_PB_GetState(BUTTON_USER) == RESET)
  {
  }

	HAL_UART_DeInit(&UartHandle);
	HAL_UART_Init(&UartHandle);
	HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxBuffer7, 11,10);
	delay_ms(50);
  HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxBuffer5, 10,10);
	delay_ms(50);
  HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxBuffer0, 19,10);
	delay_ms(550);
  HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxBuffer, 9,10);
	delay_ms(1650);
  HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxBuffer2, 9,10);
	delay_ms(650);
	HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxBuffer3, 42,10);

	delay_ms(1000);
  while (1)
  {
//		TimeStamp_Second(aShowTime, aShowDate);
  //delay_ms(1000);
	BSP_LED_Off(LED4);
	BSP_LED_On(LED5);	
  while (BSP_PB_GetState(BUTTON_USER) == RESET)
  {
  }
	
	HAL_UART_DeInit(&UartHandle);
	HAL_UART_Init(&UartHandle);
	HAL_UART_Receive_IT(&UartHandle, (uint8_t*)aRxBuffer, 12);
  Uart_Wait();
	if(aRxBuffer[0] == 0x55)
	{
		for(i=0;i<=5;i++)
		{
			if(aRxBuffer[2*i+1] > 0x040)
				aMeasBuffer[i] = (aRxBuffer[2*i+1] - 0x37) << 4;
			else	
				aMeasBuffer[i] = (aRxBuffer[2*i+1] - 0x30) << 4;
			
			if(aRxBuffer[2*i+2] > 0x040)
				aMeasBuffer[i] += aRxBuffer[2*i+2] - 0x37;
			else
				aMeasBuffer[i] += aRxBuffer[2*i+2] - 0x30;
		}
		
	aTCP_Buffer[15] = aMeasBuffer[1] + aMeasBuffer[2];
	aTCP_Buffer[17] = aMeasBuffer[2];
	aTCP_Buffer[19] = aMeasBuffer[3];
	CRC_calc = crc16((uint32_t*)aTCP_Buffer, 20);
	aTCP_Buffer[21] = (uint8_t)CRC_calc;
	aTCP_Buffer[20] =CRC_calc >> 8;
		
	HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxBuffer4, 11,10);
 // Uart_Wait();
	BSP_LED_Toggle(LED4);
	delay_ms(1000);
	BSP_LED_Toggle(LED4);
	HAL_UART_Transmit(&UartHandle, (uint8_t*)aTCP_Buffer, 22,10);
 // Uart_Wait();
	BSP_LED_Toggle(LED4); 
	delay_ms(50);
	BSP_LED_Toggle(LED4);
  HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxBuffer6, 1,10);
	BSP_LED_Toggle(LED4);
	delay_ms(1000);
	}
  //RTC_CalendarShow(aShowTime, aShowDate);
  }
}
 
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV                     = 1
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
#ifdef USE_FULL_ASSERT
  uint32_t ret = HAL_OK;
#endif /* USE_FULL_ASSERT */
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  
#ifdef USE_FULL_ASSERT
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    assert_failed((uint8_t *)__FILE__, __LINE__);
  }
#else
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
#endif /* USE_FULL_ASSERT */
    	
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

#ifdef USE_FULL_ASSERT
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
  if(ret != HAL_OK)
  {
    assert_failed((uint8_t *)__FILE__, __LINE__);
  }
#else
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
#endif /* USE_FULL_ASSERT */
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;

  /* Turn LED6 on: Transfer in transmission process is correct */
  BSP_LED_On(LED6); 
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;
  
  /* Turn LED4 on: Transfer in reception process is correct */
  BSP_LED_On(LED4);
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  /* Turn LED3 on: Transfer error in reception/transmission process */
  BSP_LED_On(LED3); 
}
static void RTC_CalendarConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;
  
  /*##-1- Configure the Date #################################################*/
  /* Set Date: Monday August 19th 2013 */
  sdatestructure.Year = 0x14;
  sdatestructure.Month = RTC_MONTH_OCTOBER;
  sdatestructure.Date = 0x14;
  sdatestructure.WeekDay = RTC_WEEKDAY_TUESDAY;
  
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    //Error_Handler(); 
  } 
  
  /*##-2- Configure the Time #################################################*/
  /* Set Time: 02:00:00 */
  stimestructure.Hours = 0x14;
  stimestructure.Minutes = 0x21;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    //Error_Handler(); 
  }
  
  /*##-3- Writes a data in a RTC Backup data Register0 #######################*/
  HAL_RTCEx_BKUPWrite(&RtcHandle,RTC_BKP_DR0,0x32F2);  
}

/**
  * @brief  Display the current time and date.
  * @param  showtime : pointer to buffer
  * @param  showdate : pointer to buffer
  * @retval None
  */
static void TimeStamp_First(uint8_t* showtime, uint8_t* showdate)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
  //sprintf((char*)showtime,"%0.2d:%0.2d:%0.2d",stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
  /* Display date Format : mm-dd-yy */
  //sprintf((char*)showdate,"%0.2d-%0.2d-%0.2d",sdatestructureget.Month, sdatestructureget.Date, 2000 + sdatestructureget.Year); 
} 

static void TimeStamp_Second(uint8_t* showtime, uint8_t* showdate)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
  sprintf((char*)showtime,"%0.2d:%0.2d:%0.2d",stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
  /* Display date Format : mm-dd-yy */
  sprintf((char*)showdate,"%0.2d-%0.2d-%0.2d",sdatestructureget.Month, sdatestructureget.Date, 2000 + sdatestructureget.Year); 
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT *****END OF FILE****/
