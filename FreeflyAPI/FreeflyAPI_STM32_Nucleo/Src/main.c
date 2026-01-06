/*-----------------------------------------------------------------
	MIT License

	Copyright (c) 2017 Freefly Systems

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

	Filename: "main.c"
	This code implements a simple movi controller using an STM32 Nucleo dev kit, and an Arduino joystick shield.
	It uses the Freefly QX library to build control and parse stauts packets.
	It sends one control packet "QX277" at approximately 50Hz and recieves a status packet "QX287" packet in response.
	The joystick sheild inputs are as follows:
		- Joystick Button (D2) - cycles between the joystick controlling the gimbal in rate mode, 
			absolute position mode (Euler angles) and FIZ lens motor absolute position.
		- Button D3 - Switches gimbal kill state on/off
		- Button D4 - Resets faults on all FIZ axes and autocalibrates all
		- Button D5 - Limits range on the focus lens motor axis to a range specified by pressing, moving and releasing
		- Button D6 - Camera record start stop
		
-----------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include <stdio.h>	// for printf()
#include "QX_Protocol.h"
#include "QX_Protocol_App.h"
#include "simple_buffer.h"


/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart6;


/* Private variables ---------------------------------------------------------*/
uint32_t ADC_Buff[2];
float joystick_x, joystick_y, joystick_x_raw, joystick_y_raw;
uint8_t UART_Rx_char;
uint8_t UART_Tx_Buf[500];
uint8_t control_state = 0;
int32_t D2_debounce_cntr = 0;
uint32_t print_cntr = 0;


/* Private function prototypes -----------------------------------------------*/

// hardware
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);

// application
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void button_debounced_D2_press_event(void);
void button_debounced_D3_press_event(void);
uint8_t get_button_debounced_D2(uint8_t cycles);
uint8_t get_button_debounced_D3(uint8_t cycles);
void print_lens_state(Lens_Axis_State_General_e state);

/* Private functions -----------------------------------------------*/

//----------------------------------------------------------------------------
int main(void)
{
	/* MCU Configuration----------------------------------------------------------*/
	
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	
	/* Configure the system clock */
	SystemClock_Config();
	
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_USART6_UART_Init();
	
	// Start joystick analog
	HAL_ADC_Start_DMA(&hadc1, ADC_Buff, 2);
	
	// Setup buttons
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	
	GPIO_InitStruct.Pin = BUTTON_D2_PIN;
    HAL_GPIO_Init(BUTTON_D2_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = BUTTON_D3_PIN;
    HAL_GPIO_Init(BUTTON_D3_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = BUTTON_D4_PIN;
    HAL_GPIO_Init(BUTTON_D4_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = BUTTON_D5_PIN;
    HAL_GPIO_Init(BUTTON_D5_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = BUTTON_D6_PIN;
    HAL_GPIO_Init(BUTTON_D6_PORT, &GPIO_InitStruct);
	
	// Initialization - creates one client/controller instance
	QX_InitCli(&QX_Clients[0], QX_DEV_ID_MOVI_API_CONTROLLER, QX_ParsePacket_Cli_MoVI_Ctrl_CB);
	
	// Start UART Rx - Interrupts will fill a software circular buffer
	HAL_UART_Receive_IT(&huart6, &UART_Rx_char, 1);	
	
	/* Infinite loop */
	while (1)
	{
		// Set loop to be roughly 50 HZ
		HAL_Delay(20);
		
		// Get joystick inputs
		joystick_x_raw = mapfloat((float)ADC_Buff[1], 0.0f, 4096.0f, -1.0f, 1.0f);
		joystick_y_raw = mapfloat((float)ADC_Buff[0], 0.0f, 4096.0f, -1.0f, 1.0f);
		
		// Range Limit to +/-1.0f
		if (joystick_x_raw > 1.0f){
			joystick_x_raw = 1.0f;
		} else if (joystick_x_raw < -1.0f){
			joystick_x_raw = -1.0f;
		} else {
			joystick_x_raw = joystick_x_raw;
		}
		if (joystick_y_raw > 1.0f){
			joystick_y_raw = 1.0f;
		} else if (joystick_y_raw < -1.0f){
			joystick_y_raw = -1.0f;
		} else {
			joystick_y_raw = joystick_y_raw;
		}
	
		// Joystick deadband
		if ((joystick_x_raw > 0.05f) || (joystick_x_raw < -0.05f)){
			joystick_x = joystick_x_raw;
		} else {
			joystick_x = 0;
		}
		if ((joystick_y_raw > 0.05f) || (joystick_y_raw < -0.05f)){
			joystick_y = joystick_y_raw;
		} else {
			joystick_y = 0;
		}
		
		// Get gimbal kill command value and apply to the ctrl structure
		get_button_debounced_D3(5);
		
		// Handle control mode
		get_button_debounced_D2(5);
		switch (control_state) 
		{
			// Pan/Tilt rate control
			case 0:
				FreeflyAPI.control.pan.type = RATE;
				FreeflyAPI.control.tilt.type = RATE;
				FreeflyAPI.control.roll.type = DEFER;
				FreeflyAPI.control.focus.type = DEFER;
				FreeflyAPI.control.iris.type = DEFER;
				FreeflyAPI.control.zoom.type = DEFER;
				
				FreeflyAPI.control.pan.value = joystick_x;
				FreeflyAPI.control.tilt.value = joystick_y;
				FreeflyAPI.control.roll.value = 0;
				FreeflyAPI.control.focus.value = 0;
				FreeflyAPI.control.iris.value = 0;
				FreeflyAPI.control.zoom.value = 0;
				break;
			
			// Pan/Tilt absolute position control
			case 1:
				FreeflyAPI.control.pan.type = ABSOLUTE;
				FreeflyAPI.control.tilt.type = ABSOLUTE;
				FreeflyAPI.control.roll.type = DEFER;
				FreeflyAPI.control.focus.type = DEFER;
				FreeflyAPI.control.iris.type = DEFER;
				FreeflyAPI.control.zoom.type = DEFER;
				
				// use value without deadband for position control
				FreeflyAPI.control.pan.value = joystick_x_raw;	
				FreeflyAPI.control.tilt.value = joystick_y_raw;
				FreeflyAPI.control.roll.value = 0;
				FreeflyAPI.control.focus.value = 0;
				FreeflyAPI.control.iris.value = 0;
				FreeflyAPI.control.zoom.value = 0;
				break;
			
			// Focus/Iris absolute position control
			case 2:
				FreeflyAPI.control.pan.type = DEFER;
				FreeflyAPI.control.tilt.type = DEFER;
				FreeflyAPI.control.roll.type = DEFER;
				FreeflyAPI.control.focus.type = ABSOLUTE;
				FreeflyAPI.control.iris.type = ABSOLUTE;
				FreeflyAPI.control.zoom.type = ABSOLUTE;
				
				// use value without deadband for position control
				FreeflyAPI.control.pan.value = 0;
				FreeflyAPI.control.tilt.value = 0;
				FreeflyAPI.control.roll.value = 0;
				FreeflyAPI.control.focus.value = joystick_x_raw;
				FreeflyAPI.control.iris.value = joystick_y_raw;
				FreeflyAPI.control.zoom.value = 0;
				break;
			
			default:
				FreeflyAPI.control.pan.type = DEFER;
				FreeflyAPI.control.tilt.type = DEFER;
				FreeflyAPI.control.roll.type = DEFER;
				FreeflyAPI.control.focus.type = DEFER;
				FreeflyAPI.control.iris.type = DEFER;
				FreeflyAPI.control.zoom.type = DEFER;
				
				FreeflyAPI.control.pan.value = 0;
				FreeflyAPI.control.tilt.value = 0;
				FreeflyAPI.control.roll.value = 0;
				FreeflyAPI.control.focus.value = 0;
				FreeflyAPI.control.iris.value = 0;
				FreeflyAPI.control.zoom.value = 0;
				break;
		}
		
		// Clear lens controller axist faults. If a motor is not detected, the axis will fault and reuire that this is pressed to continue
		FreeflyAPI.control.fiz_clearFaults_all_flag = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(BUTTON_D4_PORT, BUTTON_D4_PIN)) ? 1 : 0;
		FreeflyAPI.control.fiz_autoCalStart_all_flag = FreeflyAPI.control.fiz_clearFaults_all_flag;	// do an auto cal directly out of reset. These can be independent also.
		
		// Press and hold this button to set a sub range limit. Focus is used for example. Press sets range start, release sets range end
		FreeflyAPI.control.fiz_setSubRangeLim_F_flag = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(BUTTON_D5_PORT, BUTTON_D5_PIN)) ? 1 : 0;
		
		// Starts / Stops configured camera
		FreeflyAPI.control.fiz_record_button_flag = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(BUTTON_D6_PORT, BUTTON_D6_PIN)) ? 1 : 0;
		
		// Send control packet
		FreeflyAPI.send();
		
		// Get Recieved Messages
		int i = 64;
		uint8_t c;
		while ((BufRemove(RECV_BUF_IDX, &c) != 0) && (i-- > 0)){
			QX_StreamRxCharSM(QX_COMMS_PORT_UART, c);
		}
		
		// Empty the send buffer
		for(i = 0; i < sizeof(UART_Tx_Buf); i++){
			if(BufRemove(SEND_BUF_IDX, &UART_Tx_Buf[i]) == 0) break;
		}
		if(i > 0)
			HAL_UART_Transmit_IT(&huart6, UART_Tx_Buf, i);
		
		// Print out status packets
		if (print_cntr++ > 20)
		{
			print_cntr = 0;
			printf("---------------------------------------------\n\r");
			printf("MoVI Gimbal and Status Data: \n\r");
			if (control_state == 0) printf("Test control mode: Gimbal rate \n\r");
			if (control_state == 1) printf("Test control mode: Gimbal absolute position \n\r");
			if (control_state == 2) printf("Test control mode: Focus and iris absolute position \n\r");
			printf("Left Battery: %f V\n\r", FreeflyAPI.status.battery_v_left);
			printf("Right Battery: %f V\n\r", FreeflyAPI.status.battery_v_right);
			printf("Gimbal Position (quaternion): %f, %f, %f, %f \n\r", FreeflyAPI.status.gimbal_i, FreeflyAPI.status.gimbal_j, FreeflyAPI.status.gimbal_k, FreeflyAPI.status.gimbal_r);
			printf("Camera: ");
			(FreeflyAPI.status.camera_recording == 0) ? printf("Stopped\n\r") : printf("Recording\n\r");
			printf("Lens Range Limits: ");
			(FreeflyAPI.status.focus_range_limits_active == 0) ? printf("Focus: OFF, ") : printf("Focus: ON, ");
			(FreeflyAPI.status.iris_range_limits_active == 0) ? printf("Iris: OFF, ") : printf("Iris: ON, ");
			(FreeflyAPI.status.zoom_range_limits_active == 0) ? printf("Zoom: OFF \n\r") : printf("Zoom: ON \n\r");
			printf("Focus Position: %d counts\n\r", FreeflyAPI.status.focus_position);
			printf("Iris Position: %d counts\n\r", FreeflyAPI.status.iris_position);
			printf("Zoom Position: %d counts\n\r", FreeflyAPI.status.zoom_position);
			printf("Focus State: "); 
			print_lens_state(FreeflyAPI.status.focus_state);
			printf("Iris State: "); 
			print_lens_state(FreeflyAPI.status.iris_state);
			printf("Zoom State: "); 
			print_lens_state(FreeflyAPI.status.zoom_state);
			printf("\r\n");
		}
		

	}
}


//----------------------------------------------------------------------------
// Print lens state
void print_lens_state(Lens_Axis_State_General_e state)
{
	switch (state)
	{
		case Lens_AxisState_Disabled: printf("Disabled"); break;
		case Lens_AxisState_Reset: printf("Reset"); break;
		case Lens_AxisState_Faulted: printf("Faulted"); break;
		case Lens_AxisState_Move_to_Command: printf("Moving to commanded position"); break;
		case Lens_AxisState_Calibrated: printf("Calibrated"); break;
		case Lens_AxisState_Uncalibrated: printf("Uncalibrated"); break;
		case Lens_AxisState_Man_Cal_Set_Max: printf("Manual Cal - Set max"); break;
		case Lens_AxisState_Man_Cal_Set_Min: printf("Manual Cal - Set min"); break;
		case Lens_AxisState_Auto_Cal_SensingTorque: printf("Auto Cal - Sensing torque"); break;
		case Lens_AxisState_Auto_Cal_Set_Max: printf("Auto Cal - Set max"); break;
		case Lens_AxisState_Auto_Cal_Set_Min: printf("Auto Cal - Set min"); break;
		default: break;
	}
	printf("\r\n");
}

//----------------------------------------------------------------------------
// allow printf through debugger serial output window
// http://armcortexm.blogs.upv.es/stm32f4-discovery-and-printf-redirection-to-debug-viewer-in-keil/
int fputc(int c, FILE *stream)
{
   return (ITM_SendChar(c));
}


//----------------------------------------------------------------------------
// IRQ transfer finished or DMA transfer finished or was disabled
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == USART6)
    {
		BufAdd(RECV_BUF_IDX, UART_Rx_char);
		HAL_UART_Receive_IT(huart, &UART_Rx_char, 1);
    }
}


//----------------------------------------------------------------------------
uint8_t get_button_debounced_D2(uint8_t cycles)
{
	static uint8_t value = 0;
	static uint8_t value_last = 0;
	static uint8_t debounce_cntr = 0;
	
	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(BUTTON_D2_PORT, BUTTON_D2_PIN)) // Pressed
	{
		debounce_cntr++;
		if (debounce_cntr >= cycles){
			value = 1;
			debounce_cntr = cycles;
		}
	}
	else	// Raised
	{
		debounce_cntr--;
		if (debounce_cntr <= 0){
			debounce_cntr = 0;
			value = 0; 
		}
	}
	
	if ((value_last == 0) && (value == 1))
	{
		button_debounced_D2_press_event();
	}
	value_last = value;
	
	return value;
}

//----------------------------------------------------------------------------
uint8_t get_button_debounced_D3(uint8_t cycles)
{
	static uint8_t value = 0;
	static uint8_t value_last = 0;
	static uint8_t debounce_cntr = 0;
	
	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(BUTTON_D3_PORT, BUTTON_D3_PIN)) // Pressed
	{
		debounce_cntr++;
		if (debounce_cntr >= cycles){
			value = 1;
			debounce_cntr = cycles;
		}
	}
	else	// Raised
	{
		debounce_cntr--;
		if (debounce_cntr <= 0){
			debounce_cntr = 0;
			value = 0; 
		}
	}
	
	if ((value_last == 0) && (value == 1))
	{
		button_debounced_D3_press_event();
	}
	value_last = value;
	
	return value;
}


//----------------------------------------------------------------------------
// cycle through control types
void button_debounced_D2_press_event(void)
{
	control_state++;
	if (control_state > 2) control_state = 0;
}


//----------------------------------------------------------------------------
// toggle gimbal kill state
void button_debounced_D3_press_event(void)
{
	FreeflyAPI.control.gimbal_kill = (FreeflyAPI.control.gimbal_kill == 0) ? 1 : 0;
}


//----------------------------------------------------------------------------
//Mapping function
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//----------------------------------------------------------------------------
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 111111;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

