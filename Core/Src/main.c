/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tmc/ic/TMC4671/TMC4671.h"
#include "modbus/mb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

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
	MX_CAN_Init();
	MX_UART4_Init();
	MX_UART5_Init();
	MX_I2C1_Init();
	MX_SPI2_Init();
	MX_USART2_UART_Init();
	MX_ADC2_Init();
	MX_TIM1_Init();
	MX_ADC3_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	/*
	 Enable 74AC541 - needs to be low for activation
	 HAL_GPIO_WritePin(GPIOC, OK_DRV_EN_Pin|EN_OK_Pin|SOFT_START_RELAY_Pin|ENC2_SELECT_Pin|FAULT_LED_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOA, TMC_OK_LED_Pin|RST_TMC_Pin|EN_TMC_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOB, STO3_out_Pin|EM_BRAKE_Pin, GPIO_PIN_RESET);
	 */
	HAL_GPIO_WritePin(GPIOA, TMC_OK_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, FAULT_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, DEBUG_LED_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(GPIOC, SOFT_START_RELAY_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, RST_TMC_Pin | EN_TMC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, EN_TMC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, EN_OK_Pin, GPIO_PIN_RESET); //pull low - disable the

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	int32_t chipInfo = 0;
	int32_t polePairs = 0;

	uint32_t Raw_ADC_temp[5];

	tmc4671_writeInt(0, TMC4671_CHIPINFO_ADDR, 0);
	chipInfo = tmc4671_readInt(0, TMC4671_CHIPINFO_DATA);

	//motor config
	//tmc4671_setMotorType(0, TMC4671_THREE_PHASE_BLDC); //BLDC
	//tmc4671_setPolePairs(0, 4);
	//tmc4671_SimpleABNEncoderInit(1, 8192, 1000, 1000);

	//set PI constants
	//tmc4671_setTorqueFluxPI(0, 100, 0);
	//tmc4671_setVelocityPI(0, 1, 2);
	//tmc4671_setPositionPI(0, 3, 4);

	//================================================================================
	//Prototype motor init and movement START
	//================================================================================

	// Motor type &  PWM configuration
	tmc4671_writeInt(0, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030004); //4 pole pairs
	tmc4671_writeInt(0, TMC4671_PWM_POLARITIES, 0x00000000);	//low and low
	tmc4671_writeInt(0, TMC4671_PWM_MAXCNT, 0x00000F9F);		//3990 for 25kHz
	tmc4671_writeInt(0, TMC4671_PWM_BBM_H_BBM_L, 0x0000FFFF);	//255 for 2.5us
	tmc4671_writeInt(0, TMC4671_PWM_SV_CHOP, 0x00000007);		//SVM disabled

	// ADC configuration
	tmc4671_writeInt(0, TMC4671_ADC_I_SELECT, 0x24000100);
	tmc4671_writeInt(0, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
	tmc4671_writeInt(0, TMC4671_dsADC_MCLK_A, 0x20000000);
	tmc4671_writeInt(0, TMC4671_dsADC_MCLK_B, 0x20000000);
	tmc4671_writeInt(0, TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E);
	tmc4671_writeInt(0, TMC4671_ADC_I0_SCALE_OFFSET, 0x01008001);
	tmc4671_writeInt(0, TMC4671_ADC_I1_SCALE_OFFSET, 0x01008001);

	// ABN encoder settings
	tmc4671_writeInt(0, TMC4671_ABN_DECODER_MODE, 0x00000000);
	tmc4671_writeInt(0, TMC4671_ABN_DECODER_PPR, 0x00002000);		//8192 ppr
	tmc4671_writeInt(0, TMC4671_ABN_DECODER_COUNT, 0x00001CA8);
	tmc4671_writeInt(0, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x000003E8);

	// Limits
	tmc4671_writeInt(0, TMC4671_PID_TORQUE_FLUX_LIMITS, 12000);

	// PI settings
	//tmc4671_writeInt(0, TMC4671_PID_TORQUE_P_TORQUE_I, 0x01000100);
	//tmc4671_writeInt(0, TMC4671_PID_FLUX_P_FLUX_I, 0x01000100);
	tmc4671_setTorqueFluxPI(0, 750, 2);
	tmc4671_setVelocityPI(0, 8000, 2000);
	tmc4671_setPositionPI(0, 80, 60);

	//experimental PI settings

	// ===== ABN encoder test drive =====
	/*
	 //Open loop test drive
	 tmc4671_writeInt(0, TMC4671_OPENLOOP_MODE, 0x00000000);
	 tmc4671_writeInt(0, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);			//open loop velocity mode
	 tmc4671_writeInt(0, TMC4671_OPENLOOP_ACCELERATION, 0x0000003C); 			//60
	 tmc4671_writeInt(0, TMC4671_OPENLOOP_VELOCITY_TARGET, 40);				//rpms
	 tmc4671_writeInt(0, TMC4671_PHI_E_SELECTION, 0x00000002);					//config for open loop
	 tmc4671_writeInt(0, TMC4671_UQ_UD_EXT, 4500);								//value for Parker servomotors
	 HAL_Delay(3000);															//spin the motor
	 tmc4671_writeInt(0, TMC4671_OPENLOOP_VELOCITY_TARGET, 0);					//set 0 rpms
	 HAL_Delay(3000);															//wait for motor to stop
	 tmc4671_writeInt(0, TMC4671_UQ_UD_EXT, 0);								//disable open loop flux

	 // Init encoder (mode 0)
	 tmc4671_writeInt(0, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);			//open loop velocity mode
	 tmc4671_writeInt(0, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);	//reset the offset
	 tmc4671_writeInt(0, TMC4671_PHI_E_SELECTION, 0x00000001);					//phi_e_ext - open loop
	 tmc4671_writeInt(0, TMC4671_PHI_E_EXT, 0x00000000);
	 tmc4671_writeInt(0, TMC4671_UQ_UD_EXT, 5200);
	 HAL_Delay(2000);
	 HAL_GPIO_WritePin(GPIOA, TMC_OK_LED_Pin, GPIO_PIN_RESET);

	 tmc4671_writeInt(0, TMC4671_ABN_DECODER_COUNT, 0x00000000);

	 // Feedback selection
	 tmc4671_writeInt(0, TMC4671_PHI_E_SELECTION, 0x00000003);					//phi_e_abn - use ABN encoder for electrical phase
	 tmc4671_writeInt(0, TMC4671_VELOCITY_SELECTION, 0x00000009);

	 // Switch to torque mode
	 tmc4671_writeInt(0, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000001);

	 // Rotate right
	 tmc4671_setTargetTorque_mA(0, 256, 2000); 									//clarify meaning of the middle variable
	 //tmc4671_writeInt(0, TMC4671_PID_TORQUE_FLUX_TARGET, 0x03E80000);
	 HAL_Delay(5000);
	 HAL_GPIO_WritePin(GPIOC, FAULT_LED_Pin, GPIO_PIN_RESET);

	 // Rotate left
	 tmc4671_setTargetTorque_mA(0, 256, -2000); 									//clarify meaning of the middle variable
	 //tmc4671_writeInt(0, TMC4671_PID_TORQUE_FLUX_TARGET, 0xFC180000);
	 HAL_Delay(5000);
	 HAL_GPIO_WritePin(GPIOF, DEBUG_LED_Pin, GPIO_PIN_RESET);

	 // Stop
	 tmc4671_writeInt(0, TMC4671_PID_TORQUE_FLUX_TARGET, 0x00000000);


	 */
	//reset encoder position
	tmc4671_setActualPosition(0, 0);
	HAL_GPIO_WritePin(GPIOA, TMC_OK_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, FAULT_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, DEBUG_LED_Pin, GPIO_PIN_SET);
	//start new testing procedure

	//velocity mode test drive - crude speed ramp - TO BE TESTED
	/*
	 tmc4671_setTorqueFluxLimit_mA(0, 256, 80*141);							//8A * sqrt(2)
	 tmc4671_writeInt(0, TMC4671_VELOCITY_SELECTION, 0x00000009);				//mechanical velocity selection, standard mode
	 tmc4671_setTargetVelocity(0, 100);
	 HAL_Delay(200);
	 tmc4671_setTargetVelocity(0, 250);
	 HAL_Delay(200);
	 tmc4671_setTargetVelocity(0, 500);
	 HAL_Delay(200);
	 tmc4671_setTargetVelocity(0, 750);
	 HAL_Delay(200);
	 tmc4671_setTargetVelocity(0, 1000);
	 HAL_Delay(5000);
	 tmc4671_setTargetVelocity(0, 750);
	 HAL_Delay(200);
	 tmc4671_setTargetVelocity(0, 500);
	 HAL_Delay(200);
	 tmc4671_setTargetVelocity(0, 250);
	 HAL_Delay(200);
	 tmc4671_setTargetVelocity(0, 0);
	 HAL_Delay(200);
	 */

	//================================================================================
	//Prototype motor init and movement STOP
	//================================================================================
	//info about what to input can be found in TMC4671_register.h
	//================================================================================
	//Prototype ADC temperature measurements
	//================================================================================
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

	ModBus_Init();

	uint32_t timeFor1000msTask = HAL_GetTick();

	while (1) {
		if (HAL_GetTick() - timeFor1000msTask >= 1000) {
			timeFor1000msTask = HAL_GetTick();
			uint32_t chTab[] = { ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
			ADC_CHANNEL_5, ADC_CHANNEL_11 };
			for (int32 i = 0; i < 5; i++) {

				ADC_ChannelConfTypeDef sConfig = { 0 };
				sConfig.Channel = chTab[i];
				sConfig.Rank = ADC_REGULAR_RANK_1;
				sConfig.SingleDiff = ADC_SINGLE_ENDED;
				sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
				if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
					Error_Handler();
				}
				HAL_ADC_Start(&hadc2);
				HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
				Raw_ADC_temp[i] = HAL_ADC_GetValue(&hadc2);
				HAL_ADC_Stop(&hadc2);
			}
			HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		ModBus_Perform();

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL10;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_UART4 | RCC_PERIPHCLK_UART5 | RCC_PERIPHCLK_I2C1
			| RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_ADC34;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
	PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
