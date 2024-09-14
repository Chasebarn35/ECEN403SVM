/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_it.h"
#include "algos.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;




/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1)
	{
	}
	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(Short_Pin)!=RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(Short_Pin);
		HAL_GPIO_WritePin(GPIOC,Start_Pin,GPIO_PIN_RESET);//TURNS OFF START
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
	}

	if(__HAL_GPIO_EXTI_GET_IT(V_c_Pin)!=RESET){
		__HAL_GPIO_EXTI_CLEAR_IT(V_c_Pin);
		//TODO MORE WITH V_C
	}
	HAL_GPIO_EXTI_IRQHandler(V_c_Pin);
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(V_a_Pin)!=RESET){
		__HAL_GPIO_EXTI_CLEAR_IT(V_a_Pin);
		ThetaV=0;
	}

	if(__HAL_GPIO_EXTI_GET_IT(V_b_Pin)!=RESET){ //TODO POTENTIALLY REMOVE?
		__HAL_GPIO_EXTI_CLEAR_IT(V_b_Pin);
		//TODO MORE WITH V_B
	}

	//INITIALIZE STARTUP AND WRITE 1 TO START PIN
	if(__HAL_GPIO_EXTI_GET_IT(B1_Pin)!=RESET){
		if(!hasStarted){
			HAL_GPIO_WritePin(LEDPin_GPIO_Port,LEDPin_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin( Start_GPIO_Port, Start_Pin,GPIO_PIN_SET);//TURN ON START
			__HAL_GPIO_EXTI_CLEAR_IT(B1_Pin);
			hasStarted = 1;
		}
	}

	/* USER CODE END EXTI15_10_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(V_b_Pin);
	HAL_GPIO_EXTI_IRQHandler(V_a_Pin);
	HAL_GPIO_EXTI_IRQHandler(B1_Pin);
	/* USER CODE BEGIN EXTI15_10_IRQn 1 */

	/* USER CODE END EXTI15_10_IRQn 1 */
}

/**
 * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
 */
//10KHz interrupt
void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GET_ITSTATUS(&htim6,TIM_IT_UPDATE)!=RESET){
		PhaseChange(SVM[5*currVec + adrT]);
		adrT++;
		if(adrT>=5){
			adrT = 0; //TODO make adrT larger, determine adrT
			currVec++;
			if(currVec >= 2)
				currVec = 0;
					}
		TIM_GET_CLEAR_IT(&htim6,TIM_IT_UPDATE);
	}
}

/**
 * @brief This function handles TIM7 global interrupt.
 */
//50KHz Interrupt
void TIM7_IRQHandler(void)
{

	if(TIM_GET_ITSTATUS(&htim7,TIM_IT_UPDATE)!=RESET){
		//calculate park transformation
		//transform Va or Vb

		//TODO IF Theta
		ThetaV+=(60*M_PI)/50000;//Reset via V_A interrupt pin
		ThetaC+=(DFreq*M_PI)/50000;//DFreq Increment every 50Khz
		if(ThetaC >= (2*M_PI))
			ThetaC = 0;
		TIM_GET_CLEAR_IT(&htim7,TIM_IT_UPDATE);
	}
}



/**
 * @brief This function handles LPUART1 global interrupt.
 */

uint8_t isFreqMode = 0;
uint8_t bufferIndex = 0; 
void LPUART1_IRQHandler(void)
{
	/* USER CODE BEGIN LPUART1_IRQn 0 */

	uint8_t rxData;
	uint8_t txString[100];//TODO VERIFIY THATS THE MAX SIZE
	uint8_t txStrSize;
	char   freqStr[5];
	uint8_t BUFSIZE = 6;
	uint8_t rxBuffer[BUFSIZE];

	if(__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_RXNE)!=RESET){
                rxData = hlpuart1.Instance->RDR;
		if(!isFreqMode){	
		switch(rxData){ 
			case 'h':
				snprintf((char*)txString,96,"Usage:\r\n h\t help command \r\n d\t display current output frequency\r\n f\t change current frequency\r\n");
				txStrSize = 96;
			break;
			case 'd':
				snprintf(freqStr,5,"%.2f",DFreq);	
				snprintf((char*)txString,27, "Current frequency:\t %s\r\n", freqStr);//TODO VERIFIY FUNCTIONALITY
				txStrSize = 26; 
			break;
			case 'f':
				snprintf((char*)txString,24,"Frequency set to: ");
				txStrSize = 18;
				snprintf((char*)rxBuffer, BUFSIZE, "000000");
				isFreqMode = 1;//cheap bool flag
			break;
			default:
				snprintf((char*)txString,18,"Invalid command\n\r");
				txStrSize = 18;
			break;
		}
		HAL_UART_Transmit(&hlpuart1, txString, txStrSize, 0xFFFF);
		}
		else{//TODO READING MULTIPLE RXDATAS UNTIL END OF INPUT
		switch(rxData){
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
                        case '6':
                        case '7':
                        case '8':
                        case '9':
                        case '0':
                        case '.':
			if(bufferIndex < BUFSIZE - 1){
				rxBuffer[bufferIndex++] = rxData;
				rxBuffer[bufferIndex] = '\0';
				HAL_UART_Transmit(&hlpuart1, &rxData, 1, 0xFFFF);	
			}
			break;
                        case '\r':
                        case '\n':
				DFreq = (float32_t)atof((char*)rxBuffer); //TODO VERIFY THIS WORKS
				HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\n\r", 2, 0xFFFF);	
                                isFreqMode = 0;
				bufferIndex = 0;
                        break;
                        default:
				//do nothing 
                        break;
		}



		}
		__HAL_UART_CLEAR_FLAG(&hlpuart1, UART_FLAG_RXNE);
        }
			

	/* USER CODE END LPUART1_IRQn 0 */
	HAL_UART_IRQHandler(&hlpuart1);
	/* USER CODE BEGIN LPUART1_IRQn 1 */

	/* USER CODE END LPUART1_IRQn 1 */
}


int __io_putchar(int ch)
{
    /* Support printf over UART */
    (void) HAL_UART_Transmit(&hlpuart1, (uint8_t *) &ch, 1, 0xFFFF);
    return ch;
}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
