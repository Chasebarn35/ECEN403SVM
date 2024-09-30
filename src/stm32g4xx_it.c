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
#include "arm_math.h"
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

	if(__HAL_GPIO_EXTI_GET_IT(V_c_Pin)!=RESET){ //TODO POTENTIALLY REMOVE?
		__HAL_GPIO_EXTI_CLEAR_IT(V_c_Pin);
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
	//HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n", 2, 0xFFFF);//LOL LMAO
	if(TIM_GET_ITSTATUS(&htim6,TIM_IT_UPDATE)!=RESET){
		currVec = Mat[0] | Mat[3] << 1 | Mat[6]*3 | Mat[1] << 2 | Mat[4]<<3 | Mat[7]*3 << 2 | Mat[2] << 4 | Mat[5] << 5 | Mat[8]*3 << 4;
		PhaseChange(currVec);
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

		ThetaV+=(60*2*M_PI)/50000;//Reset via V_A interrupt pin
		ThetaC+=(DFreq*2*M_PI)/50000;//DFreq Increment every 50Khz
		if(ThetaC >= (2*M_PI))
			ThetaC = 0;
		switch(triangle){
		case 1:
		triangleWave += 2500/50000;
		if(triangleWave >=1) 
			triangle = 0;
		break;
		case 0:
		triangleWave -= 2500/50000;
                if(triangleWave <=0)
                        triangle = 1;
		break;
		}

		float32_t cos_a,cos_b,cos_c;


		cos_a = arm_cos_f32(ThetaV + _PIdiv3);
		cos_b = arm_cos_f32(ThetaV + 2*_PIdiv3);
		cos_c = arm_cos_f32(ThetaV + 4*_PIdiv3);

		if(cos_a >= cos_b && cos_a >= cos_c){
			Mat[0] = 1;
			Mat[3] = 0;
			Mat[6] = 0;
			V_IN[0] = cos_a;
			V_IN[1] = cos_b;
			V_IN[2] = cos_c;
			virt_a = &VMat[0];
			virt_b = &VMat[3];	
			virt_c = &VMat[6];
		}
		else if(cos_b >= cos_a && cos_b >= cos_c){
			Mat[0] = 0;
			Mat[3] = 1;
			Mat[6] = 0;
			V_IN[0] = cos_b;
			V_IN[1] = cos_c;
			V_IN[2] = cos_a;
			virt_a = &VMat[3];
			virt_b = &VMat[6];
			virt_c = &VMat[0];
		}
		else{
			Mat[0] = 0;
			Mat[3] = 0;
			Mat[6] = 1;
			V_IN[0] = cos_c;//TODO VERIFY THIS STUFF
			V_IN[1] = cos_a;
			V_IN[2] = cos_b;
                        virt_a = &VMat[6];
			virt_b = &VMat[0];
			virt_c = &VMat[3];
		}

		V_AB = _U_IN * (V_IN[0] - V_IN[1]);
		V_BC = _U_IN * (V_IN[1] - V_IN[2]);
		V_AC = _U_IN * (V_IN[0] - V_IN[2]);

		DENOM = V_AB*V_AB + V_AC*V_AC + V_BC*V_BC;
		cosinevalue = arm_cos_f32(ThetaV + _PIdiv3);
		v_ab = -_U_OUT * _SQRT3 * cosinevalue;
		cosinevalue = arm_cos_f32(ThetaV + 2*_PIdiv3);
		v_ac = -_U_OUT * _SQRT3 * cosinevalue;//TODO DETERMINE _U_OUT
		virt_a[0] = 1;
		virt_b[0] = 0;
		virt_c[0] = 0;
		virt_b[1] = ((V_AB-V_BC)*v_ab)/DENOM;//TODO DETERMINE SPEED OF CALCULATIONS
		virt_b[2] = ((V_AB-V_BC)*v_ac)/DENOM;
		virt_c[1] = ((V_BC+V_AC)*v_ab)/DENOM;
		virt_c[2] = ((V_BC+V_AC)*v_ac)/DENOM;
		virt_a[1] = 1 - virt_b[1] - virt_c[1];
		virt_a[2] = 1 - virt_b[2] - virt_c[2];

		Mat[1] = VMat[1] > triangleWave ? 1 : 0;
		Mat[2] = VMat[2] > triangleWave ? 1 : 0;
		Mat[7] = VMat[7] > 1 - triangleWave ? 1 : 0; 
		Mat[8] = VMat[8] > 1 - triangleWave ? 1 : 0; 
		Mat[4] = (!Mat[1] && !Mat[7])?1:0;
		Mat[5] = (!Mat[2] && !Mat[8])?1:0;
		

		//TODO IF Theta
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
				snprintf((char*)txString,88,"Usage:\r\n h\t help \r\n d\t output frequency\r\n m\t matrix\r\n t\t thetaC\r\n f\t change frequency\r\n");
				txStrSize = 88;//TODO VERIFY
			break;
			case 'd':
				snprintf(freqStr,5,"%.2f",DFreq);	
				snprintf((char*)txString,27, "Current frequency:\t %s\r\n", freqStr);
				txStrSize = 26; 
			break;
			case 'f':
				snprintf((char*)txString,24,"Frequency set to: ");
				txStrSize = 18;
				snprintf((char*)rxBuffer, BUFSIZE, "000000");
				isFreqMode = 1;//cheap bool flag
			break;
			case 'c':
				snprintf(freqStr,10,"%.4f",ThetaC);
				snprintf((char*)txString,27, "Current ThetaC:\t %s\r\n", freqStr);
				txStrSize = 26; 
			break;
			case 'm':
				snprintf((char*)txString,35,"[%d %d %d ] \r\n[%d %d %d ] \r\n[%d %d %d ] \r\n\n",Mat[0], Mat[1], Mat[2],Mat[3], Mat[4], Mat[5], Mat[6],Mat[7], Mat[8]);
				txStrSize = 35;	
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
