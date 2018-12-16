/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// Soglie minime per la presenza di una gamba sui sensori di flessione
#define FLEX_DESTRA_TH 50
#define FLEX_SINISTRA_TH 50

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
uint32_t getPosizione(float,float,float,float);
void setCoordinate(float ratio_up,float ratio_down,float ratio_left,float ratio_right);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t rawValue[7];
uint32_t flex_center, flex_leg_sx, flex_leg_dx, right, up, down, left;	// Sensori
long int  perc , u_diff, d_diff, dx_diff, sx_diff, lateral_unb, frontal_unb, u_diff_ref, d_diff_ref,dx_diff_ref, sx_diff_ref;
uint32_t l_tresh = 300;
uint32_t h_tresh = 600;
float ratio_up, ratio_down, ratio_right, ratio_left, fright, fleft, fup, fdown, x_index, y_index;
int state = 0;
int init = 1;
char msg[40];
float max_value = 250;
int flag=0;
float offset_destra=0,offset_sinistra=0;
int flag_motore=0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM10_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  // DMA : i valori dei 6 sensori saranno disponibili nel vettore rawValue
  HAL_ADC_Start_DMA (&hadc1, (uint32_t *) rawValue, 7);
  HAL_TIM_Base_Init(&htim2); //Timer principale : ad ogni overflow elaboriamo i dati dai sensori
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1); // Timer PWM per il motore

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Viene acceso un led diverso in base al valore della variabile di stato.
  // La variabile di stato viene calcolata nella callback dell'interruzione del Timer 2
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  switch(state){
	  // Postura corretta : viene acceso il LED verde ed il motore non viene attivato
		  case 0:
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
			  __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,0);
			  break;
	 // Postura intermedia : viene aceso il LED giallo ed il motore non viene atttivato
		  case 1:
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
			  __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,0);
			  break;
	 // Postura scorretta : viene acceso il LED rosso e attivato il motore
		  case 2:
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
			  float fflex_leg_sx = (float)flex_leg_sx;
			  float fflex_leg_dx = (float)flex_leg_dx;
			  // Se la postura è scorretta (vedi nella callback quando flag_motore viene posto ad 1)
			  // il motore viene attivato per riportare il
			  // pad in posizione centrale settando il dutycycle al 7.5%
			  if(flag_motore==1){
				  __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,23);
			  }
			  else{
				  __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,0);

			  }
			  break;

	  }

  }
  /* USER CODE END 3 */

}

// Ogni 0.5 secondi elaboriamo i dati proveniente dai sensori per calcolare la variabile di
// stato (verificare se la postura è corretta o meno), stimare la posizione e inviare i dati
// tramite UART
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim){
// This callback is automatically called by the HAL on the UEV event
	if(htim->Instance == TIM2){

		uint32_t posizione; // ID posizione stimata
		uint32_t data[5]; // Vettore di dati da inviare tramite UART (4 sensori + ID posizione)

		// I valori dei 6 sensori sono contenuti nel vettore rawValue usando il DMA
		right = rawValue[0]+1;
		up = rawValue[1]+1;
		down = rawValue[2]+1;
		left = rawValue[3]+1;
		flex_leg_sx =rawValue[5];
		flex_leg_dx =rawValue[6];

		float fflex_leg_sx = (float)flex_leg_sx;
		float fflex_leg_dx = (float)flex_leg_dx;
		fflex_leg_sx=fflex_leg_sx-offset_sinistra;
		fflex_leg_dx=fflex_leg_dx-offset_destra;
		// Per i primi 12*0.5 secondi dopo l'accensione (quando non vi è seduto nessuno)
		// calcoliamo l'offset da sottrarre al valore ricevuto dai sensori di flessione
		// per essere sicuro che sia 0 quando i sensori non sono piegati e diverso da 0
		// nel caso contrario
		if(flag<12){
			flag++;
		}else if(flag==12){
			flag=13;
			offset_destra=fflex_leg_dx;
			offset_sinistra=fflex_leg_sx;
		}

		fright = (float)right;
		fleft = (float)left;
		fup = (float)up;
		fdown = (float)down;
		// Calcoliamo il rapporto tra i valori dei sensori, da cui ricaveremo le coordinate
		// del baricentro
		ratio_up = fup/fdown;
		ratio_down = fdown/fup;
		ratio_right = fright/fleft;
		ratio_left = fleft/fright;

		// Calcoliamo le coordinate del baricentro : la funzione quindi modicficherà le
		// variabili globali x_index e y_index (che sono appunto le coordinate del baricentro)
		setCoordinate(ratio_up,ratio_down,ratio_left,ratio_right);

		/*sprintf(msg, "avanti: %Ld\r\n",up);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "dietro: %Ld\r\n",down);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "destra: %Ld\r\n",right);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "sinistra: %Ld\r\n",left);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);*/

		// Baricentro sbilanciato troppo a destra o sinistra : la variabile flag_motore
		// farà attivare il motore nel while
		if((x_index < -15 ) ||(x_index>15) ){
			flag_motore=1;
		}
		else{
			flag_motore=0;
		}
		// Baricentro nella zona rossa : posizione scorretta quindi la variabile di stato è 2
		// e verrà accesso il LED rosso
		if((x_index < -15 ) ||(x_index>15) || (y_index > 20 ) || fflex_leg_dx<FLEX_DESTRA_TH || fflex_leg_sx<FLEX_SINISTRA_TH){
			state = 2;
		}
		// Baricentro nella zona gialla : posizione intermedia quindi la variabile di stato è 1
		// e verrà accesso il LED giallo
		else if((x_index > -15 && x_index < -5) || ((x_index > 5 && x_index < 15)) || (y_index > 5 && y_index<20)){
			state = 1;
		}
		// Baricentro nella zona verde : posizione corretta
		// verrà accesso il LED verde
		else{
			state = 0;
		}

		// Le coordinate e i valori dei sensori di flessione vengono passati a getPosizione
		// che stimerà la posizione assunta e assegnerà il rispettivo valore alla variabile
		// globale posizione
		posizione=getPosizione(x_index,y_index,fflex_leg_dx,fflex_leg_sx);

		// I 4 valori dei sensori di peso e l'ID della posizione vengono messi in un vettore
		// e inviati con l'UART
		data[0]=up;
		data[1]=down;
		data[2]=left;
		data[3]=right;
		data[4]=posizione;
		HAL_UART_Transmit(&huart6,(uint32_t*)data,20,HAL_MAX_DELAY);

		/*sprintf(msg, "ratio_right: %.7f\r\n",ratio_right);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "ratio_left: %.7f\r\n",ratio_left);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "ratio_up: %.7f\r\n",ratio_up);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "ratio_down: %.7f\r\n",ratio_down);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);*/

		// DEBUG
		sprintf(msg, "------------------------------------------------\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "ascissa: %.7f\r\n",x_index);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "ordinata: %.7f\r\n",y_index);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "flex_destra: %.7f\r\n",fflex_leg_dx);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "flex_sinistra: %.7f\r\n",fflex_leg_sx);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		/*sprintf(msg, "OFFSET DESTRA : %.7f\r\n",offset_destra);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "OFFSET SINISTRA : %.7f\r\n",offset_sinistra);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);*/

		sprintf(msg, "POSIZIONE : %d\r\n",posizione);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "FLAG MOTORE : %d\r\n",flag_motore);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);


	}
}

// Calcola le coordinate x e y del baricentro. I valori vanno da -250 a 250 (max_value)
void setCoordinate(float ratio_up,float ratio_down,float ratio_left,float ratio_right){
	// CALCOLO ORDINATA
	if(ratio_up >= ratio_down){
		if(ratio_up > max_value){
			y_index = max_value;
		}
		else{
			y_index = ratio_up;
		}
	}else{
		if(ratio_down > max_value){
			y_index = -max_value;
		}
		else{
			y_index = -ratio_down;
		}
	}
	//CALCOLO ASCISSA
	if(ratio_right >= ratio_left){
				if(ratio_right > max_value){
					x_index = max_value;
				}
				else{
					x_index = ratio_right;
				}
			}else{
				if(ratio_left > max_value){
					x_index = -max_value;
				}
				else{
					x_index = -ratio_left;
				}
			}

}

// Calcolo l'ID della posizione stimata. E' la somma di due interi : una cifra associata alla
// zona del barientro ed una per la posizione delle gambe.
// Consulta la documentazione per vedere gli ID possibili e le posizioni associate ad essi.
uint32_t getPosizione(float x,float y,float gamba_destra,float gamba_sinistra){
	uint32_t zona,gambe;
	// Controllo della Zona

	// Zona 0
	if((x>-5 && x<5) && (y>-5 && y<5)){
		zona=0;
	}
	// Zona 1
	else if((x>-5 && x<5)&& (y<-5)){
		zona=1;
	}
	// Zona 2
	else if((x<-5)&&(y<-5)){
		zona=2;
	}
	// Zona 3
	else if((y<-5)&&(x>5)){
		zona=3;
	}
	// Zona 4
	else if((x>-5 && x<5)&&(y>5)){
		zona=4;
	}
	// Zona 5
	else if((x<-5)&&(y>-5)){
		zona=5;
	}
	// Zona 6
	else if((x>5)&&(y>-5)){
		zona=6;
	}

	// Gambe Giu
	if((gamba_destra>FLEX_DESTRA_TH)&&(gamba_sinistra>FLEX_SINISTRA_TH)){
		gambe=10;
	}
	// Gamba Destra Alzata
	else if((gamba_destra<FLEX_DESTRA_TH)&&(gamba_sinistra>FLEX_SINISTRA_TH)){
		gambe=20;
	}
	// Gamba Sinistra Alzata
	else if((gamba_destra>FLEX_DESTRA_TH)&&(gamba_sinistra<FLEX_SINISTRA_TH)){
		gambe=30;
	}
	// Gambe Alzate
	else{
		gambe=40;
	}
	return zona+gambe;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

	  RCC_OscInitTypeDef RCC_OscInitStruct;
	  RCC_ClkInitTypeDef RCC_ClkInitStruct;

	    /**Configure the main internal regulator output voltage
	    */
	  __HAL_RCC_PWR_CLK_ENABLE();

	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	    /**Initializes the CPU, AHB and APB busses clocks
	    */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = 16;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	    /**Initializes the CPU, AHB and APB busses clocks
	    */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig;
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the analog watchdog 
    */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_1;
  AnalogWDGConfig.ITMode = DISABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 319;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim10);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
