/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>    // Para printf
#include <string.h>   // Para strcmp, strtok
#include <stdlib.h>   // Para atof, atoi
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	ESTADO_MENU,
	ESTADO_CONFIG_BJT,
	ESTADO_EJECUTANDO_BJT
} Estado_App_t;

// Modos de operación BJT
typedef enum {
	BJT_MODO_ICVB,  // Ic vs Vb (barrer base, fijar colector)
	BJT_MODO_ICVC   // Ic vs Vc (barrer colector, fijar base)
} BJT_Modo_t;

// Configuración de barrido BJT
typedef struct {
	BJT_Modo_t modo;
	float vb_fijo_mv;
	float vc_fijo_mv;
	float v_inicio_mv;
	float v_fin_mv;
	float v_paso_mv;
	uint16_t puntos;
} Config_BJT_t;

// Comandos disponibles
typedef enum {
	CMD_AYUDA,
	CMD_BJT,
	CMD_MODO,
	CMD_ESTABLECER,
	CMD_MOSTRAR,
	CMD_EJECUTAR,
	CMD_VOLVER,
	CMD_DESCONOCIDO
} Comando_ID_t;

// Estructura para la tabla de comandos
typedef struct {
	const char* texto;
	Comando_ID_t id;
} Tabla_Comando_t;


// Tabla de comandos
static const Tabla_Comando_t COMANDOS[] = {
		{"ayuda",      CMD_AYUDA},
		{"bjt",        CMD_BJT},
		{"modo",       CMD_MODO},
		{"establecer", CMD_ESTABLECER},
		{"mostrar",    CMD_MOSTRAR},
		{"ejecutar",   CMD_EJECUTAR},
		{"volver",     CMD_VOLVER}
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUFFER_SIZE 128 // Tamaño del buffer de recepción UART
#define NUM_COMANDOS (sizeof(COMANDOS) / sizeof(COMANDOS[0]))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Definir variables
uint8_t TIMER_ADVISORY_FLAG = 0; // bandera para indicar que el timer ha interrumpido
uint8_t EXTI_AVISORY_FLAG = 0; // bandera para indicar que el EXTI ha interrumpido
uint8_t EXTI_AVISORY_LED_FLAG = 0;
//Se inicia en 0
uint16_t Tarifa = 0; // variable que guarda la tarifa actual
uint8_t Unidad_mil = 0;// variable que guarda las unidades de mil
uint8_t Centenas = 0; // variable que guarda las centenas
uint8_t Decenas = 0; // variable que guarda las decenas
uint8_t Unidades = 0;// variable que guarda las unidades


uint8_t g_uart_rx_buffer[UART_RX_BUFFER_SIZE]; // Buffer DMA
uint8_t g_uart_cmd_buffer[UART_RX_BUFFER_SIZE]; // Buffer para procesar
volatile uint16_t g_uart_cmd_len = 0;
volatile uint8_t g_uart_cmd_ready = 0; // Flag: 1 = comando listo


static Estado_App_t g_estado = ESTADO_MENU;
static Config_BJT_t g_config = {
		.modo = BJT_MODO_ICVB,
		.vb_fijo_mv = 0.0f,
		.vc_fijo_mv = 0.0f,
		.v_inicio_mv = 0.0f,
		.v_fin_mv = 0.0f,
		.v_paso_mv = 0.0f,
		.puntos = 0
};
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void lightNumber(uint8_t number);
static void mostrar_menu_principal(void);
static void mostrar_config_bjt(void);
static Comando_ID_t buscar_comando(const char* texto);
static void analizar_y_ejecutar_comando(uint8_t* buffer, uint16_t tam);
static void despachar_comando(Comando_ID_t id, char* params);
static void manejar_menu(Comando_ID_t id, char* params);
static void manejar_config_bjt(Comando_ID_t id, char* params);
static void manejar_ejecucion_bjt(Comando_ID_t id);
static void tick_barrido_bjt(void);

/* --- Tarea 3: Prototipos de Drivers (Stubs) --- */
extern void setear_voltaje_base_mv(float v);
extern void setear_voltaje_colector_mv(float v);
extern float leer_voltaje_base_mv(void);
extern float leer_voltaje_colector_mv(void);
extern float leer_corriente_colector_ma(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
// TODO: estudiar ṕr que las cosas estan asi


void mostrar_menu_principal(void) {
	const char* menu =
			"\r\n=== Analizador BJT ===\r\n"
			"Comandos disponibles:\r\n"
			"1. bjt - Entrar al menú de configuración BJT\r\n"
			"2. ayuda - Mostrar este menú\r\n"
			"\r\n> ";
	printf(menu);
}

void mostrar_config_bjt(void) {
	printf("\r\n--- CONFIGURACIÓN BJT ---\r\n");
	printf("Modo      : %s\r\n", (g_config.modo == BJT_MODO_ICVB) ? "Ic vs Vb" : "Ic vs Vc");
	printf("Vb fijo   : %.1f mV\r\n", g_config.vb_fijo_mv);
	printf("Vc fijo   : %.1f mV\r\n", g_config.vc_fijo_mv);
	printf("Inicio    : %.1f mV\r\n", g_config.v_inicio_mv);
	printf("Fin       : %.1f mV\r\n", g_config.v_fin_mv);
	printf("Paso      : %.1f mV\r\n", g_config.v_paso_mv);
	printf("Puntos    : %u\r\n",    g_config.puntos);
	printf("Comandos:\r\n");
	printf("  modo icvb | modo icvc\r\n");
	printf("  establecer [vb|vc|inicio|fin|paso|puntos] <valor>\r\n");
	printf("  mostrar | ayuda\r\n");
	printf("  ejecutar\r\n");
	printf("  volver\r\n\r\n> ");
}

static Comando_ID_t buscar_comando(const char* texto) {
	for (size_t i = 0; i < NUM_COMANDOS; i++) {
		if (strcmp(texto, COMANDOS[i].texto) == 0) {
			return COMANDOS[i].id;
		}
	}
	return CMD_DESCONOCIDO;
}

static void analizar_y_ejecutar_comando(uint8_t* buffer, uint16_t tamaño)
{


	char* texto_comando = strtok((char*)buffer, " \r\n");
	char* parametros    = strtok(NULL, ""); // El resto de la línea

	if (texto_comando == NULL) {
		printf("> ");
		return; // Buffer vacío o solo espacios
	}

	Comando_ID_t id = buscar_comando(texto_comando);
	despachar_comando(id, parametros);
}

static void despachar_comando(Comando_ID_t id, char* params) {
	switch (g_estado) {
	case ESTADO_MENU:
		manejar_menu(id, params);
		break;
	case ESTADO_CONFIG_BJT:
		manejar_config_bjt(id, params);
		break;
	case ESTADO_EJECUTANDO_BJT:
		manejar_ejecucion_bjt(id);
		break;
	}
}

static void manejar_menu(Comando_ID_t id, char* params) {
	switch (id) {
	case CMD_AYUDA:
		mostrar_menu_principal();
		break;
	case CMD_BJT:
		g_estado = ESTADO_CONFIG_BJT;
		mostrar_config_bjt();
		break;
	default:
		printf("Comando inválido. Use 'ayuda'.\r\n> ");
		break;
	}
}

static void manejar_config_bjt(Comando_ID_t id, char* params) {
	switch (id) {
	case CMD_AYUDA:
	case CMD_MOSTRAR:
		mostrar_config_bjt();
		break;

	case CMD_MODO:
		if (!params) {
			printf("Uso: modo [icvb|icvc]\r\n> ");
		} else if (strcmp(params, "icvb") == 0) {
			g_config.modo = BJT_MODO_ICVB;
			printf("Modo: Ic vs Vb\r\n> ");
		} else if (strcmp(params, "icvc") == 0) {
			g_config.modo = BJT_MODO_ICVC;
			printf("Modo: Ic vs Vc\r\n> ");
		} else {
			printf("Modo inválido. Use 'icvb' o 'icvc'.\r\n> ");
		}
		break;

	case CMD_ESTABLECER: {
		if (!params) {
			printf("Uso: establecer [param] <valor>\r\n> ");
			break;
		}

		char* param = strtok(params, " ");
		char* valor = strtok(NULL, " \r\n");

		if (!param || !valor) {
			printf("Parámetros incompletos.\r\n> ");
			break;
		}

		if (strcmp(param, "puntos") == 0) {
			g_config.puntos = (uint16_t)atoi(valor);
		} else {
			float v = atof(valor);
			if (strcmp(param, "vb") == 0)         g_config.vb_fijo_mv = v;
			else if (strcmp(param, "vc") == 0)    g_config.vc_fijo_mv = v;
			else if (strcmp(param, "inicio") == 0) g_config.v_inicio_mv = v;
			else if (strcmp(param, "fin") == 0)    g_config.v_fin_mv = v;
			else if (strcmp(param, "paso") == 0)   g_config.v_paso_mv = v;
			else {
				printf("Parámetro '%s' inválido.\r\n> ", param);
				break;
			}
		}
		printf("Actualizado: %s = %s\r\n> ", param, valor);
		break;
	}

	case CMD_EJECUTAR:
		printf("Iniciando barrido...\r\n");
		// Encabezados de la tabla CSV
		printf("Indice,Vb_mV,Vc_mV,Ic_mA\r\n");
		g_estado = ESTADO_EJECUTANDO_BJT;
		// El barrido se ejecuta en tick_barrido_bjt()
		break;

	case CMD_VOLVER:
		g_estado = ESTADO_MENU;
		mostrar_menu_principal();
		break;

	default:
		printf("Comando inválido. Use 'mostrar' o 'ayuda'.\r\n> ");
		break;
	}
}


static void manejar_ejecucion_bjt(Comando_ID_t id) {
   // Durante la ejecución, cualquier comando detiene el barrido
   printf("\r\n¡Barrido abortado por el usuario!\r\n");
   g_estado = ESTADO_CONFIG_BJT;
   mostrar_config_bjt();
}


static void tick_barrido_bjt(void) {
   static uint16_t indice = 0;
   static uint8_t activo = 0;

   // Solo operar en estado de ejecución
   if (g_estado != ESTADO_EJECUTANDO_BJT) {
       activo = 0; // Resetea el estado si salimos de ejecución
       return;
   }

   // Inicializar al entrar
   if (!activo) {
       indice = 0;
       activo = 1;
       // TODO: Inicializar hardware (DAC, ADC, etc.) si es necesario
   }

   // Verificar fin del barrido
   if (g_config.puntos == 0 || (g_config.v_paso_mv == 0 && g_config.puntos > 1)) {
       printf("Error: 'puntos' o 'paso' no configurados.\r\n");
       g_estado = ESTADO_CONFIG_BJT;
       mostrar_config_bjt();
       return;
   }

   if (indice >= g_config.puntos) {
       printf("Barrido completado.\r\n");
       g_estado = ESTADO_CONFIG_BJT;
       mostrar_config_bjt();
       return;
   }

   // Calcular voltaje de barrido
   float v_barrido = g_config.v_inicio_mv + (float)indice * g_config.v_paso_mv;

   // Aplicar voltajes según modo
   float vb_aplicado, vc_aplicado;
   if (g_config.modo == BJT_MODO_ICVB) {
       vb_aplicado = v_barrido;
       vc_aplicado = g_config.vc_fijo_mv;
   } else { // BJT_MODO_ICVC
       vb_aplicado = g_config.vb_fijo_mv;
       vc_aplicado = v_barrido;
   }

   setear_voltaje_base_mv(vb_aplicado);
   setear_voltaje_colector_mv(vc_aplicado);

   // TODO: Implementar "delay" de asentamiento apropiado

   // Medir valores
   float vb_medido = leer_voltaje_base_mv();
   float vc_medido = leer_voltaje_colector_mv();
   float ic_medido = leer_corriente_colector_ma();

   // Transmitir datos en formato CSV
   printf("%u,%.2f,%.2f,%.3f\r\n", indice, vb_medido, vc_medido, ic_medido);

   indice++;
}




void setear_voltaje_base_mv(float v_mv) {
    // TODO: Implementar PWM para Vb
}
void setear_voltaje_colector_mv(float v_mv) {
    // TODO: Implementar PWM para Vc
}
float leer_voltaje_base_mv(void) {
    // TODO: Implementar ADC para Vb
    return 0.0f; // Placeholder
}


float leer_voltaje_colector_mv(void){
	//TODO
	return 0.0f;
}

float leer_corriente_colector_ma(void){
	//TODO
	return 0.0f;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2); // Iniciar TIM2 (Blinky)
	HAL_TIM_Base_Start_IT(&htim3); // Iniciar TIM3 (Refresco Display)

	// Calcular digitos iniciales (para Tarifa = 0)
	Unidad_mil = Tarifa / 1000;
	Centenas = (Tarifa % 1000) / 100;
	Decenas = (Tarifa % 100) / 10;
	Unidades = Tarifa % 10;



	mostrar_menu_principal(); // Mostrar menú al inicio

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{


		if (TIMER_ADVISORY_FLAG) //trabajo del timer dentro del while
		{
			static uint8_t contador = 0;
			contador++;
			if (contador > 3) contador = 0; // reiniciar el contador cada 3 interrupciones
			HAL_GPIO_WritePin(UNIDADES_GPIO_Port, UNIDADES_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DECENAS_GPIO_Port, DECENAS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(CENTENAS_GPIO_Port, CENTENAS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(UNIDADES_MIL_GPIO_Port, UNIDADES_MIL_Pin, GPIO_PIN_SET);

			switch(contador){
			case 0:
				// encender unidades
				lightNumber(Unidades); // llamar a la función para encender el número
				HAL_GPIO_WritePin(UNIDADES_GPIO_Port, UNIDADES_Pin, GPIO_PIN_RESET);

				break;
			case 1:
				// encender decenas
				lightNumber(Decenas); // llamar a la función para encender el número
				HAL_GPIO_WritePin(DECENAS_GPIO_Port, DECENAS_Pin, GPIO_PIN_RESET);
				break;
			case 2:
				// encender centenas
				lightNumber(Centenas); // llamar a la función para encender el número
				HAL_GPIO_WritePin(CENTENAS_GPIO_Port, CENTENAS_Pin, GPIO_PIN_RESET);
				break;
			case 3:
				// encender unidades de mil
				lightNumber(Unidad_mil); // llamar a la función para encender el número
				HAL_GPIO_WritePin(UNIDADES_MIL_GPIO_Port, UNIDADES_MIL_Pin, GPIO_PIN_RESET);
				break;
			default:
				break;
			}
			TIMER_ADVISORY_FLAG = 0; // bajar la bandera de interrupción
		}

		if (EXTI_AVISORY_LED_FLAG)
		{

			// El incremento es 50 (equivale a 5ms con tick de 10kHz)
			htim3.Instance->ARR += 50;

			// El valor de reseteo es 509 (59 + 9 * 50)
			if (htim3.Instance->ARR > 509) // Usar '>' por seguridad
			{
				// El valor base es 59 (equivale a 6ms)
				htim3.Instance->ARR = 59;
			}
			EXTI_AVISORY_LED_FLAG = 0;
		}

		if (EXTI_AVISORY_FLAG) //trabajo del EXTI dentro del while
		{
			EXTI_AVISORY_FLAG = 0; // bajar la bandera de interrupción

			//Revisar el estado del pin DT para determinar la dirección del giro
			if (HAL_GPIO_ReadPin(ENCODER_DATA_GPIO_Port, ENCODER_DATA_Pin) == GPIO_PIN_SET){
				Tarifa++; // incrementar la tarifa
				if (Tarifa > 4095) Tarifa = 0; // reiniciar la tarifa si supera 4095
			} else {
				if (Tarifa > 0) Tarifa--; // decrementar la tarifa
				else if (Tarifa == 0) Tarifa = 4095; // reiniciar si la tarifa es igual a 0
			}
			// actualizar las variables de las cifras de la tarifa
			Unidad_mil = Tarifa / 1000;
			Centenas = (Tarifa % 1000) / 100;
			Decenas = (Tarifa % 100) / 10;
			Unidades = Tarifa % 10;
			//Dentro de la rutina de servicio de interrupción del EXTI no se debe hacer nada más que subir una bandera
			//y hacer el trabajo pesado en el while principal

		}

		if (g_uart_cmd_ready)
		{
		    uint16_t len = g_uart_cmd_len;

		    if (len >= UART_RX_BUFFER_SIZE) {
		        len = UART_RX_BUFFER_SIZE - 1;
		    }

		    memcpy(g_uart_cmd_buffer, (uint8_t*)g_uart_rx_buffer, len);
		    g_uart_cmd_buffer[len] = '\0';

		    g_uart_cmd_ready = 0;
		    g_uart_cmd_len = 0;

		    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_uart_rx_buffer, UART_RX_BUFFER_SIZE);
		    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

		    printf("\r\n");
		    analizar_y_ejecutar_comando(g_uart_cmd_buffer, len);
		}

		/* --- Tarea 3: Máquina de Estados BJT (No bloqueante) --- */
		// Esta función solo hace trabajo si g_estado == ESTADO_EJECUTANDO_BJT
		tick_barrido_bjt();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 59;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, UNIDADES_MIL_Pin|CENTENAS_Pin|DECENAS_Pin|SEG_D_Pin
                          |SEG_C_Pin|SEG_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BLINKY_GPIO_Port, LED_BLINKY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG_A_Pin|UNIDADES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SEG_F_Pin|SEG_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : UNIDADES_MIL_Pin */
  GPIO_InitStruct.Pin = UNIDADES_MIL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UNIDADES_MIL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BLINKY_Pin */
  GPIO_InitStruct.Pin = LED_BLINKY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_BLINKY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CENTENAS_Pin DECENAS_Pin SEG_D_Pin SEG_C_Pin
                           SEG_E_Pin */
  GPIO_InitStruct.Pin = CENTENAS_Pin|DECENAS_Pin|SEG_D_Pin|SEG_C_Pin
                          |SEG_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_DATA_Pin */
  GPIO_InitStruct.Pin = ENCODER_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER_CLK_Pin ENCODER_SW_Pin */
  GPIO_InitStruct.Pin = ENCODER_CLK_Pin|ENCODER_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_A_Pin UNIDADES_Pin */
  GPIO_InitStruct.Pin = SEG_A_Pin|UNIDADES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_F_Pin SEG_B_Pin */
  GPIO_InitStruct.Pin = SEG_F_Pin|SEG_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SEG_G_Pin */
  GPIO_InitStruct.Pin = SEG_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SEG_G_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void lightNumber(uint8_t number) {
	// Definiciones de pines para un display de 7 segmentos (a-g)
	// se puede reducir el codigo apagando todos los segmentos primero y luego encendiendo unicamente
	// los necesarios pero se hizo así por claridad

	switch (number) {
	case 0: // Muestra el número 0
		HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_RESET); // a
		HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, GPIO_PIN_RESET); // b
		HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, GPIO_PIN_RESET); // c
		HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, GPIO_PIN_RESET); // d
		HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, GPIO_PIN_RESET); // e
		HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, GPIO_PIN_RESET); // f
		HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, GPIO_PIN_SET); // g (apagado)
		break;

	case 1: // Muestra el número 1
		HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_SET); // a (apagado)
		HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, GPIO_PIN_RESET); // b
		HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, GPIO_PIN_RESET); // c
		HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, GPIO_PIN_SET); // d (apagado)
		HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, GPIO_PIN_SET); // e (apagado)
		HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, GPIO_PIN_SET); // f (apagado)
		HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, GPIO_PIN_SET); // g (apagado)
		break;

	case 2: // Muestra el número 2
		HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_RESET); // a
		HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, GPIO_PIN_RESET); // b
		HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, GPIO_PIN_SET); // c (apagado)
		HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, GPIO_PIN_RESET); // d
		HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, GPIO_PIN_RESET); // e
		HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, GPIO_PIN_SET); // f (apagado)
		HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, GPIO_PIN_RESET); // g
		break;

	case 3: // Muestra el número 3
		HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_RESET); // a
		HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, GPIO_PIN_RESET); // b
		HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, GPIO_PIN_RESET); // c
		HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, GPIO_PIN_RESET); // d
		HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, GPIO_PIN_SET); // e (apagado)
		HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, GPIO_PIN_SET); // f (apagado)
		HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, GPIO_PIN_RESET); // g
		break;

	case 4: // Muestra el número 4
		HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_SET); // a (apagado)
		HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, GPIO_PIN_RESET); // b
		HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, GPIO_PIN_RESET); // c
		HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, GPIO_PIN_SET); // d (apagado)
		HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, GPIO_PIN_SET); // e (apagado)
		HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, GPIO_PIN_RESET); // f
		HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, GPIO_PIN_RESET); // g
		break;

	case 5: // Muestra el número 5
		HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_RESET); // a
		HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, GPIO_PIN_SET); // b (apagado)
		HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, GPIO_PIN_RESET); // c
		HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, GPIO_PIN_RESET); // d
		HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, GPIO_PIN_SET); // e (apagado)
		HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, GPIO_PIN_RESET); // f
		HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, GPIO_PIN_RESET); // g
		break;

	case 6: // Muestra el número 6
		HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_RESET); // a
		HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, GPIO_PIN_SET); // b (apagado)
		HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, GPIO_PIN_RESET); // c
		HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, GPIO_PIN_RESET); // d
		HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, GPIO_PIN_RESET); // e
		HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, GPIO_PIN_RESET); // f
		HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, GPIO_PIN_RESET); // g
		break;

	case 7: // Muestra el número 7
		HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_RESET); // a
		HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, GPIO_PIN_RESET); // b
		HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, GPIO_PIN_RESET); // c
		HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, GPIO_PIN_SET); // d (apagado)
		HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, GPIO_PIN_SET); // e (apagado)
		HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, GPIO_PIN_SET); // f (apagado)
		HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, GPIO_PIN_SET); // g (apagado)
		break;

	case 8: // Muestra el número 8
		HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_RESET); // a
		HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, GPIO_PIN_RESET); // b
		HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, GPIO_PIN_RESET); // c
		HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, GPIO_PIN_RESET); // d
		HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, GPIO_PIN_RESET); // e
		HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, GPIO_PIN_RESET); // f
		HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, GPIO_PIN_RESET); // g
		break;

	case 9: // Muestra el número 9
		HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_RESET); // a
		HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, GPIO_PIN_RESET); // b
		HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, GPIO_PIN_RESET); // c
		HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, GPIO_PIN_RESET); // d
		HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, GPIO_PIN_SET); // e (apagado)
		HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, GPIO_PIN_RESET); // f
		HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, GPIO_PIN_RESET); // g
		break;

	default: // Apaga todos los segmentos excepto el g (para indicar error)
		HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_SET); // a (apagado)
		HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, GPIO_PIN_SET); // b (apagado)
		HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, GPIO_PIN_SET); // c (apagado)
		HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, GPIO_PIN_SET); // d (apagado)
		HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, GPIO_PIN_SET); // e (apagado)
		HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, GPIO_PIN_SET); // f (apagado)
		HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, GPIO_PIN_RESET); //g
		break;
	}
}



/*
 * CODIOGO PUNTO 3
 *
 *
 *
 * */




void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		if (g_uart_cmd_ready == 0) // Si no hay un comando procesándose
		{
			g_uart_cmd_len = Size;
			g_uart_cmd_ready = 1;

			// Detener DMA para evitar sobreescritura
			HAL_UART_DMAStop(&huart2);
		}
		// Si ya hay un comando (g_uart_cmd_ready = 1), este nuevo se ignora
		// y se reiniciará en el while(1) después de procesar el anterior.
	}
}

/*
 *
 *
 *
 *
 * */


// Callback de TIM: se llama cada vez que vence el periodo (UIF)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3) // Interrupción del display
	{
		TIMER_ADVISORY_FLAG = 1; // Subir la bandera para el while(1)
	}
	else if(htim->Instance == TIM2) // Interrupción del blinky
	{
		HAL_GPIO_TogglePin(LED_BLINKY_GPIO_Port, LED_BLINKY_Pin);
	}
}

// Callback general del EXTI: se llama cuando ocurre una interrupción externa
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Interrupción del giro del encoder
	if(GPIO_Pin == ENCODER_CLK_Pin) // Usar el nombre de CubeMX
	{
		EXTI_AVISORY_FLAG = 1; // Subir la bandera para el while(1)
	}

	// Interrupción del switch (botón) del encoder
	else if (GPIO_Pin == ENCODER_SW_Pin) // Usar el nombre de CubeMX
	{
		EXTI_AVISORY_LED_FLAG = 1;
	}
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
