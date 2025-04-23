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
#include "usb_device.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
#include "stm32f4xx.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	int16_t min_xval;
	int16_t max_xval;
	int16_t min_yval;
	int16_t max_yval;
}calibValues;

typedef struct {
	int16_t xdata;
	int16_t ydata;
	int16_t zdata;
}accelValues;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SAMPLES		100

#define CALIB_CYCLES	50
#define SAMPLE_DELAY    100
#define SCALING			100

#define ZYXDA_BIT 		0x08       //XYZ data available

#define READ			0x80
#define WRITE			0x0

#define CTRL_REG4 		0x20	   //Sensor CTRL_REG4 base address
#define ODR				0x6		   //100Hz output data rate
#define BDU				0x0		   //Block data update continuous
#define AXIS_EN			0x7        //X,Y,Z axis enabled
#define CTRL_REG4_CFG   (ODR << 4)|(BDU << 3)|(AXIS_EN)

#define CTRL_REG5		0x24
#define BW				0x3		   //50Hz bandwidth anti-aliasing filter
#define GSCALE			0x4		   //+/- 16g scale
#define ST				0x0		   //self test
#define SPI_MODE 		0x0        //4-wire mode
#define CTRL_REG5_CFG   (BW << 6)|(GSCALE << 3)|(ST << 1)|(SPI_MODE)

#define OUT_X_LO		0x28
#define OUT_X_HI		0x29
#define OUT_Y_LO		0x2A
#define OUT_Y_HI		0x2B
#define OUT_Z_LO		0x2C
#define OUT_Z_HI		0x2D

#define STATUS			0x27

#define WHO_AM_I		0x0F
#define SENSOR_ID		0x3F


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t flag = 0;			//Variable to store the button flag

//extern the USB handler
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void accelRead(uint8_t *addr, uint8_t *data, uint16_t size);
void accelWrite(uint8_t *reg_config);
void accelInit(void);
calibValues accelCalib(uint32_t cycles, uint32_t sample_time);
accelValues accelGetData(calibValues cv, int32_t scaling);
void sendHIDReport(accelValues av);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch){
	  ITM_SendChar(ch);
	  return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t sensor_id[2] = {(WHO_AM_I|READ), 0x00};
  uint8_t sensor_id_r[2] = {0x00, 0x00};

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
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /*Initialize sensor*/
  accelInit();

  /*checking WHO_AM_I Sensor ID*/
  accelRead(sensor_id, sensor_id_r , 2);
  if(sensor_id_r[1] != SENSOR_ID){
	  printf("Error: cannot connect to sensor");
  }

  /*Calibrate Sensor Min and Max Values*/
  calibValues cv = accelCalib(CALIB_CYCLES, SAMPLE_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  accelValues av = accelGetData(cv, SCALING);

	  sendHIDReport(av);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  __HAL_SPI_ENABLE(&hspi1); // This macro sets the SPE bit
  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SWJ;  // SWO is part of debug port
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void accelRead(uint8_t *addr, uint8_t *data, uint16_t size)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(&hspi1, addr, data, size, 10000);

	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

void accelWrite(uint8_t *reg_config)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, reg_config, 2, 10000);

	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

void accelInit(void)
{
	uint8_t mem_ctrl4[2] = {CTRL_REG4, CTRL_REG4_CFG};
	uint8_t mem_ctrl5[2] = {CTRL_REG5, CTRL_REG5_CFG};
	accelWrite(mem_ctrl4);
	accelWrite(mem_ctrl5);
}

calibValues accelCalib(uint32_t cycles, uint32_t sample_time)
{
	calibValues cv;
	cv.max_xval = 0, cv.max_yval = 0, cv.min_xval = 0, cv.min_yval = 0;
	uint8_t axis_baddr[7] = {(OUT_X_LO|READ), (OUT_X_HI|READ), (OUT_Y_LO|READ),
			(OUT_Y_HI|READ), (OUT_Z_LO|READ), (OUT_Z_HI|READ), 0x00};
	uint8_t axis_data[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	for(int i = 0; i < cycles; i ++){
		accelRead(axis_baddr, axis_data, 7);
	    cv.min_xval = MIN(cv.min_xval, (((int16_t) axis_data[2] << 8)| (int16_t) axis_data[1]));
	    cv.max_xval = MAX(cv.max_xval, (((int16_t) axis_data[2] << 8)| (int16_t) axis_data[1]));
	    cv.min_yval = MIN(cv.min_yval, (((int16_t) axis_data[4] << 8)| (int16_t) axis_data[3]));
	    cv.max_yval = MAX(cv.max_yval, (((int16_t) axis_data[4] << 8)| (int16_t) axis_data[3]));
	    HAL_Delay(sample_time);
	}
	return cv;
}

accelValues accelGetData(calibValues cv, int32_t scaling)
{
	accelValues av;
	uint8_t status[2] = {(STATUS|READ), 0x00};
	uint8_t status_buf[2] = {0x00, 0x00};
	uint8_t axis_baddr[7] = {(OUT_X_LO|READ), (OUT_X_HI|READ), (OUT_Y_LO|READ),
			(OUT_Y_HI|READ), (OUT_Z_LO|READ), (OUT_Z_HI|READ), 0x00};
	uint8_t axis_data[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	/*poll status*/
	while((status_buf[1] & ZYXDA_BIT) == 0){
		accelRead(status, status_buf, 2);
	}

	  //check for new data and running avg
	accelRead(axis_baddr, axis_data, 7);
	av.xdata = ((int16_t) axis_data[2] << 8)| (int16_t) axis_data[1];
	av.ydata = ((int16_t) axis_data[4] << 8)| (int16_t) axis_data[3];
	av.zdata = ((int16_t) axis_data[6] << 8)| (int16_t) axis_data[5];

	if(av.xdata < cv.min_xval) av.xdata = av.xdata - cv.min_xval;
	if(av.xdata > cv.max_xval) av.xdata = av.xdata - cv.max_xval;

	if(av.ydata < cv.min_yval) av.ydata = av.ydata - cv.min_yval;
	if(av.ydata > cv.max_yval) av.ydata = av.ydata - cv.max_yval;

	av.xdata = av.xdata/scaling;
	av.ydata = av.ydata/scaling;

	printf("x, y ,z: %d %d %d\n", av.xdata, av.ydata, av.zdata);

	if(av.xdata > 10){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
	}else{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
	}

	if(av.ydata > 10){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
	}else{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
	}

	if(av.zdata > 1000){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
	}else{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
	}

	return av;

}

void sendHIDReport(accelValues av)
{
	uint8_t report[5];

	report[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);        // Button 1 pressed
	report[1] = av.xdata & 0xFF; //LSB
	report[2] = ((av.xdata & 0xFF00)>>8); //MSB
	report[3] = av.ydata & 0xFF; //LSB
	report[4] = ((av.ydata & 0xFF00)>>8); //MSB

	HAL_Delay(10);
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
	HAL_Delay(10);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	flag ^= 1;
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
