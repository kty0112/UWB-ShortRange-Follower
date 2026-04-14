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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stm32f4xx_hal.h>
#include "examples_defines.h"
#include <port.h>
#include <string.h>           // strlen 함수 사용
#include <stdlib.h>           // 추가: malloc, free 등을 위해
#include "ds_twr.h"           // custom code 헤더
#include "ds_twr_fuc.h"        // custom code 함수 헤더

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

SPI_HandleTypeDef   *hcurrent_active_spi=&hspi1;//Is the current active SPI pointer - 1 or 2
uint16_t            pin_io_active_spi=DW_NSS_Pin;//CS IO for SPI. Default SPI1
GPIO_PinState       SPI_CS_state=GPIO_PIN_RESET;//Determine the CS for the IO
host_using_spi_e    host_spi = SPI_1;

SPI_HandleTypeDef   hspi4;

uint8_t anchor_macro = 0; // 이 앵커의 번호 (ds_twr.h에 정의 필요)
uint8_t tag_macro = 1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define UNIT_TEST 0
#define WAIT_FOR_USB_CDC 0

uint8_t imu_rx_buffer[IMU_DATA_SIZE];
uint8_t latest_imu_data[IMU_DATA_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

// extern example_ptr example_pointer;

extern int unit_test_main(void);
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn test_run_info()
 *
 * @brief  This gets run info from a test and sends it through virtual COM port.
 *
 * @param data - Message data, this data should be NULL string.
 *
 * output parameters
 *
 * no return value
 */
void test_run_info(unsigned char *data)
{
    uint16_t data_length;

    data_length = strlen((const char *)data);
//    CDC_Transmit_FS(data, data_length); /*Transmit the data through USB - Virtual port*/
//    CDC_Transmit_FS((uint8_t *)"\n\r", 2); /*Transmit end of line through USB - Virtual port*/
    HAL_UART_Transmit(&huart3, (uint8_t *)data, data_length, HAL_MAX_DELAY); /* Transmit the data via UART3 */
    HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

}

void raspi_send(unsigned char *data)
{
    uint16_t data_length;

    data_length = strlen((const char *)data);
//    CDC_Transmit_FS(data, data_length); /*Transmit the data through USB - Virtual port*/
//    CDC_Transmit_FS((uint8_t *)"\n\r", 2); /*Transmit end of line through USB - Virtual port*/
    HAL_UART_Transmit(&huart3, (uint8_t *)data, data_length, HAL_MAX_DELAY); /* Transmit the data via UART3 */
    HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

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

//  build_examples();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* disable IRQ from DW3000*/
  port_DisableEXT_IRQ();

  /*Sleep to wait for USB CDC to initialise with host*/
//  #ifdef WAIT_FOR_USB_CDC
//  Sleep(5000);
//  #endif
  /*
   * DW_RESET_Pin has been configured by CubeMx as Exti0 line
   * Reconfigure the line as Input
   */
  setup_DWICRSTnIRQ(0);

  if (UNIT_TEST)
  {
	  unit_test_main();
  }
  else
  {

	  // Run the selected example as selected in example_selection.h
	  // example_pointer();

    	  // Run the selected example as selected in example_selection.h
	  // example_pointer();



    // responder_init();  // 앵커 시작

    // initiator_init();  // 태그 시작


	  raspi_send((unsigned char *)"\r\n======================\r\n");
	      raspi_send((unsigned char *)"  STM32 BOOT SUCCESS! \r\n");
	      raspi_send((unsigned char *)"======================\r\n");

    // UART6 에러 플래그 초기화
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    __HAL_UART_CLEAR_NEFLAG(&huart6);
    __HAL_UART_CLEAR_FEFLAG(&huart6);
    __HAL_UART_FLUSH_DRREGISTER(&huart6);

    // IMU 데이터 백그라운드 수신 시작 (인터럽트 방식 - 1바이트씩 정밀 파싱)
    extern uint8_t imu_rx_byte;
    HAL_UART_Receive_IT(&huart6, &imu_rx_byte, 1);

    if (anchor_macro == 0) {

      initiator_init();

    }
    else if (anchor_macro == 1){

      // 앵커 좌표 설정 (앵커 1), 원점
      pos_anchors[0].x = 0.0;
      pos_anchors[0].y = 0.0;

      responder_init();

    }
    else if (anchor_macro >= 2){

      initiator_map_init();

      responder_init();

    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* TIM1_CC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */
uint8_t imu_rx_byte;
static uint8_t packet[11];
static uint8_t pkt_idx = 0;
static int16_t acc[3] = {0};
static int16_t angle[3] = {0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    packet[pkt_idx++] = imu_rx_byte;
    
    if (packet[0] != 0x55) {
        pkt_idx = 0; // 헤더가 파손되었으면 초기화
    }
    else if (pkt_idx == 11) {
        if (packet[1] == 0x51) { // Acceleration packet
            acc[0] = (int16_t)((packet[3]<<8) | packet[2]);
            acc[1] = (int16_t)((packet[5]<<8) | packet[4]);
            acc[2] = (int16_t)((packet[7]<<8) | packet[6]);
        } 
        else if (packet[1] == 0x53) { // Angle packet
            angle[0] = (int16_t)((packet[3]<<8) | packet[2]); // Roll
            angle[1] = (int16_t)((packet[5]<<8) | packet[4]); // Pitch
            angle[2] = (int16_t)((packet[7]<<8) | packet[6]); // Yaw
            
            // UWB 전송용 버퍼에 정리해서 담기 (Accel X,Y,Z + Angle Roll,Pitch,Yaw)
            memcpy(&latest_imu_data[0], &acc[0], 2);
            memcpy(&latest_imu_data[2], &acc[1], 2);
            memcpy(&latest_imu_data[4], &acc[2], 2);
            memcpy(&latest_imu_data[6], &angle[0], 2);
            memcpy(&latest_imu_data[8], &angle[1], 2);
            memcpy(&latest_imu_data[10], &angle[2], 2);
        }
        pkt_idx = 0;
    }
    
    // 다음 1바이트 수신 대기
    HAL_UART_Receive_IT(&huart6, &imu_rx_byte, 1);
  }
}

// 오버런 발생 시 강제로 죽지 않고 다시 수신 재개하도록 약하게 처리
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_FLUSH_DRREGISTER(huart);
    uint32_t isrflags = huart->Instance->SR; // 읽어서 플래그 강제 클리어
    (void)isrflags;
    HAL_UART_Receive_IT(&huart6, &imu_rx_byte, 1);
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
#ifdef USE_FULL_ASSERT
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
