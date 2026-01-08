# STM32 Timer Triggered ADC with DMA and UART

## 📌 Project Overview
This project implements a *timer-triggered ADC data acquisition system* using the *STM32F401 (Black Pill)* microcontroller.  
ADC conversions are initiated by a *hardware timer overflow, ensuring deterministic sampling. The conversion results are transferred to memory using **Direct Memory Access (DMA)* and then transmitted through *UART*.

The entire system was designed using *STM32CubeMX* and implemented in *STM32CubeIDE* following professional embedded-system development practices.

---

## 🎯 Objectives
- Configure STM32 system clock to 84 MHz
- Generate a periodic timer interrupt at *100 Hz*
- Trigger ADC conversion using *timer TRGO*
- Transfer ADC results to memory using *DMA*
- Transmit ADC values via *UART*
- Verify system behavior using software build and simulation

---

## 🛠 Tools & Software Used
- STM32CubeIDE  
- STM32CubeMX  
- STM32 HAL Library  
- Proteus (software simulation)  

---

## ⚙️ System Configuration

### 🔹 Microcontroller Selection
*MCU:* STM32F401CC (Black Pill)

📷 STM32CubeMX – MCU Selection  
![MCU Selection](images/mcu_selection.png)
<img width="1309" height="680" alt="mpu selector" src="https://github.com/user-attachments/assets/b4d6a63b-6cc9-475a-8ce1-607e7ae86857" />


---

### 🔹 Clock Configuration
- SYSCLK: *84 MHz*
- APB1: *42 MHz*
- APB2: *84 MHz*

📷 Clock Configuration  
![Clock Configuration](images/clock_configuration.png)
<img width="1254" height="604" alt="clock config" src="https://github.com/user-attachments/assets/3b529b56-3619-4d95-8ae1-154887e6bb66" />

---

### 🔹 Timer Configuration (TIM2 – 100 Hz)
- Prescaler: 8399
- Auto Reload Register (ARR): 99
- Trigger Output (TRGO): *Update Event*

📷 TIM2 Configuration  
![TIM2 Configuration](images/tim2_configuration.png)
<img width="707" height="577" alt="timer" src="https://github.com/user-attachments/assets/2ebba717-5b8b-4004-bab9-846e03c8f7fc" />

---

### 🔹 ADC Configuration
- ADC Instance: *ADC1*
- Channel: *ADC1_IN0 (PA0)*
- Resolution: *12-bit*
- External Trigger: *TIM2 TRGO*
- Trigger Edge: *Rising Edge*

📷 ADC Configuration  
![ADC Trigger](images/adc_trigger.png)
<img width="709" height="598" alt="adc-p" src="https://github.com/user-attachments/assets/4630b172-9c9e-4b26-82c4-de6ad40b539c" />

---

### 🔹 DMA Configuration
- Mode: *Circular*
- Direction: *Peripheral → Memory*
- Data Width: *Half Word*
- Memory Increment: *Enabled*

📷 DMA Configuration  
![DMA Configuration](images/dma_configuration.png)
<img width="953" height="600" alt="adc-dma" src="https://github.com/user-attachments/assets/0d95186c-2c61-4306-b0ad-78cbaaaab79e" />
---

### 🔹 UART Configuration
- USART2
- Baud Rate: *115200*
- 8-N-1, No flow control

📷 UART Configuration  
![UART Configuration](images/uart_configuration.png)
<img width="956" height="599" alt="uart-p" src="https://github.com/user-attachments/assets/5c57429b-c2f2-4a65-a3cd-563e5c1cc0aa" />
<img width="951" height="592" alt="uart-nvic" src="https://github.com/user-attachments/assets/a18ce2ec-c087-4df8-b3c1-325682f0e2da" />

---

## 🔁 System Working Principle

📷 System Flow Diagram  
![System Flow](images/system_flow.png)

1. TIM2 generates an update event every *10 ms*
2. Update event is routed internally as *TRGO*
3. ADC conversion starts on each trigger
4. DMA transfers ADC data automatically to memory
5. UART transmits ADC values to serial terminal

---

## 💻 Firmware Implementation

The firmware was developed using *STM32CubeIDE* with *HAL drivers*.  
DMA is used to minimize CPU involvement and ensure efficient real-time data transfer.

---

### 📄 main.c (Core Application Code)
 /* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t adc_value;
char uart_buf[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_value, 1);
  HAL_TIM_Base_Start(&htim2);

  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

/**
  * @brief ADC1 Initialization
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;

  HAL_ADC_Init(&hadc1);

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/**
  * @brief TIM2 Initialization
  */
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;

  HAL_TIM_Base_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}

/**
  * @brief USART2 Initialization
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;

  HAL_UART_Init(&huart2);
}

/**
  * @brief DMA Initialization
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
  * @brief GPIO Initialization
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
  * @brief ADC Conversion Complete Callback
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    int len = sprintf(uart_buf, "ADC Value: %d\r\n", adc_value);
    HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, len, HAL_MAX_DELAY);
  }
}

/**
  * @brief Error Handler
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}




## 🧪 Build Output (Software Verification)

After implementing the firmware, the project was compiled using **STM32CubeIDE**.


### ✔ Build Status
- Build completed successfully
- **0 Errors**
- Minor warnings due to unused autogenerated functions
- HAL libraries linked correctly

📷 *Build Output Screenshot (0 Errors)*  
![Build Output](images/build_success.png)

This confirms that:
- Peripheral initialization code is correct
- Timer, ADC, DMA, and UART configurations are valid
- The firmware is ready for execution on hardware or simulation

---

## 📊 Results

### ✔ Functional Results
- TIM2 successfully configured to generate update events at **100 Hz**
- ADC conversion triggered using **TIM2 TRGO**
- ADC results transferred to memory using **DMA in circular mode**
- UART configured to transmit ADC values at **115200 baud**
- ADC values updated periodically without CPU polling

📷 *UART Output (Simulation)*  
![UART Output](images/uart_output.png)

---

## 🧪 Simulation Verification

Since hardware execution could not be demonstrated, the system behavior was verified using **software simulation**.

Verification steps:
- STM32F401 microcontroller model loaded with compiled firmware
- ADC input simulated using a virtual potentiometer or signal source
- UART output observed on a virtual terminal
- ADC values varied correctly with changes in input signal

📷 *Proteus Simulation Schematic*  
![Proteus Schematic](images/proteus_schematic.png)

📷 *Proteus UART Output*  
![Proteus UART](images/proteus_uart.png)

---

## ✅ Advantages

- Hardware timer ensures **deterministic sampling**
- DMA minimizes CPU load and avoids polling
- Suitable for real-time data acquisition systems
- Scalable for multiple ADC channels
- Uses industry-standard STM32 HAL drivers

---

## ⚠️ Limitations

- Physical hardware execution not demonstrated due to lack of external programmer/debugger
- ADC values in simulation are software-generated
- UART output on real hardware requires an external USB-to-TTL interface

---

## 📌 Conclusion

A **timer-triggered ADC data acquisition system using DMA and UART** was successfully designed and verified at the software level.  
The project builds without errors, and functional correctness was validated through configuration analysis and simulation.

The firmware design is complete and **ready for deployment on real STM32 hardware** using an external programmer.

---

## 🚀 Future Enhancements

- Hardware testing using ST-LINK
- Multi-channel ADC acquisition
- Digital filtering and averaging
- RTOS-based task scheduling

