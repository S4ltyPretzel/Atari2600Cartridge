/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Flash_D6_Pin GPIO_PIN_13
#define Flash_D6_GPIO_Port GPIOC
#define Flash_D7_Pin GPIO_PIN_14
#define Flash_D7_GPIO_Port GPIOC
#define Flash_D8_Pin GPIO_PIN_15
#define Flash_D8_GPIO_Port GPIOC
#define Flash_A8_Pin GPIO_PIN_0
#define Flash_A8_GPIO_Port GPIOC
#define Flash_A9_Pin GPIO_PIN_1
#define Flash_A9_GPIO_Port GPIOC
#define Flash_A10_Pin GPIO_PIN_2
#define Flash_A10_GPIO_Port GPIOC
#define Flash_A11_Pin GPIO_PIN_3
#define Flash_A11_GPIO_Port GPIOC
#define Flash_A0_Pin GPIO_PIN_0
#define Flash_A0_GPIO_Port GPIOA
#define Flash_A1_Pin GPIO_PIN_1
#define Flash_A1_GPIO_Port GPIOA
#define Flash_A2_Pin GPIO_PIN_2
#define Flash_A2_GPIO_Port GPIOA
#define Flash_A3_Pin GPIO_PIN_3
#define Flash_A3_GPIO_Port GPIOA
#define Flash_A4_Pin GPIO_PIN_4
#define Flash_A4_GPIO_Port GPIOA
#define Flash_A5_Pin GPIO_PIN_5
#define Flash_A5_GPIO_Port GPIOA
#define Flash_A6_Pin GPIO_PIN_6
#define Flash_A6_GPIO_Port GPIOA
#define Flash_A7_Pin GPIO_PIN_7
#define Flash_A7_GPIO_Port GPIOA
#define Flash_Slot_Pin GPIO_PIN_4
#define Flash_Slot_GPIO_Port GPIOC
#define Flash_G_Pin GPIO_PIN_5
#define Flash_G_GPIO_Port GPIOC
#define BUT1_Pin GPIO_PIN_0
#define BUT1_GPIO_Port GPIOB
#define BUT2_Pin GPIO_PIN_1
#define BUT2_GPIO_Port GPIOB
#define Flash_CS_Pin GPIO_PIN_2
#define Flash_CS_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_10
#define LCD_RST_GPIO_Port GPIOB
#define LCD_CE_Pin GPIO_PIN_12
#define LCD_CE_GPIO_Port GPIOB
#define LCD_SCK_Pin GPIO_PIN_13
#define LCD_SCK_GPIO_Port GPIOB
#define Flash_WE_Pin GPIO_PIN_14
#define Flash_WE_GPIO_Port GPIOB
#define LCD_MOSI_Pin GPIO_PIN_15
#define LCD_MOSI_GPIO_Port GPIOB
#define Flash_RP_Pin GPIO_PIN_6
#define Flash_RP_GPIO_Port GPIOC
#define Flash_RB_Pin GPIO_PIN_7
#define Flash_RB_GPIO_Port GPIOC
#define Flash_D1_Pin GPIO_PIN_8
#define Flash_D1_GPIO_Port GPIOC
#define Flash_D2_Pin GPIO_PIN_9
#define Flash_D2_GPIO_Port GPIOC
#define Buf1_OE_Pin GPIO_PIN_8
#define Buf1_OE_GPIO_Port GPIOA
#define Buf2_OE_Pin GPIO_PIN_11
#define Buf2_OE_GPIO_Port GPIOA
#define Buf3_OE_Pin GPIO_PIN_12
#define Buf3_OE_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_15
#define SD_CS_GPIO_Port GPIOA
#define Flash_D3_Pin GPIO_PIN_10
#define Flash_D3_GPIO_Port GPIOC
#define Flash_D4_Pin GPIO_PIN_11
#define Flash_D4_GPIO_Port GPIOC
#define Flash_D5_Pin GPIO_PIN_12
#define Flash_D5_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOD
#define SD_SCK_Pin GPIO_PIN_3
#define SD_SCK_GPIO_Port GPIOB
#define SD_MISO_Pin GPIO_PIN_4
#define SD_MISO_GPIO_Port GPIOB
#define SD_MOSI_Pin GPIO_PIN_5
#define SD_MOSI_GPIO_Port GPIOB
#define Flash_A_1_Pin GPIO_PIN_6
#define Flash_A_1_GPIO_Port GPIOB
#define BUT3_Pin GPIO_PIN_8
#define BUT3_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_9
#define LCD_DC_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
