/**
  ******************************************************************************
  * File Name          : flash.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#ifndef __FLASH_H
#define __FLASH_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f3xx.h"
#include "stm32f3xx_hal.h"

/* Private define ------------------------------------------------------------*/
#define PageStartAddr   			((uint32_t)0x0008000)   // Start of user Flash area
#define PageEndAddr     			((uint32_t)0x0008800)   // End of user Flash area
#define flashEndPgAddr				((uint32_t)0x0000F800) //End page of the Flash

#define FLASH_KEYR1               ((uint32_t)0x45670123 )
#define FLASH_KEYR2               ((uint32_t)0xCDEF89AB )


/** 
  * @brief Status structures definition  
  */  
typedef enum {FAIL = 0, OK =!FAIL} TestStatus;


/* Exported functions ------------------------------------------------------- */
void FLASH_Unlock(void);
void FLASH_Lock(void);
void FLASH_ClearFlag(uint32_t FLASH_FLAG);
void flashInit(void);
uint32_t getPgStartAddr(uint32_t addr);
void readPage(uint32_t pgStartAddr, uint8_t *pgImage);
uint32_t getAddrInPage(uint32_t addr);
uint32_t modifyPageImage(uint8_t *pgImage, uint32_t pgDataAddrInPage, uint32_t numOfBytes, uint8_t* dataPtr);
int erasePage(uint32_t pgStartAddr);
static void FLASH_Program_HalfWord(uint32_t Address, uint16_t Data);
uint32_t writePage(uint32_t pgStartAddr, uint8_t *pgImage);
uint32_t verifyPage(uint32_t pgStartAddr, uint8_t *pgImage);
uint32_t readFlash(uint32_t addr, uint32_t numOfBytes, uint8_t *dataPtr1);
uint32_t writeFlash(uint32_t addr, uint32_t numOfBytes, uint8_t *dataPtr);

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __FLASH_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
