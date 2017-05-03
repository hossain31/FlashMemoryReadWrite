/**
  ******************************************************************************
  * File Name          : flash.c
  * Description        : FLASH Interface configuration functions
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
	
	/* Includes ------------------------------------------------------------------*/
#include "flash.h"
#include "stm32f3xx_hal.h"


/**
  * @brief  Unlocks the FLASH control register access
  * @param  None
  * @retval None
  */
void FLASH_Unlock(void)
{
	if((FLASH->CR & FLASH_CR_LOCK) != RESET)
  {
    /* Authorize the FLASH Registers access */
    WRITE_REG(FLASH->KEYR, FLASH_KEYR1);
    WRITE_REG(FLASH->KEYR, FLASH_KEYR2);
  }
}


/**
  * @brief  Locks the FLASH control register access
  * @param  None
  * @retval None
  */
void FLASH_Lock(void)
{
  /* Set the LOCK Bit to lock the FLASH Registers access */
	SET_BIT(FLASH->CR, FLASH_CR_LOCK);
}


/**
  * @brief  Clears the FLASH's pending flags.
  * @param  FLASH_FLAG: specifies the FLASH flags to clear.
  *          This parameter can be any combination of the following values:
  *            @arg FLASH_FLAG_EOP: FLASH End of Operation flag 
  *            @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag 
  *            @arg FLASH_FLAG_PGPERR: FLASH Programming Parallelism error flag
  *            @arg FLASH_FLAG_BSY: FLASH Programming Busy flag
  * @retval None
  */
void FLASH_ClearFlag(uint32_t FLASH_FLAG)
{
  /* Check the parameters */
  assert_param(IS_FLASH_CLEAR_FLAG(FLASH_FLAG));
  
  /* Clear the flags */
  FLASH->SR = FLASH_FLAG;
}


/**
  * @brief  Initialize the Flash memory to erase/write
  */
void flashInit(void)
{ 
  /* Unlock the Flash memory */
  FLASH_Unlock();;

	/* Clear pending flags (if any) */  
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR | FLASH_FLAG_BSY);
}


/**
  * @brief  Get the flash page start address
  */
uint32_t getPgStartAddr(uint32_t addr)
{
	addr = addr & ~0x7FF;
	return addr;
}


/**
  * @brief  Read flash page from a given address
  */
void readPage(uint32_t pgStartAddr, uint8_t *pgImage)
{
	for (int i=0; i<2048; i++)
	{
		*(pgImage++) = *(uint8_t*) pgStartAddr++;
	}
}


/**
  * @brief  Get the page address in Page (pgImage) that we gonna edit
 */
uint32_t getAddrInPage(uint32_t addr)
{
	addr=(addr-PageStartAddr);
	return addr;
}


/**
  * @brief  Modify the desired location of the buffer(pgImage)
  */
uint32_t modifyPageImage(uint8_t *pgImage, uint32_t pgDataAddrInPage, uint32_t numOfBytes, uint8_t* dataPtr)
{
	pgImage = pgImage + pgDataAddrInPage;
	for (int i=0; i<numOfBytes; i++)
	{
		*pgImage++ = *dataPtr++;
	}	
	return (0); //OK = 0
}


/**
  * @brief  Erase the specified FLASH memory page
  * @param  pgStartAddr FLASH page to erase     
  * @retval OK or FAIL
  */
int erasePage(uint32_t pgStartAddr)
{
	flashInit();
	
	static FLASH_EraseInitTypeDef EraseInitStruct; //Variable used for Erase procedure
  uint32_t NbOfPages = 0x00000001;
	uint32_t PageError = 0xFFFFFFFF;
   	
	//Fill EraseInit structure
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = pgStartAddr;
	EraseInitStruct.NbPages     = NbOfPages;
	
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{
		return(1); //FAIL = 1
	}
	
	FLASH_Lock(); //Lock the Flash to disable the flash control register access
	return (0); //OK = 0
}


/**
  * @brief  Erase the specified FLASH memory page
  * @param  pgStartAddr FLASH page to erase     
  * @retval None
  */
/*	
int erasePage(uint32_t pgStartAddr)
{
	flashInit();
	FLASH_PageErase(pgStartAddr);
	FLASH_Lock(); //Lock the Flash to disable the flash control register access
}*/


/**
  * @brief  Write HalfWord function
  */
static void FLASH_Program_HalfWord(uint32_t Address, uint16_t Data)
{
    /* Proceed to program the new data */
    SET_BIT(FLASH->CR, FLASH_CR_PG);

  /* Write data in the address */
  *(__IO uint16_t*)Address = Data;
	//return (0);
}


/**
  * @brief  Write flash page from the pgStartAddr address
  */
uint32_t writePage(uint32_t pgStartAddr, uint8_t *pgImage)
{	
	flashInit(); //initialize the flash for writing

	for (int i=0; i<2048; ) //PageEndAddr=0x00008800, PageStartAddr=0x00008000, //2KB
	{
		FLASH_Program_HalfWord(pgStartAddr+i, pgImage[i+1]*256 + pgImage[i]);
		i= i + 2;
	}

	FLASH_Lock();
	
	return (0); //OK
}


/**
  * @brief  Verify the newly written page in flash from the buffer pgImage
	* MemoryProgramStatus = OK : data programmed correctly
  * MemoryProgramStatus = FAIL : data programmed incorrectly
	* status : count how many times data programmed incorrectly
  */
uint32_t verifyPage(uint32_t pgStartAddr, uint8_t *pgImage)
{
	int status = 0;
	__IO TestStatus MemoryProgramStatus = OK;
	
	for (int i=0; i<2048; i++)
	{    
	if (*((uint8_t*)(pgStartAddr++)) != pgImage[i])
		{
			MemoryProgramStatus = FAIL;
			status++;  
		}
	}
	return status++;
}


/**
  * @brief  Read any address from the flash memory
  */
uint32_t readFlash(uint32_t addr, uint32_t numOfBytes, uint8_t *dataPtr1)
{	
	/* If the address is too big, Error */
	if (addr >= flashEndPgAddr) //flashEndPgAddr= 0x0000F800
  return (1); //FAIL

	/* Read the desired location of the flash memory */
	for (int i=0; i<numOfBytes; i++)
  {
		*(dataPtr1++) = *(uint8_t*) addr++;
  }
	return (0); //OK
}


/**
  * @brief  Write the edited page into the flash memory
  */
uint32_t writeFlash(uint32_t addr, uint32_t numOfBytes, uint8_t *dataPtr)
{
	extern uint8_t pgImage[4096];
	uint8_t data[4]={0xAB, 0xCD, 0xEF, 0x24};
	uint32_t pgStartAddr;
	uint32_t pgDataAddrInPage;
	
	pgStartAddr = getPgStartAddr(addr);
	readPage(pgStartAddr, pgImage);
	pgDataAddrInPage= getAddrInPage(addr);
	modifyPageImage(pgImage, pgDataAddrInPage, 4, data);
	erasePage(pgStartAddr);
	writePage(pgStartAddr, pgImage);
	verifyPage(pgStartAddr, pgImage);
	return (0); //OK
	
}
