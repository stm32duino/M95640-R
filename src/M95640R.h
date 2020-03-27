/**
 ******************************************************************************
 * @file    M95640R.h
 * @author  SRA
 * @version V1.0.0
 * @date    March 2020
 * @brief   Abstract Class of a M95640-R EEPROM
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
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


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __M95640R_H__
#define __M95640R_H__

/* Includes ------------------------------------------------------------------*/

#include "SPI.h"

/* Defines -------------------------------------------------------------------*/
/* EEPROM SPI commands */
#define EEPROM_CMD_WREN    0x06    // Write Enable
#define EEPROM_CMD_WRDI    0x04    // Write Disable
#define EEPROM_CMD_RDSR    0x05    // Read Status Register
#define EEPROM_CMD_WRSR    0x01    // Write Status Register
#define EEPROM_CMD_READ    0x03    // Read from Memory Array
#define EEPROM_CMD_WRITE   0x02    // Write to Memory Array   

/* EEPROM SPI status */
#define EEPROM_STATUS_SRWD    0x80       // Status Register Write Disable
#define EEPROM_STATUS_BP      0x0C       // Block Protect
#define EEPROM_STATUS_WEL     0x02       // Write Enable   
#define EEPROM_STATUS_WIP     0x01       // Write in Progress

/* Typedefs ------------------------------------------------------------------*/

/* Class Declaration ---------------------------------------------------------*/
   
/**
 * Abstract class of a M95640-R EEPROM
 */
class M95640R
{
  public:
    M95640R(SPIClass *spi, int ns);
    void begin(void);
    void end(void);
    void EepromRead(uint16_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
    void EepromWrite(uint16_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
    void EepromWaitEndWriteOperation(void);
    void EepromWriteEnable(void);
    uint8_t EepromStatus(void);
    uint8_t EepromSetSrwd(void);

  private:

    SPIClass *dev_spi;
    int ns_pin;
};

#endif /* __M95640R_H__ */
