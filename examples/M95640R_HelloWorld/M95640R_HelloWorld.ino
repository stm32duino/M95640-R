/**
 ******************************************************************************
 * @file    M95640R_HelloWorld.ino
 * @author  SRA
 * @version V1.0.0
 * @date    March 2020
 * @brief   Arduino test application for the STMicrolectronics
 *          Sub-1 GHz RF expansion board based on M95640-R EEPROM.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
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

#include "SPI.h"
#include "M95640R.h"

#define SerialPort Serial

typedef enum
{
  RANGE_EXT_NONE = 0,
  RANGE_EXT_SKY_66100_11 = 1,
  RANGE_EXT_SKYWORKS_SE2435L = 2,
  RANGE_EXT_SKYWORKS_SKY66420 = 3,
  RANGE_EXT_CUSTOM = 4
} RangeExtType;

SPIClass *devSPI;
M95640R *myM95640R;

uint8_t EepromIdentification(void);
void read_eeprom_content(uint32_t *s_frequency, uint32_t *s_RfXtalFrequency, RangeExtType *s_RfRangeExtender);
uint32_t S2LPGetFrequencyBand(uint8_t s_RfModuleBand);


/* Setup ---------------------------------------------------------------------*/

void setup() {
  uint32_t s_frequency = 868000000;
  uint32_t s_RfXtalFrequency = 50000000;
  RangeExtType s_RfRangeExtender = RANGE_EXT_NONE;

  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Put S2-LP in Shutdown
  pinMode(D7, OUTPUT);
  digitalWrite(D7, HIGH);

  // Initialize SPI
  devSPI = new SPIClass(D11, D12, D3);
  devSPI->begin();

  // Initialize M95640-R
  myM95640R = new M95640R(devSPI, D5);
  myM95640R->begin();

  /* Auto recognize FKI or X-NUCLEO daughter boards */
  if(EepromIdentification())
  {
    Serial.println("EEPROM present");
    read_eeprom_content(&s_frequency, &s_RfXtalFrequency, &s_RfRangeExtender);
  } else
  {
    Serial.println("EEPROM not present");
  }

  Serial.print("s_frequency=");
  Serial.print(s_frequency);
  Serial.print(", s_RfXtalFrequency=");
  Serial.print(s_RfXtalFrequency);
  Serial.print(", s_RfRangeExtender=");
  Serial.println(s_RfRangeExtender);
}

/* Loop ----------------------------------------------------------------------*/

void loop() {
  
}

uint8_t EepromIdentification(void)
{
  uint8_t status=0;

  status = myM95640R->EepromStatus();

  if((status&0xF0) == EEPROM_STATUS_SRWD) {
    /* If it is EEPROM_STATUS_SRWD => OK, the EEPROM is present and ready to work */
    status=1;
  }
  else
  {
    myM95640R->EepromWriteEnable();
    delay(10);
    /* Else the bit may be not set (first time we see this EEPROM), try to set it*/
    status = myM95640R->EepromSetSrwd();
    delay(10);
    /*check again*/
    status = myM95640R->EepromStatus();

    if((status&0xF0) == EEPROM_STATUS_SRWD) { // 0xF0 mask [SRWD 0 0 0]
      /* If it is EEPROM_STATUS_SRWD => OK, the EEPROM is present and ready to work */
      status=1;
    }
    else
    {
      /* Else no EEPROM is present */
      status = 0;
    }
  }

  return status;
}

void read_eeprom_content(uint32_t *s_frequency, uint32_t *s_RfXtalFrequency, RangeExtType *s_RfRangeExtender)
{
  float foffset = 0;
  uint8_t tmpBuffer[32];
  uint8_t s_RfModuleBand = 0;
  int32_t xtal_comp_value = 0;

  /* Read the EEPROM */
  myM95640R->EepromRead(0x0000, 32, tmpBuffer);

  /* Data in EEPROM is not valid ... */
  if(tmpBuffer[0]==0 || tmpBuffer[0]==0xFF) {
    *s_RfXtalFrequency = 50000000;

    /* If EEPROM fails, set no EXT_PA by default */
    *s_RfRangeExtender = RANGE_EXT_NONE;

    return;
  }

  switch(tmpBuffer[1]) {
  case 0:
    *s_RfXtalFrequency = 24000000;
    break;
  case 1:
    *s_RfXtalFrequency = 25000000;
    break;
  case 2:
    *s_RfXtalFrequency = 26000000;
    break;
  case 3:
    *s_RfXtalFrequency = 48000000;
    break;
  case 4:
    *s_RfXtalFrequency = 50000000;
    break;
  case 5:
    *s_RfXtalFrequency = 52000000;
    break;
  default:
    *s_RfXtalFrequency = 50000000;
    break;
  }

  s_RfModuleBand = tmpBuffer[3];

  myM95640R->EepromRead(0x0021,4,tmpBuffer);

  for(uint8_t i=0;i<4;i++)
  {
    ((uint8_t*)&foffset)[i]=tmpBuffer[3-i];
  }

  xtal_comp_value = 0;

  /* foffset is a value measured during manufacturing as follows:
  *
  * foffset = fnominal-fmeasured.
  * To compensate such value it should be reported to xtal freq
  * and then subtracted
  *
  */
  if (foffset != 0xFFFFFFFF) {
    uint32_t frequency = S2LPGetFrequencyBand(s_RfModuleBand);

    if (frequency != 0)
    {
	  uint32_t xtal_frequency = *s_RfXtalFrequency;

	  /* This is the value to be added to the xtal nominal value
	  to compensate the xtal offset */
	  xtal_comp_value = (int32_t) ((xtal_frequency*(-foffset))/frequency);

      *s_frequency = frequency;
    }
  }

  *s_RfXtalFrequency = *s_RfXtalFrequency + xtal_comp_value;

  *s_RfRangeExtender = (RangeExtType)tmpBuffer[5];
}

uint32_t S2LPGetFrequencyBand(uint8_t s_RfModuleBand)
{
  uint32_t frequency = 0;
  const uint32_t band_frequencies[] = {
    169000000,
    315000000,
    433000000,
    868000000,
    915000000,
    450000000
  };

  if (s_RfModuleBand < (sizeof(band_frequencies)/sizeof(uint32_t))) {
    frequency = band_frequencies[s_RfModuleBand];
  }

  return frequency;
}
