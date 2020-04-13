/* Speeduino FlashAsEEPROM Library v.1.0.0
 * Copyright (C) 2020 by Tjeerd Hoogendijk
 * Created by Tjeerd Hoogendijk - 11/4/2020
 *
 * This file is part of the Speeduino project. This library is for using the Internal 
 * flash of the STM32F407 as configuration storage. In its current form it enables reading
 * and writing individual bytes as if it where an AVR EEPROM. It uses some 
 * wear leveling (16x). When the begin() fuction is called for the first time 
 * it will "format" the Last sector of the internal flash. 
 * !!!!THIS DISTROYS ANY EXISTING DATA ON THE LAST SECTOR FLASH!!!
 *  
 * 131072 bytes used of the internal flash resulting in xxxx bytes of usable EEPROM
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License v3.0
 * along with the Speeduino FlashAsEEPROM Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#ifndef FLASH_AS_EEPROM_H
#define FLASH_AS_EEPROM_H

#if defined(USE_STM32FLASH_EEPROM)
#define FLASH_SIZE_USED                   131072 //must be a multiple of sectorsize //(131072/1024)*63 = 8064 bytes of EEPROM
#define FLASH_SECTOR_BASE_ADDRESS         0x08060000 //This is sector 7 and last sector of the STM32F407VE6 internal flash this one is 128kbyte 
#define EMULATION_SECTOR_SIZE             1024 //Size for each sector 
#define EMULATION_WEAR_FACTOR             14 //14 bytes for storage, 2 bytes=16 bits to store wich of the 14 bytes holds the valid data 
#define EMULATION_MAGIC_NUMBER_SIZE       16 //magic number size in bytes. Magic number is stored at the start of each Sector
#define ADRESS_TRANSLATION_SIZE           2  //number of bytes to translate EEPROM adres to valid data adress in flash. must be >=EMULATION_WEAR_FACTOR/8    
#define EMULATION_BYTES_PER_SECTOR        (EMULATION_SECTOR_SIZE - EMULATION_MAGIC_NUMBER_SIZE)/(ADRESS_TRANSLATION_SIZE + EMULATION_WEAR_FACTOR) 
#define MAGICNUMBER1                      0xC0 //magic number to identify eeprom emulation
#define MAGICNUMBER2                      0xFF //magic number to identify eeprom emulation
#define MAGICNUMBER3                      0xEE //magic number to identify eeprom emulation
#define BITS_PER_BYTE                     8 

#define EEPROM_LENGTH  (FLASH_SIZE_USED/EMULATION_SECTOR_SIZE)*EMULATION_BYTES_PER_SECTOR

#include "Arduino.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32_eeprom.h"
#include "stm32f4xx_hal_flash.h"

class STM32FlashAsEEPROM {

  public:

    STM32FlashAsEEPROM(void);

      /**
     * Format flash storage for EEPROM emulation 
     */
    int8_t formatFlashForUse();

    /**
     * Read an eeprom cell
     * @param index
     * @return value
     */
    uint8_t read(uint16_t);

    /**
     * Write value to an eeprom cell
     * @param index
     * @param value
     */
    int8_t write(uint16_t, uint8_t);

    /**
     * Update a eeprom cell
     * @param index
     * @param value
     */
    int8_t update(uint16_t, uint8_t);

    /**
     * get EEPROM length
     */
    uint16_t length() { return EEPROM_LENGTH; }

  private:
    void init();
    int8_t flash_read_byte( uint8_t *data, uint16_t bytes, uint16_t offset);
    uint8_t   writeMagicNumbers(uint16_t sectorNumber);
    
    bool _initialized;
    uint8_t   Buffer[ADRESS_TRANSLATION_SIZE + EMULATION_WEAR_FACTOR];
    uint8_t   Magicbuf[4];
    uint16_t  SectorNumber;
    uint8_t   CountOnes;
    uint16_t  FlashAddress;

    // uint16_t  count(uint8_t *buf[]);
};

extern STM32FlashAsEEPROM EEPROM;

#endif
#endif


