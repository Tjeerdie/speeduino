/* Speeduino STM32FlashAsEEPROM Library v.1.0.0
 * Copyright (C) 2020 by Tjeerd Hoogendijk
 * Created by Tjeerd Hoogendijk - 11/4/2020
 *
 * This file is part of the Speeduino project. This library is for using the Internal 
 * flash of the STM32F407 as configuration storage. In its current form it enables reading
 * and writing individual bytes as if it where an AVR EEPROM. It uses some 
 * wear leveling (14x). When the begin() fuction is called for the first time 
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

#if defined(USE_STM32FLASH_EEPROM)
#include "src/FlashAsEEPROM/STM32FlashAsEEPROM.h"
//#include "globals.h"     

STM32FlashAsEEPROM::STM32FlashAsEEPROM()
{
    Magicbuf[0] = MAGICNUMBER1;
    Magicbuf[1] = MAGICNUMBER2;
    Magicbuf[2] = MAGICNUMBER3;
    Magicbuf[3] = 0x00;
}
void STM32FlashAsEEPROM::init()
{
    uint8_t formatted = 1; //assume formated for use untill proven otherwise

    //check for magic numbers
    uint8_t buf[EMULATION_MAGIC_NUMBER_SIZE] = {0};
    for(uint16_t i=0; i<FLASH_SIZE_USED/EMULATION_SECTOR_SIZE; i++ ){
        flash_read_byte(*buf, 15, EMULATION_SECTOR_SIZE*i);
    //   Serial.println(EMULATION_SECTOR_SIZE*i, DEC);
    //   for (uint16_t i = 0; i < len(buf); i++)
    //   {
    //     Serial.print(buf[i], HEX);
    //   }
        if((buf[0] != MAGICNUMBER1) | (buf[1] != MAGICNUMBER2) | (buf[2] != MAGICNUMBER3)){
        //if one of the SECTORS has no magic numbers it is not formatted
        formatted = 0;
        }
    }
    //If not formatted format flash. This takes 10 seconds or more!
    if(!formatted){
        formatFlashForUse();
    }

}

int8_t STM32FlashAsEEPROM::write(uint16_t addressEEPROM, uint8_t Value)
{ 
    // uint8_t index;
    // uint8_t values[100];
    // for (index = 0; index < count; ++index) {
    //     if (IS_FLASH_ADDRESS(address+index)) {
    //         /* flash write */
    //         FLASH_Unlock();
    //         if (FLASH_ProgramByte(address+index, values[index]) != FLASH_COMPLETE) {
    //               return FLASH_ERROR;
    //         }
    //         FLASH_Lock();
    //     } else {
    //         /* ram write */
    //         ((uint8_t*)address)[index] = values[index];
    //     }
    // }
    return 0;
}

int8_t STM32FlashAsEEPROM::update(uint16_t address, uint8_t val){
    //a write function call is already an update. 
    write(address, val);
    return 0;
}

int8_t STM32FlashAsEEPROM::formatFlashForUse()
{

  //Format the flash for use by erasing and 
  //write the magic number at the start of each 
  //sector

  FLASH_EraseInitTypeDef EraseInitStruct;
  // uint32_t offset = 0;
  // uint32_t address = FLASH_BASE_ADDRESS;
  // uint32_t address_end = FLASH_BASE_ADDRESS + E2END;
  uint32_t SectorError = 0;
  // uint32_t data = 0;
  int8_t status=-1;
  uint32_t count=0;
  /* ERASING page */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FLASH_SECTOR_7;
  EraseInitStruct.NbSectors = 1;

  HAL_FLASH_Unlock();

  while(status!=HAL_OK | count>10)
  {
      status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
      count++;
  }

//   winbondSPIFlash.writePage(sectorNumber*SECTORSIZE, magicbuf, MAGICNUMBER_OFFSET)
//   for(uint16_t i = 0; i<FLASH_SIZE_USED/EMULATION_SECTOR_SIZE; i++){
//     writeMagicNumber(i);
//   } 
    uint32_t address=0;
    for(uint16_t i = 0; i < FLASH_SIZE_USED / EMULATION_SECTOR_SIZE; i++){
        address = FLASH_SECTOR_BASE_ADDRESS + i*EMULATION_SECTOR_SIZE;
        for (uint16_t j = 0; j < 4; ++j) {
             if (IS_FLASH_ADDRESS(address+j)) {
                /* flash write */
                if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address+j, Magicbuf[j]) == HAL_OK) {}
            } else {
                status = -1;
            }
        }
    }


  HAL_FLASH_Lock();
 
  if (status!=HAL_OK | count>10){
      return -1;
  }else{return 0;}


  return status;
}

// int8_t STM32FlashAsEEPROM::clear(){
//   return formatFlashForUse();
// }

uint8_t STM32FlashAsEEPROM::read(uint16_t addressEEPROM){
    //The infoblock is at the start of each sector 
    //The first two pages will be used for the infoblock
    //The first 4 bytes of each page must have the magic number 
    //version 0.1 does not check magic number
    init();
    // uint8_t buf[INFOBYTES_PER_BYTE];
    //Check if address is outside of the maximum. limit to get inside maximum and continue as normal.
    //Should be changed so that it returns -1 (error)
    // if (addressEEPROM >= FLASHSIZEUSED/SECTORSIZE * (SECTORSIZE/FLASH_PAGESIZE - 2)){addressEEPROM = FLASHSIZEUSED/SECTORSIZE * (SECTORSIZE/FLASH_PAGESIZE - 2);}  
    
    //Check at what sector number the adress resides. 14 bytes per sector
    // sectorNumber = addressEEPROM/(SECTORSIZE/FLASH_PAGESIZE - 2);

    //Check at what page number in the sector the adress can be found (16 pages per sector, 14 used)
    // pageNumber = addressEEPROM - (sectorNumber * ((SECTORSIZE/FLASH_PAGESIZE) - 2));

    //The absulute adress of the infoblock of the byte in flash adress
    // infoFlashAddress = sectorNumber*SECTORSIZE + pageNumber * INFOBYTES_PER_BYTE + MAGICNUMBER_OFFSET;

    // return ByteBuf[0];
}

// uint16_t STM32FlashAsEEPROM::count(uint8_t *buf[]){
//     uint16_t count=0;
//     for(uint8_t j=0; j < 32; j++)
//       for(uint8_t i=0; i<8; i++){
//         if((buf[j] & 1) == 1){ //if current bit 1
//           count++;//increase count
//         }
//         buf[j]=buf[j]>>1;//right shift
//       }
//     return count;
// }

int8_t STM32FlashAsEEPROM::flash_read_byte( uint8_t *data, uint16_t bytes, uint16_t offset ) {
      uint8_t* base_addr = (uint8_t *) FLASH_SECTOR_BASE_ADDRESS;
      uint16_t i;
      if( bytes + offset >= FLASH_SIZE_USED ) {
        /* ERROR : the last byte is outside the backup SRAM region */
        return -1;
      }
      
      for( i = 0; i < bytes; i++ ) { 
        *(data + i) = *(base_addr + offset + i);
      }
      return 0;
    }



STM32FlashAsEEPROM EEPROM;

#endif