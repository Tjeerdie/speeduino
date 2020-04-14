/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

#include "globals.h"
#include "table.h"
#include "comms.h"
#include BOARD_H 
#include "storage.h"

void writeAllConfig()
{
  writeConfig(veSetPage);
  if (eepromWritesPending == false) { writeConfig(veMapPage, !FLUSHBUFFER); }
  if (eepromWritesPending == false) { writeConfig(ignMapPage, !FLUSHBUFFER); }
  if (eepromWritesPending == false) { writeConfig(ignSetPage, !FLUSHBUFFER); }
  if (eepromWritesPending == false) { writeConfig(afrMapPage, !FLUSHBUFFER); }
  if (eepromWritesPending == false) { writeConfig(afrSetPage, !FLUSHBUFFER); }
  if (eepromWritesPending == false) { writeConfig(boostvvtPage, !FLUSHBUFFER); }
  if (eepromWritesPending == false) { writeConfig(seqFuelPage, !FLUSHBUFFER); }
  if (eepromWritesPending == false) { writeConfig(canbusPage, !FLUSHBUFFER); }
  if (eepromWritesPending == false) { writeConfig(warmupPage, !FLUSHBUFFER); }
  if (eepromWritesPending == false) { writeConfig(fuelMap2Page, !FLUSHBUFFER);}
  if (eepromWritesPending == false) { flushConfigBuffer();}
  
}


/*
Takes the current configuration (config pages and maps)
and writes them to EEPROM as per the layout defined in storage.h
*/
void writeConfig(byte tableNum, byte BufferFlush = FLUSHBUFFER)
{
  /*
  We only ever write to the EEPROM where the new value is different from the currently stored byte
  This is due to the limited write life of the EEPROM (Approximately 100,000 writes)
  */

  int offset;
  int i, z, y;
  int writeCounter = 0;
  byte newVal; //Used for tempoerarily storing the new intended value
  //Create a pointer to the config page
  byte* pnt_configPage;

  switch(tableNum)
  {
    case veMapPage:
      /*---------------------------------------------------
      | Fuel table (See storage.h for data layout) - Page 1
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      if(readConfigByte(EEPROM_CONFIG1_XSIZE) != fuelTable.xSize) { writeConfigByte(EEPROM_CONFIG1_XSIZE, fuelTable.xSize); writeCounter++; } //Write the VE Tables RPM dimension size
      if(readConfigByte(EEPROM_CONFIG1_YSIZE) != fuelTable.ySize) { writeConfigByte(EEPROM_CONFIG1_YSIZE, fuelTable.ySize); writeCounter++; } //Write the VE Tables MAP/TPS dimension size
      for(int x=EEPROM_CONFIG1_MAP; x<EEPROM_CONFIG1_XBINS; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG1_MAP;
        if( readConfigByte(x) != (fuelTable.values[15-(offset/16)][offset%16]) ) { writeConfigByte(x, fuelTable.values[15-(offset/16)][offset%16]); writeCounter++; }  //Write the 16x16 map
      }

      //RPM bins
      for(int x=EEPROM_CONFIG1_XBINS; x<EEPROM_CONFIG1_YBINS; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG1_XBINS;
        if( readConfigByte(x) != (byte(fuelTable.axisX[offset]/TABLE_RPM_MULTIPLIER)) ) { writeConfigByte(x, byte(fuelTable.axisX[offset]/TABLE_RPM_MULTIPLIER)); writeCounter++; } //RPM bins are divided by 100 and converted to a byte
      }
      //TPS/MAP bins
      for(int x=EEPROM_CONFIG1_YBINS; x<EEPROM_CONFIG2_START; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG1_YBINS;
        updateConfigByte(x, fuelTable.axisY[offset] / TABLE_LOAD_MULTIPLIER); //Table load is divided by 2 (Allows for MAP up to 511)
      }
      if(writeCounter > EEPROM_MAX_WRITE_BLOCK) { eepromWritesPending = true; }
      else { eepromWritesPending = false; }
      break;
      //That concludes the writing of the VE table

    case veSetPage:
      /*---------------------------------------------------
      | Config page 2 (See storage.h for data layout)
      | 64 byte long config table
      -----------------------------------------------------*/
      pnt_configPage = (byte *)&configPage2; //Create a pointer to Page 2 in memory
      for(int x=EEPROM_CONFIG2_START; x<EEPROM_CONFIG2_END; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        if(readConfigByte(x) != *(pnt_configPage + byte(x - EEPROM_CONFIG2_START))) { writeConfigByte(x, *(pnt_configPage + byte(x - EEPROM_CONFIG2_START))); writeCounter++; }
      }

      if(writeCounter > EEPROM_MAX_WRITE_BLOCK) { eepromWritesPending = true; }
      else { eepromWritesPending = false; }

      break;

    case ignMapPage:
      /*---------------------------------------------------
      | Ignition table (See storage.h for data layout) - Page 1
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      //Begin writing the Ignition table, basically the same thing as above
      if(readConfigByte(EEPROM_CONFIG3_XSIZE) != ignitionTable.xSize) { writeConfigByte(EEPROM_CONFIG3_XSIZE,ignitionTable.xSize); writeCounter++; } //Write the ignition Table RPM dimension size
      if(readConfigByte(EEPROM_CONFIG3_YSIZE) != ignitionTable.ySize) { writeConfigByte(EEPROM_CONFIG3_YSIZE,ignitionTable.ySize); writeCounter++; } //Write the ignition Table MAP/TPS dimension size

      for(int x=EEPROM_CONFIG3_MAP; x<EEPROM_CONFIG3_XBINS; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG3_MAP;
        newVal = ignitionTable.values[15-(offset/16)][offset%16];
        if(readConfigByte(x) != newVal) { writeConfigByte(x, newVal); writeCounter++; }  //Write the 16x16 map with translation
      }
      //RPM bins
      for(int x=EEPROM_CONFIG3_XBINS; x<EEPROM_CONFIG3_YBINS; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG3_XBINS;
        newVal = ignitionTable.axisX[offset]/TABLE_RPM_MULTIPLIER;
        if(readConfigByte(x) != newVal) { writeConfigByte(x, newVal); writeCounter++; } //RPM bins are divided by 100 and converted to a byte
      }
      //TPS/MAP bins
      for(int x=EEPROM_CONFIG3_YBINS; x<EEPROM_CONFIG4_START; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG3_YBINS;
        newVal = ignitionTable.axisY[offset]/TABLE_LOAD_MULTIPLIER;
        if(readConfigByte(x) != newVal) { writeConfigByte(x, newVal); writeCounter++; } //Table load is divided by 2 (Allows for MAP up to 511)
      }

      if(writeCounter > EEPROM_MAX_WRITE_BLOCK) { eepromWritesPending = true; }
      else { eepromWritesPending = false; }

      break;

    case ignSetPage:
      /*---------------------------------------------------
      | Config page 2 (See storage.h for data layout)
      | 64 byte long config table
      -----------------------------------------------------*/
      pnt_configPage = (byte *)&configPage4; //Create a pointer to Page 2 in memory
      for(int x=EEPROM_CONFIG4_START; x<EEPROM_CONFIG4_END; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        if(readConfigByte(x) != *(pnt_configPage + byte(x - EEPROM_CONFIG4_START))) { writeConfigByte(x, *(pnt_configPage + byte(x - EEPROM_CONFIG4_START))); writeCounter++; }
      }

      if(writeCounter > EEPROM_MAX_WRITE_BLOCK) { eepromWritesPending = true; }
      else { eepromWritesPending = false; }

      break;

    case afrMapPage:
      /*---------------------------------------------------
      | AFR table (See storage.h for data layout) - Page 5
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      //Begin writing the Ignition table, basically the same thing as above
      if(readConfigByte(EEPROM_CONFIG5_XSIZE) != afrTable.xSize) { writeConfigByte(EEPROM_CONFIG5_XSIZE,afrTable.xSize); writeCounter++; } //Write the ignition Table RPM dimension size
      if(readConfigByte(EEPROM_CONFIG5_YSIZE) != afrTable.ySize) { writeConfigByte(EEPROM_CONFIG5_YSIZE,afrTable.ySize); writeCounter++; } //Write the ignition Table MAP/TPS dimension size

      for(int x=EEPROM_CONFIG5_MAP; x<EEPROM_CONFIG5_XBINS; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG5_MAP;
        if(readConfigByte(x) != (afrTable.values[15-(offset/16)][offset%16]) ) { writeConfigByte(x, afrTable.values[15-(offset/16)][offset%16]); writeCounter++; }  //Write the 16x16 map
      }
      //RPM bins
      for(int x=EEPROM_CONFIG5_XBINS; x<EEPROM_CONFIG5_YBINS; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG5_XBINS;
        if(readConfigByte(x) != byte(afrTable.axisX[offset]/TABLE_RPM_MULTIPLIER)) { writeConfigByte(x, byte(afrTable.axisX[offset]/TABLE_RPM_MULTIPLIER)); writeCounter++; } //RPM bins are divided by 100 and converted to a byte
      }
      //TPS/MAP bins
      for(int x=EEPROM_CONFIG5_YBINS; x<EEPROM_CONFIG6_START; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG5_YBINS;
        updateConfigByte(x, afrTable.axisY[offset]/TABLE_LOAD_MULTIPLIER); //Table load is divided by 2 (Allows for MAP up to 511)
      }

      if(writeCounter > EEPROM_MAX_WRITE_BLOCK) { eepromWritesPending = true; }
      else { eepromWritesPending = false; }

      break;

    case afrSetPage:
      /*---------------------------------------------------
      | Config page 3 (See storage.h for data layout)
      | 64 byte long config table
      -----------------------------------------------------*/
      pnt_configPage = (byte *)&configPage6; //Create a pointer to Page 3 in memory
      for(int x=EEPROM_CONFIG6_START; x<EEPROM_CONFIG6_END; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        if(readConfigByte(x) != *(pnt_configPage + byte(x - EEPROM_CONFIG6_START))) { writeConfigByte(x, *(pnt_configPage + byte(x - EEPROM_CONFIG6_START))); writeCounter++; }
      }

      if(writeCounter > EEPROM_MAX_WRITE_BLOCK) { eepromWritesPending = true; }
      else { eepromWritesPending = false; }

      break;

    case boostvvtPage:
      /*---------------------------------------------------
      | Boost and vvt tables (See storage.h for data layout) - Page 8
      | 8x8 table itself + the 8 values along each of the axis
      -----------------------------------------------------*/
      //Begin writing the 2 tables, basically the same thing as above but we're doing these 2 together (2 tables per page instead of 1)
      if(readConfigByte(EEPROM_CONFIG7_XSIZE1) != boostTable.xSize) { writeConfigByte(EEPROM_CONFIG7_XSIZE1,boostTable.xSize); writeCounter++; } //Write the boost Table RPM dimension size
      if(readConfigByte(EEPROM_CONFIG7_YSIZE1) != boostTable.ySize) { writeConfigByte(EEPROM_CONFIG7_YSIZE1,boostTable.ySize); writeCounter++; } //Write the boost Table MAP/TPS dimension size
      if(readConfigByte(EEPROM_CONFIG7_XSIZE2) != vvtTable.xSize) { writeConfigByte(EEPROM_CONFIG7_XSIZE2,vvtTable.xSize); writeCounter++; } //Write the vvt Table RPM dimension size
      if(readConfigByte(EEPROM_CONFIG7_YSIZE2) != vvtTable.ySize) { writeConfigByte(EEPROM_CONFIG7_YSIZE2,vvtTable.ySize); writeCounter++; } //Write the vvt Table MAP/TPS dimension size
      if(readConfigByte(EEPROM_CONFIG7_XSIZE3) != stagingTable.xSize) { writeConfigByte(EEPROM_CONFIG7_XSIZE3,stagingTable.xSize); writeCounter++; } //Write the staging Table RPM dimension size
      if(readConfigByte(EEPROM_CONFIG7_YSIZE3) != stagingTable.ySize) { writeConfigByte(EEPROM_CONFIG7_YSIZE3,stagingTable.ySize); writeCounter++; } //Write the staging Table MAP/TPS dimension size

      y = EEPROM_CONFIG7_MAP2; //We do the 3 maps together in the same loop
      z = EEPROM_CONFIG7_MAP3;
      for(int x=EEPROM_CONFIG7_MAP1; x<EEPROM_CONFIG7_XBINS1; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG7_MAP1;
        if(readConfigByte(x) != (boostTable.values[7-(offset/8)][offset%8]) ) { writeConfigByte(x, boostTable.values[7-(offset/8)][offset%8]); writeCounter++; }  //Write the 8x8 map
        offset = y - EEPROM_CONFIG7_MAP2;
        if(readConfigByte(y) != (vvtTable.values[7-(offset/8)][offset%8]) ) { writeConfigByte(y, vvtTable.values[7-(offset/8)][offset%8]); writeCounter++; }  //Write the 8x8 map
        offset = z - EEPROM_CONFIG7_MAP3;
        if(readConfigByte(z) != (stagingTable.values[7-(offset/8)][offset%8]) ) { writeConfigByte(z, stagingTable.values[7-(offset/8)][offset%8]); writeCounter++; }  //Write the 8x8 map
        y++;
        z++;
      }
      //RPM bins
      y = EEPROM_CONFIG7_XBINS2;
      z = EEPROM_CONFIG7_XBINS3;
      for(int x=EEPROM_CONFIG7_XBINS1; x<EEPROM_CONFIG7_YBINS1; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG7_XBINS1;
        if(readConfigByte(x) != byte(boostTable.axisX[offset]/TABLE_RPM_MULTIPLIER)) { writeConfigByte(x, byte(boostTable.axisX[offset]/TABLE_RPM_MULTIPLIER)); writeCounter++; } //RPM bins are divided by 100 and converted to a byte
        offset = y - EEPROM_CONFIG7_XBINS2;
        if(readConfigByte(y) != byte(vvtTable.axisX[offset]/TABLE_RPM_MULTIPLIER)) { writeConfigByte(y, byte(vvtTable.axisX[offset]/TABLE_RPM_MULTIPLIER)); writeCounter++; } //RPM bins are divided by 100 and converted to a byte
        offset = z - EEPROM_CONFIG7_XBINS3;
        if(readConfigByte(z) != byte(stagingTable.axisX[offset]/TABLE_RPM_MULTIPLIER)) { writeConfigByte(z, byte(stagingTable.axisX[offset]/TABLE_RPM_MULTIPLIER)); writeCounter++; } //RPM bins are divided by 100 and converted to a byte
        y++;
        z++;
      }
      //TPS/MAP bins
      y=EEPROM_CONFIG7_YBINS2;
      z=EEPROM_CONFIG7_YBINS3;
      for(int x=EEPROM_CONFIG7_YBINS1; x<EEPROM_CONFIG7_XSIZE2; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG7_YBINS1;
        if(readConfigByte(x) != boostTable.axisY[offset]) { writeConfigByte(x, boostTable.axisY[offset]); writeCounter++; } //TABLE_LOAD_MULTIPLIER is NOT used for boost as it is TPS based (0-100)
        offset = y - EEPROM_CONFIG7_YBINS2;
        if(readConfigByte(y) != vvtTable.axisY[offset]) { writeConfigByte(y, vvtTable.axisY[offset]); writeCounter++; } //TABLE_LOAD_MULTIPLIER is NOT used for VVT as it is TPS based (0-100)
        offset = z - EEPROM_CONFIG7_YBINS3;
        if(readConfigByte(z) != byte(stagingTable.axisY[offset]/TABLE_LOAD_MULTIPLIER)) { writeConfigByte(z, byte(stagingTable.axisY[offset]/TABLE_LOAD_MULTIPLIER)); writeCounter++; } //RPM bins are divided by 100 and converted to a byte
        y++;
        z++;
      }

      if(writeCounter > EEPROM_MAX_WRITE_BLOCK) { eepromWritesPending = true; }
      else { eepromWritesPending = false; }

      break;

    case seqFuelPage:
      /*---------------------------------------------------
      | Fuel trim tables (See storage.h for data layout) - Page 9
      | 6x6 tables itself + the 6 values along each of the axis
      -----------------------------------------------------*/
      //Begin writing the 2 tables, basically the same thing as above but we're doing these 2 together (2 tables per page instead of 1)
      if(readConfigByte(EEPROM_CONFIG8_XSIZE1) != trim1Table.xSize) { writeConfigByte(EEPROM_CONFIG8_XSIZE1,trim1Table.xSize); writeCounter++; } //Write the boost Table RPM dimension size
      if(readConfigByte(EEPROM_CONFIG8_YSIZE1) != trim1Table.ySize) { writeConfigByte(EEPROM_CONFIG8_YSIZE1,trim1Table.ySize); writeCounter++; } //Write the boost Table MAP/TPS dimension size
      if(readConfigByte(EEPROM_CONFIG8_XSIZE2) != trim2Table.xSize) { writeConfigByte(EEPROM_CONFIG8_XSIZE2,trim2Table.xSize); writeCounter++; } //Write the boost Table RPM dimension size
      if(readConfigByte(EEPROM_CONFIG8_YSIZE2) != trim2Table.ySize) { writeConfigByte(EEPROM_CONFIG8_YSIZE2,trim2Table.ySize); writeCounter++; } //Write the boost Table MAP/TPS dimension size
      if(readConfigByte(EEPROM_CONFIG8_XSIZE3) != trim3Table.xSize) { writeConfigByte(EEPROM_CONFIG8_XSIZE3,trim3Table.xSize); writeCounter++; } //Write the boost Table RPM dimension size
      if(readConfigByte(EEPROM_CONFIG8_YSIZE3) != trim3Table.ySize) { writeConfigByte(EEPROM_CONFIG8_YSIZE3,trim3Table.ySize); writeCounter++; } //Write the boost Table MAP/TPS dimension size
      if(readConfigByte(EEPROM_CONFIG8_XSIZE4) != trim4Table.xSize) { writeConfigByte(EEPROM_CONFIG8_XSIZE4,trim4Table.xSize); writeCounter++; } //Write the boost Table RPM dimension size
      if(readConfigByte(EEPROM_CONFIG8_YSIZE4) != trim4Table.ySize) { writeConfigByte(EEPROM_CONFIG8_YSIZE4,trim4Table.ySize); writeCounter++; } //Write the boost Table MAP/TPS dimension size

      y = EEPROM_CONFIG8_MAP2; //We do the 4 maps together in the same loop
      z = EEPROM_CONFIG8_MAP3; //We do the 4 maps together in the same loop
      i = EEPROM_CONFIG8_MAP4; //We do the 4 maps together in the same loop

      for(int x=EEPROM_CONFIG8_MAP1; x<EEPROM_CONFIG8_XBINS1; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG8_MAP1;
        newVal = trim1Table.values[5-(offset/6)][offset%6];
        if (readConfigByte(x) != newVal ) { updateConfigByte(x, newVal ); writeCounter++; } //Write the 6x6 map

        offset = y - EEPROM_CONFIG8_MAP2;
        newVal = trim2Table.values[5-(offset/6)][offset%6];
        if (readConfigByte(y) != newVal ) { updateConfigByte(y, newVal); writeCounter++; } //Write the 6x6 map

        offset = z - EEPROM_CONFIG8_MAP3;
        newVal = trim3Table.values[5-(offset/6)][offset%6];
        if (readConfigByte(z) != newVal ) { updateConfigByte(z, newVal); writeCounter++; } //Write the 6x6 map

        offset = i - EEPROM_CONFIG8_MAP4;
        newVal = trim4Table.values[5-(offset/6)][offset%6];
        if (readConfigByte(i) != newVal ) { updateConfigByte(i, newVal); writeCounter++; } //Write the 6x6 map

        y++;
        z++;
        i++;
      }
      //RPM bins
      y = EEPROM_CONFIG8_XBINS2;
      z = EEPROM_CONFIG8_XBINS3;
      i = EEPROM_CONFIG8_XBINS4;
      for(int x=EEPROM_CONFIG8_XBINS1; x<EEPROM_CONFIG8_YBINS1; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { eepromWritesPending = true; break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG8_XBINS1;
        updateConfigByte(x, byte(trim1Table.axisX[offset]/TABLE_RPM_MULTIPLIER)); //RPM bins are divided by 100 and converted to a byte
        offset = y - EEPROM_CONFIG8_XBINS2;
        updateConfigByte(y, byte(trim2Table.axisX[offset]/TABLE_RPM_MULTIPLIER)); //RPM bins are divided by 100 and converted to a byte
        offset = z - EEPROM_CONFIG8_XBINS3;
        updateConfigByte(z, byte(trim3Table.axisX[offset]/TABLE_RPM_MULTIPLIER)); //RPM bins are divided by 100 and converted to a byte
        offset = i - EEPROM_CONFIG8_XBINS4;
        updateConfigByte(i, byte(trim4Table.axisX[offset]/TABLE_RPM_MULTIPLIER)); //RPM bins are divided by 100 and converted to a byte
        y++;
        z++;
        i++;
      }
      //TPS/MAP bins
      y=EEPROM_CONFIG8_YBINS2;
      z=EEPROM_CONFIG8_YBINS3;
      i=EEPROM_CONFIG8_YBINS4;
      for(int x=EEPROM_CONFIG8_YBINS1; x<EEPROM_CONFIG8_XSIZE2; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { eepromWritesPending = true; break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG8_YBINS1;
        updateConfigByte(x, trim1Table.axisY[offset]/TABLE_LOAD_MULTIPLIER); //Table load is divided by 2 (Allows for MAP up to 511)
        offset = y - EEPROM_CONFIG8_YBINS2;
        updateConfigByte(y, trim2Table.axisY[offset]/TABLE_LOAD_MULTIPLIER); //Table load is divided by 2 (Allows for MAP up to 511)
        offset = z - EEPROM_CONFIG8_YBINS3;
        updateConfigByte(z, trim3Table.axisY[offset]/TABLE_LOAD_MULTIPLIER); //Table load is divided by 2 (Allows for MAP up to 511)
        offset = i - EEPROM_CONFIG8_YBINS4;
        updateConfigByte(i, trim4Table.axisY[offset]/TABLE_LOAD_MULTIPLIER); //Table load is divided by 2 (Allows for MAP up to 511)
        y++;
        z++;
        i++;
      }
      if(writeCounter > EEPROM_MAX_WRITE_BLOCK) { eepromWritesPending = true; }
      else { eepromWritesPending = false; }

      break;

    case canbusPage:
      /*---------------------------------------------------
      | Config page 10 (See storage.h for data layout)
      | 192 byte long config table
      -----------------------------------------------------*/
      pnt_configPage = (byte *)&configPage9; //Create a pointer to Page 10 in memory
      for(int x=EEPROM_CONFIG9_START; x<EEPROM_CONFIG9_END; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        if(readConfigByte(x) != *(pnt_configPage + byte(x - EEPROM_CONFIG9_START))) { writeConfigByte(x, *(pnt_configPage + byte(x - EEPROM_CONFIG9_START))); writeCounter++; }
      }

      if(writeCounter > EEPROM_MAX_WRITE_BLOCK) { eepromWritesPending = true; }
      else { eepromWritesPending = false; }

      break;

    case warmupPage:
      /*---------------------------------------------------
      | Config page 11 (See storage.h for data layout)
      | 192 byte long config table
      -----------------------------------------------------*/
      pnt_configPage = (byte *)&configPage10; //Create a pointer to Page 11 in memory
      //As there are no 3d tables in this page, all 192 bytes can simply be read in
      for(int x=EEPROM_CONFIG10_START; x<EEPROM_CONFIG10_END; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        if(readConfigByte(x) != *(pnt_configPage + byte(x - EEPROM_CONFIG10_START))) { writeConfigByte(x, *(pnt_configPage + byte(x - EEPROM_CONFIG10_START))); writeCounter++; }
      }

      if(writeCounter > EEPROM_MAX_WRITE_BLOCK) { eepromWritesPending = true; }
      else { eepromWritesPending = false; }

      break;

    case fuelMap2Page:
      /*---------------------------------------------------
      | Fuel table (See storage.h for data layout) - Page 1
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      updateConfigByte(EEPROM_CONFIG11_XSIZE, fuelTable2.xSize); writeCounter++; //Write the VE Tables RPM dimension size
      updateConfigByte(EEPROM_CONFIG11_YSIZE, fuelTable2.ySize); writeCounter++; //Write the VE Tables MAP/TPS dimension size
      for(int x=EEPROM_CONFIG11_MAP; x<EEPROM_CONFIG11_XBINS; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG11_MAP;
        if( readConfigByte(x) != (fuelTable2.values[15-(offset/16)][offset%16]) ) { writeConfigByte(x, fuelTable2.values[15-(offset/16)][offset%16]); writeCounter++; }  //Write the 16x16 map
      }

      //RPM bins
      for(int x=EEPROM_CONFIG11_XBINS; x<EEPROM_CONFIG11_YBINS; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG11_XBINS;
        if( readConfigByte(x) != (byte(fuelTable2.axisX[offset]/TABLE_RPM_MULTIPLIER)) ) { writeConfigByte(x, byte(fuelTable2.axisX[offset]/TABLE_RPM_MULTIPLIER)); writeCounter++; } //RPM bins are divided by 100 and converted to a byte
      }
      //TPS/MAP bins
      for(int x=EEPROM_CONFIG11_YBINS; x<EEPROM_CONFIG11_END; x++)
      {
        if( (writeCounter > EEPROM_MAX_WRITE_BLOCK) ) { break; } //This is a safety check to make sure we don't attempt to write too much to the EEPROM at a time.
        offset = x - EEPROM_CONFIG11_YBINS;
        updateConfigByte(x, fuelTable2.axisY[offset] / TABLE_LOAD_MULTIPLIER); //Table load is divided by 2 (Allows for MAP up to 511)
      }
      if(writeCounter > EEPROM_MAX_WRITE_BLOCK) { eepromWritesPending = true; }
      else { eepromWritesPending = false; }
      break;
      //That concludes the writing of the 2nd fuel table

    default:
      break;
  }
  if((eepromWritesPending == false) && (writeCounter >= 1) && (BufferFlush)) {
    flushConfigBuffer();
    }
}

void loadConfig()
{
  
  int offset;
  //Create a pointer to the config page
  byte* pnt_configPage;
  fillConfigBuffer();

  //Fuel table (See storage.h for data layout)
  for(uint16_t x=EEPROM_CONFIG1_MAP; x<EEPROM_CONFIG1_XBINS; x++)
  {
    offset = x - EEPROM_CONFIG1_MAP;
    fuelTable.values[15-(offset/16)][offset%16] = readConfigByte(x); //Read the 8x8 map
  }
  //RPM bins
  for(int x=EEPROM_CONFIG1_XBINS; x<EEPROM_CONFIG1_YBINS; x++)
  {
    offset = x - EEPROM_CONFIG1_XBINS;
    fuelTable.axisX[offset] = (readConfigByte(x) * TABLE_RPM_MULTIPLIER); //RPM bins are divided by 100 when stored. Multiply them back now
  }
  //TPS/MAP bins
  for(int x=EEPROM_CONFIG1_YBINS; x<EEPROM_CONFIG2_START; x++)
  {
    offset = x - EEPROM_CONFIG1_YBINS;
    fuelTable.axisY[offset] = readConfigByte(x) * TABLE_LOAD_MULTIPLIER;
  }
  pnt_configPage = (byte *)&configPage2; //Create a pointer to Page 1 in memory
  for(int x=EEPROM_CONFIG2_START; x<EEPROM_CONFIG2_END; x++)
  {
    *(pnt_configPage + byte(x - EEPROM_CONFIG2_START)) = readConfigByte(x);
  }
  //That concludes the reading of the VE table
  
  //*********************************************************************************************************************************************************************************
  //IGNITION CONFIG PAGE (2)

  //Begin writing the Ignition table, basically the same thing as above
  for(int x=EEPROM_CONFIG3_MAP; x<EEPROM_CONFIG3_XBINS; x++)
  {
    offset = x - EEPROM_CONFIG3_MAP;
    ignitionTable.values[15-(offset/16)][offset%16] = readConfigByte(x); //Read the 8x8 map
  }
  //RPM bins
  for(int x=EEPROM_CONFIG3_XBINS; x<EEPROM_CONFIG3_YBINS; x++)
  {
    offset = x - EEPROM_CONFIG3_XBINS;
    ignitionTable.axisX[offset] = (readConfigByte(x) * TABLE_RPM_MULTIPLIER); //RPM bins are divided by 100 when stored. Multiply them back now
  }
  //TPS/MAP bins
  for(int x=EEPROM_CONFIG3_YBINS; x<EEPROM_CONFIG4_START; x++)
  {
    offset = x - EEPROM_CONFIG3_YBINS;
    ignitionTable.axisY[offset] = readConfigByte(x) * TABLE_LOAD_MULTIPLIER; //Table load is divided by 2 (Allows for MAP up to 511)
  }

  pnt_configPage = (byte *)&configPage4; //Create a pointer to Page 4 in memory
  for(int x=EEPROM_CONFIG4_START; x<EEPROM_CONFIG4_END; x++)
  {
    *(pnt_configPage + byte(x - EEPROM_CONFIG4_START)) = readConfigByte(x);
  }

  //*********************************************************************************************************************************************************************************
  //AFR TARGET CONFIG PAGE (3)

  //Begin writing the Ignition table, basically the same thing as above
  for(int x=EEPROM_CONFIG5_MAP; x<EEPROM_CONFIG5_XBINS; x++)
  {
    offset = x - EEPROM_CONFIG5_MAP;
    afrTable.values[15-(offset/16)][offset%16] = readConfigByte(x); //Read the 16x16 map
  }
  //RPM bins
  for(int x=EEPROM_CONFIG5_XBINS; x<EEPROM_CONFIG5_YBINS; x++)
  {
    offset = x - EEPROM_CONFIG5_XBINS;
    afrTable.axisX[offset] = (readConfigByte(x) * TABLE_RPM_MULTIPLIER); //RPM bins are divided by 100 when stored. Multiply them back now
  }
  //TPS/MAP bins
  for(int x=EEPROM_CONFIG5_YBINS; x<EEPROM_CONFIG6_START; x++)
  {
    offset = x - EEPROM_CONFIG5_YBINS;
    afrTable.axisY[offset] = readConfigByte(x) * TABLE_LOAD_MULTIPLIER; //Table load is divided by 2 (Allows for MAP up to 511)
  }

  pnt_configPage = (byte *)&configPage6; //Create a pointer to Page 6 in memory
  for(int x=EEPROM_CONFIG6_START; x<EEPROM_CONFIG6_END; x++)
  {
    *(pnt_configPage + byte(x - EEPROM_CONFIG6_START)) = readConfigByte(x);
  }

  //*********************************************************************************************************************************************************************************
  // Boost and vvt tables load
  int y = EEPROM_CONFIG7_MAP2;
  int z = EEPROM_CONFIG7_MAP3;
  for(int x=EEPROM_CONFIG7_MAP1; x<EEPROM_CONFIG7_XBINS1; x++)
  {
    offset = x - EEPROM_CONFIG7_MAP1;
    boostTable.values[7-(offset/8)][offset%8] = readConfigByte(x); //Read the 8x8 map
    offset = y - EEPROM_CONFIG7_MAP2;
    vvtTable.values[7-(offset/8)][offset%8] = readConfigByte(y); //Read the 8x8 map
    offset = z - EEPROM_CONFIG7_MAP3;
    stagingTable.values[7-(offset/8)][offset%8] = readConfigByte(z); //Read the 8x8 map
    y++;
    z++;
  }

  //RPM bins
  y = EEPROM_CONFIG7_XBINS2;
  z = EEPROM_CONFIG7_XBINS3;
  for(int x=EEPROM_CONFIG7_XBINS1; x<EEPROM_CONFIG7_YBINS1; x++)
  {
    offset = x - EEPROM_CONFIG7_XBINS1;
    boostTable.axisX[offset] = (readConfigByte(x) * TABLE_RPM_MULTIPLIER); //RPM bins are divided by 100 when stored. Multiply them back now
    offset = y - EEPROM_CONFIG7_XBINS2;
    vvtTable.axisX[offset] = (readConfigByte(y) * TABLE_RPM_MULTIPLIER); //RPM bins are divided by 100 when stored. Multiply them back now
    offset = z - EEPROM_CONFIG7_XBINS3;
    stagingTable.axisX[offset] = (readConfigByte(z) * TABLE_RPM_MULTIPLIER); //RPM bins are divided by 100 when stored. Multiply them back now
    y++;
    z++;
  }

  //TPS/MAP bins
  y = EEPROM_CONFIG7_YBINS2;
  z = EEPROM_CONFIG7_YBINS3;
  for(int x=EEPROM_CONFIG7_YBINS1; x<EEPROM_CONFIG7_XSIZE2; x++)
  {
    offset = x - EEPROM_CONFIG7_YBINS1;
    boostTable.axisY[offset] = readConfigByte(x); //TABLE_LOAD_MULTIPLIER is NOT used for boost as it is TPS based (0-100)
    offset = y - EEPROM_CONFIG7_YBINS2;
    vvtTable.axisY[offset] = readConfigByte(y); //TABLE_LOAD_MULTIPLIER is NOT used for VVT as it is TPS based (0-100)
    offset = z - EEPROM_CONFIG7_YBINS3;
    stagingTable.axisY[offset] = readConfigByte(z) * TABLE_LOAD_MULTIPLIER;
    y++;
    z++;
  }

  //*********************************************************************************************************************************************************************************
  // Fuel trim tables load
  y = EEPROM_CONFIG8_MAP2;
  z = EEPROM_CONFIG8_MAP3;
  int i = EEPROM_CONFIG8_MAP4;
  for(int x=EEPROM_CONFIG8_MAP1; x<EEPROM_CONFIG8_XBINS1; x++)
  {
    offset = x - EEPROM_CONFIG8_MAP1;
    trim1Table.values[5-(offset/6)][offset%6] = readConfigByte(x); //Read the 6x6 map
    offset = y - EEPROM_CONFIG8_MAP2;
    trim2Table.values[5-(offset/6)][offset%6] = readConfigByte(y); //Read the 6x6 map
    offset = z - EEPROM_CONFIG8_MAP3;
    trim3Table.values[5-(offset/6)][offset%6] = readConfigByte(z); //Read the 6x6 map
    offset = i - EEPROM_CONFIG8_MAP4;
    trim4Table.values[5-(offset/6)][offset%6] = readConfigByte(i); //Read the 6x6 map
    y++;
    z++;
    i++;
  }

  //RPM bins
  y = EEPROM_CONFIG8_XBINS2;
  z = EEPROM_CONFIG8_XBINS3;
  i = EEPROM_CONFIG8_XBINS4;
  for(int x=EEPROM_CONFIG8_XBINS1; x<EEPROM_CONFIG8_YBINS1; x++)
  {
    offset = x - EEPROM_CONFIG8_XBINS1;
    trim1Table.axisX[offset] = (readConfigByte(x) * TABLE_RPM_MULTIPLIER); //RPM bins are divided by 100 when stored. Multiply them back now
    offset = y - EEPROM_CONFIG8_XBINS2;
    trim2Table.axisX[offset] = (readConfigByte(y) * TABLE_RPM_MULTIPLIER); //RPM bins are divided by 100 when stored. Multiply them back now
    offset = z - EEPROM_CONFIG8_XBINS3;
    trim3Table.axisX[offset] = (readConfigByte(z) * TABLE_RPM_MULTIPLIER); //RPM bins are divided by 100 when stored. Multiply them back now
    offset = i - EEPROM_CONFIG8_XBINS4;
    trim4Table.axisX[offset] = (readConfigByte(i) * TABLE_RPM_MULTIPLIER); //RPM bins are divided by 100 when stored. Multiply them back now
    y++;
    z++;
    i++;
  }

  //TPS/MAP bins
  y = EEPROM_CONFIG8_YBINS2;
  z = EEPROM_CONFIG8_YBINS3;
  i = EEPROM_CONFIG8_YBINS4;
  for(int x=EEPROM_CONFIG8_YBINS1; x<EEPROM_CONFIG8_XSIZE2; x++)
  {
    offset = x - EEPROM_CONFIG8_YBINS1;
    trim1Table.axisY[offset] = readConfigByte(x) * TABLE_LOAD_MULTIPLIER; //Table load is divided by 2 (Allows for MAP up to 511)
    offset = y - EEPROM_CONFIG8_YBINS2;
    trim2Table.axisY[offset] = readConfigByte(y) * TABLE_LOAD_MULTIPLIER; //Table load is divided by 2 (Allows for MAP up to 511)
    offset = z - EEPROM_CONFIG8_YBINS3;
    trim3Table.axisY[offset] = readConfigByte(z) * TABLE_LOAD_MULTIPLIER; //Table load is divided by 2 (Allows for MAP up to 511)
    offset = i - EEPROM_CONFIG8_YBINS4;
    trim4Table.axisY[offset] = readConfigByte(i) * TABLE_LOAD_MULTIPLIER; //Table load is divided by 2 (Allows for MAP up to 511)
    y++;
    z++;
    i++;
  }
  //*********************************************************************************************************************************************************************************
  //canbus control page load
    pnt_configPage = (byte *)&configPage9; //Create a pointer to Page 10 in memory
  for(int x=EEPROM_CONFIG9_START; x<EEPROM_CONFIG9_END; x++)
  {
    *(pnt_configPage + byte(x - EEPROM_CONFIG9_START)) = readConfigByte(x);
  }

  //*********************************************************************************************************************************************************************************

  //CONFIG PAGE (10)
  pnt_configPage = (byte *)&configPage10; //Create a pointer to Page 11 in memory
  //All 192 bytes can simply be pulled straight from the configTable
  for(int x=EEPROM_CONFIG10_START; x<EEPROM_CONFIG10_END; x++)
  {
    *(pnt_configPage + byte(x - EEPROM_CONFIG10_START)) = readConfigByte(x);
  }

  //*********************************************************************************************************************************************************************************
  //Fuel table 2 (See storage.h for data layout)
  for(int x=EEPROM_CONFIG11_MAP; x<EEPROM_CONFIG11_XBINS; x++)
  {
    offset = x - EEPROM_CONFIG11_MAP;
    fuelTable2.values[15-(offset/16)][offset%16] = readConfigByte(x); //Read the 8x8 map
  }
  //RPM bins
  for(int x=EEPROM_CONFIG11_XBINS; x<EEPROM_CONFIG11_YBINS; x++)
  {
    offset = x - EEPROM_CONFIG11_XBINS;
    fuelTable2.axisX[offset] = (readConfigByte(x) * TABLE_RPM_MULTIPLIER); //RPM bins are divided by 100 when stored. Multiply them back now
  }
  //TPS/MAP bins
  for(int x=EEPROM_CONFIG11_YBINS; x<EEPROM_CONFIG11_END; x++)
  {
    offset = x - EEPROM_CONFIG11_YBINS;
    fuelTable2.axisY[offset] = readConfigByte(x) * TABLE_LOAD_MULTIPLIER;
  }

}

/*
Reads the calibration information from EEPROM.
This is separate from the config load as the calibrations do not exist as pages within the ini file for Tuner Studio
*/
void loadCalibration()
{

  for(int x=0; x<CALIBRATION_TABLE_SIZE; x++) //Each calibration table is 512 bytes long
  {
    int y = EEPROM_CALIBRATION_CLT + x;
    cltCalibrationTable[x] = readConfigByte(y);

    y = EEPROM_CALIBRATION_IAT + x;
    iatCalibrationTable[x] = readConfigByte(y);

    y = EEPROM_CALIBRATION_O2 + x;
    o2CalibrationTable[x] = readConfigByte(y);
  }

}

/*
This takes the values in the 3 calibration tables (Coolant, Inlet temp and O2)
and saves them to the EEPROM.
*/
void writeCalibration()
{

  for(int x=0; x<CALIBRATION_TABLE_SIZE; x++) //Each calibration table is 512 bytes long
  {
    int y = EEPROM_CALIBRATION_CLT + x;
    if(readConfigByte(y) != cltCalibrationTable[x]) { writeConfigByte(y, cltCalibrationTable[x]); }

    y = EEPROM_CALIBRATION_IAT + x;
    if(readConfigByte(y) != iatCalibrationTable[x]) { writeConfigByte(y, iatCalibrationTable[x]); }

    y = EEPROM_CALIBRATION_O2 + x;
    if(readConfigByte(y) != o2CalibrationTable[x]) { writeConfigByte(y, o2CalibrationTable[x]); }
  }
  flushConfigBuffer();
}

/*
Takes a page number and CRC32 value then stores it in the relevant place in EEPROM
Note: Each pages requires 4 bytes for its CRC32. These are stored in reverse page order (ie the last page is store first in EEPROM)
*/
void storePageCRC32(byte pageNo, uint32_t crc32_val)
{
  uint16_t address; //Start address for the relevant page
  address = EEPROM_PAGE_CRC32 + ((NUM_PAGES - pageNo) * 4);

  //One = Most significant -> Four = Least significant byte
  byte four = (crc32_val & 0xFF);
  byte three = ((crc32_val >> 8) & 0xFF);
  byte two = ((crc32_val >> 16) & 0xFF);
  byte one = ((crc32_val >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  updateConfigByte(address, four);
  updateConfigByte(address + 1, three);
  updateConfigByte(address + 2, two);
  updateConfigByte(address + 3, one);
  flushConfigBuffer();
}

/*
Retrieves and returns the 4 byte CRC32 for a given page from EEPROM
*/
uint32_t readPageCRC32(byte pageNo)
{
  uint16_t address; //Start address for the relevant page
  address = EEPROM_PAGE_CRC32 + ((NUM_PAGES - pageNo) * 4);

  //Read the 4 bytes from the eeprom memory.
  uint32_t four = readConfigByte(address);
  uint32_t three = readConfigByte(address + 1);
  uint32_t two = readConfigByte(address + 2);
  uint32_t one = readConfigByte(address + 3);

  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

// Utility functions.
// By having these in this file, it prevents other files from calling EEPROM functions directly. This is useful due to differences in the EEPROM libraries on different devces
byte readLastBaro() { return readConfigByte(EEPROM_LAST_BARO); }
void storeLastBaro(byte newValue) { updateConfigByte(EEPROM_LAST_BARO, newValue); flushConfigBuffer();}
void storeCalibrationValue(uint16_t location, byte value) { updateConfigByte(location, value);} //This is essentially just an abstraction for updateConfigByte()
byte readEEPROMVersion() { return readConfigByte(EEPROM_DATA_VERSION); }
void storeEEPROMVersion(byte newVersion) { updateConfigByte(EEPROM_DATA_VERSION, newVersion); flushConfigBuffer();}
