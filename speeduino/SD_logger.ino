
#include "globals.h"
#ifdef SD_CARD_LOGGER_ENABLED
#include "SD_logger.h"
#include "Arduino.h"

void updateLogdataBIN()
{
 
}

void SDinit()
{ 
    int milistart = millis();
    currentStatus.sd_status = SD_STATUS_OFF; 
    //Init the connection to the card reader
    if (SD.begin(SD_CS_PIN)) 
    {
        currentStatus.sd_status |= SD_STATUS_CARD_READY;
    }   
    else { currentStatus.sd_status |= SD_STATUS_ERROR_NO_WRITE; }

    //Debug lines on serial port 1.
    Serial1.printf("initSD took: %02d\n", (millis()-milistart));
    
}

void SDopenLogFile()
{
    int milistart = millis();

    //Attempt to create a log file for writing
    if(!(currentStatus.sd_status & SD_STATUS_FS_READY))
    {
        //very inefficient needs new implentation but works for now
        sprintf(filename, "%02d%02d%02d-%02d%02d%02d.csv", rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
        logFile = SD.open(filename, FILE_WRITE);
        if(logFile) { currentStatus.sd_status |= SD_STATUS_FS_READY; }
        else { currentStatus.sd_status &= ~SD_STATUS_FS_READY; }

        //Write a header to the buffer
        if(currentStatus.sd_status & SD_STATUS_FS_READY)
        {
            //now it is all hardcoded needs to be able to set from Tuner Studio.  
            const char fields[] = "RPM;MAP;TPS;tpsDOT;mapDOT;rpmDOT;VE1;VE2;O2;O2_2;coolant;IAT;dwell;etc;etc;etc\n";
            memcpy(LogBufferCSV, fields, sizeof(fields));
            bufferIndex += sizeof(fields);
        }
        // logFile.flush();
        // logFile.close();

        //Debug timing to get a sense how long it takes
        Serial1.printf("openLogFile took: %02d\n", (millis()-milistart));
    }



}


//this function needs to be called to close the file. When the board loses power this one is not called and the file is not closed poperly.
//This creates orphan sectors on the card, and also a file of 0 kbyte size. 
void SDcloseLogFile()
{   
    //Some debugging that can be used to see what is going on in speeduino 
    // Serial1.printf("rtc_mode: %02d\n",configPage13.rtc_mode);
    // Serial1.printf("rtc_trim: %02d\n",configPage13.rtc_trim);
    // Serial1.printf("sd_log_file_style: %02d\n",configPage13.sd_log_file_style);
    // Serial1.printf("sd_log_filenaming: %02d\n",configPage13.sd_log_filenaming);
    // Serial1.printf("sd_log_trigger_style: %02d\n",configPage13.sd_log_trigger_style);

    //Timing measurenements
    int microstart = micros();

    //write buffer to sdcard before closing
    logFile.write(LogBufferCSV, bufferIndex);  
    bufferIndex = 0;
    if(logFile){
        logFile.close();
        bufferIndex = 0;
    }
    currentStatus.sd_status &= ~SD_STATUS_FS_READY;
    Serial1.print("closing file takes us: ");
    Serial1.println(micros()-microstart);
}

void SDwriteLogEntry()
{
    uint16_t bytes_written;
    uint32_t microstart = micros();

    //only run logger if file is acutally open.
    if(currentStatus.sd_status & SD_STATUS_FS_READY)
    {

      
    //   updateLogdataBIN();
    //   bytes_written = logFile.write(LogBufferBIN, sizeof(LogBufferBIN));
      
      updateLogdataCSV();
    //   Serial1.printf("updateLogdataCSV took: %02d\n", (micros()-microstart));

    //write debugging time to sdcard to see at what point it fails (NOT used in speeduino firmware testing)
    //   currentStatus.MAP = millis();
      if (bufferIndex > WRITE_TRIGGER_BYTES){
            // logFile = SD.open(filename, FILE_WRITE);  
            // logFile.seek(logFile.size());

            //if buffer is filling up stop logging
            if (bufferIndex >= WRITE_BUFFER_SIZE-128){
                currentStatus.sd_status = SD_STATUS_ERROR_NO_WRITE;

                //Need to close the file else all data is lossed. 
                //closing at this point will lock up the board because comms with sdcard are broken
                // SDcloseLogFile();
                
            }
            bytes_written = logFile.write(LogBufferCSV, WRITE_TRIGGER_BYTES); 
            Total_bytes_written += bytes_written;
            Bufferswritten ++; 
            memcpy(LogBufferCSV, &LogBufferCSV[bytes_written], bytes_written);
            bufferIndex -= bytes_written;
            if (Bufferswritten>32)
            {
                // logFile.flush();
                Bufferswritten = 0;
            }
            
            // logFile.close();

            // Serial1.print("index= ");
            // Serial1.println(bufferIndex);
            // Serial1.print("Buffer= ");
            // Serial1.println(LogBufferCSV);    

            //write debugging info to sdcard to see at what point it fails (NOT used in speeduino firmware testing)
            // currentStatus.RPM = bytes_written;
            // currentStatus.PW1 = Total_bytes_written;
            // Serial1.printf("write log entry: %02d\n",bytes_written);   
            Serial1.printf("SDwrite: %02d uS\n", (micros()-microstart));     
      }

    //   Serial1.printf("SDwriteLogEntry took: %02d\n", (micros()-microstart));
      
      //write debugging info to sdcard to see at what point it fails (NOT used in speeduino firmware testing)
    //   currentStatus.dwell = micros()-microstart;

    }

}
void updateCSVField()
{
    uint16_t length = strlen(LogBufferCSVfield);
    memcpy(&LogBufferCSV[bufferIndex], LogBufferCSVfield, length);
    bufferIndex += length;
    LogBufferCSV[bufferIndex] = ';';
    bufferIndex += 1;
}

void updateLogdataCSV()
{
    
    // Most efficient way so far to to create a csv line. this takes ~200us on stm32
    itoa(currentStatus.RPM,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.MAP,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.TPS,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.tpsDOT,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.mapDOT,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.rpmDOT,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.VE1,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.VE2,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.O2,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.O2_2,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.coolant,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.IAT,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.dwell,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.battery10,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.advance,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.advance1,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.advance2,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.corrections,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.AEamount,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.egoCorrection,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.wueCorrection,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.batCorrection,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.iatCorrection,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.baroCorrection,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.launchCorrection,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.flexCorrection,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.fuelTempCorrection,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.flexIgnCorrection,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.afrTarget,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.idleDuty,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.CLIdleTarget,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.idleUpActive,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.CTPSActive,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.fanOn,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.ethanolPct,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.fuelTemp,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.AEEndTime,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.status1,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.spark,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.spark2,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.engine,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.PW1,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.PW2,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.PW3,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.PW4,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.PW5,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.PW6,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.PW7,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.PW8,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.runSecs,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.secl,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.loopsPerSecond,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.launchingSoft,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.launchingHard,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.freeRAM,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.startRevolutions,LogBufferCSVfield,10);updateCSVField();

    itoa(currentStatus.boostTarget,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.testOutputs,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.testActive,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.boostDuty,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.idleLoad,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.status3,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.flexBoostCorrection,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.nitrous_status,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.fuelLoad,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.fuelLoad2,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.ignLoad,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.fuelPumpOn,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.syncLossCounter,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.knockRetard,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.knockActive,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.toothLogEnabled,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.compositeLogEnabled,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.vvt1Angle,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.vvt1Angle,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.vvt1TargetAngle,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.vvt1Duty,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.injAngle,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.ASEValue,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.vss,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.idleUpOutputActive,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.gear,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.fuelPressure,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.oilPressure,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.engineProtectStatus,LogBufferCSVfield,10);updateCSVField();
    itoa(currentStatus.wmiPW,LogBufferCSVfield,10);updateCSVField();




//   long vvt2Angle;
//   byte vvt2TargetAngle;
//   byte vvt2Duty;
//   byte outputsStatus;
//   byte sd_status;
   //close CSV line 
   LogBufferCSV[bufferIndex] = '\n';
   bufferIndex += 1;
}

#endif