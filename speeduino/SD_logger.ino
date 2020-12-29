#include "globals.h"
#ifdef ENABLE_SD_LOGGER
#include "SD_logger.h"
#include "Arduino.h"
#include "src/STM32SD/bsp_sd.h"

//Private functions
void sd_logger_updateLogdataCSV();
void sd_logger_updateLogdataBIN();

//Private variables
File sd_logger_logFile;
uint8_t LogBufferBIN[128]; 
char sd_logger_LogBufferCSV[SD_LOGGER_BUFFER_SIZE]; 
char sd_logger_LogBufferCSVfield[32];
char sd_logger_fileName[32];
uint16_t sd_logger_bufferIndex;
uint16_t sd_logger_bufferswritten;
uint32_t sd_logger_totalBytesWritten;
bool sd_logger_fileNeedsFlush;

void sd_logger_init()
{ 
    currentStatus.sd_status = SD_STATUS_OFF; 
    //Init the sdcard.
    if (SD.begin(SD_CS_PIN)) 
    {
        currentStatus.sd_status |= SD_STATUS_CARD_READY;
    }   
    else { currentStatus.sd_status |= SD_STATUS_ERROR_NO_WRITE; }   
}

void sd_logger_openLogFile()
{
    //Attempt to create a log file for writing
    if(!(currentStatus.sd_status & SD_STATUS_FS_READY))
    {
        //very inefficient needs new implentation but works for now
        sprintf(sd_logger_fileName, "%02d%02d%02d-%02d%02d%02d.csv", rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());

        sd_logger_logFile = SD.open(sd_logger_fileName, FILE_WRITE);
        if(sd_logger_logFile) { currentStatus.sd_status |= SD_STATUS_FS_READY; }
        else { currentStatus.sd_status &= ~SD_STATUS_FS_READY; }
                
        //Write a header to the buffer
        if(currentStatus.sd_status & SD_STATUS_FS_READY)
        {
            //Now it is all hardcoded needs to be able to set from Tuner Studio.  
            const char fields[] =  \
"hasSync;\
RPM;\
MAP;\
TPS;\
tpsDOT;\
mapDOT;\
rpmDOT;\
VE1;\
VE2;\
O2;\
O2_2;\
coolant;\
IAT;\
dwell;\
battery10;\
advance;\
advance1;\
advance2;\
corrections;\
AEamount;\
egoCorrection;\
wueCorrection;\
batCorrection;\
iatCorrection;\
baroCorrection;\
launchCorrection;\
flexCorrection;\
fuelTempCorrection;\
flexIgnCorrection;\
afrTarget;\
idleDuty;\
CLIdleTarget;\
idleUpActive;\
CTPSActive;\
fanOn;\
ethanolPct;\
fuelTemp;\
AEEndTime;\
status1;\
spark;\
spark2;\
engine;\
PW1;\
PW2;\
PW3;\
PW4;\
PW5;\
PW6;\
PW7;\
PW8;\
runSecs;\
secl;\
loopsPerSecond;\
launchingSoft;\
launchingHard;\
freeRAM;\
startRevolutions;\
boostTarget;\
testOutputs;\
testActive;\
boostDuty;\
idleLoad;\
status3;\
flexBoostCorrection;\
nitrous_status;\
fuelLoad;\
fuelLoad2;\
ignLoad;\
fuelPumpOn;\
syncLossCounter;\
knockRetard;\
knockActive;\
toothLogEnabled;\
compositeLogEnabled;\
vvt1Angle;\
vvt1Angle;\
vvt1TargetAngle;\
vvt1Duty;\
injAngle;\
ASEValue;\
vss;\
idleUpOutputActive;\
gear;\
fuelPressure;\
oilPressure;\
engineProtectStatus;\
wmiPW;\
\n";
                                    
            memcpy(sd_logger_LogBufferCSV, fields, sizeof(fields));
            sd_logger_bufferIndex += sizeof(fields);
        }
    }

}


//this function needs to be called to close the file. When the board loses power this one is not called and the file is not closed poperly.
//This creates orphan sectors on the card, and also a file of 0 kbyte size. 
void sd_logger_closeLogFile()
{   
    uint32_t millisstart = millis();
    while(BSP_SD_GetCardState())
    {   
        if((millis()-millisstart)>SD_LOGGER_CLOSE_FILE_TOUT)
        {
            currentStatus.sd_status = SD_STATUS_ERROR_NO_WRITE;
            return;}
    };

    //write buffer to sdcard before closing
    sd_logger_logFile.write(sd_logger_LogBufferCSV, sd_logger_bufferIndex);  
    sd_logger_bufferIndex = 0;
    if(sd_logger_logFile){
        sd_logger_logFile.close();
        sd_logger_bufferIndex = 0;
    }
    currentStatus.sd_status &= ~SD_STATUS_FS_READY;
    currentStatus.sd_status &= ~SD_STATUS_LOGGING;
}

void sd_logger_writeLogEntry()
{
    uint16_t bytes_written = 0;
    uint32_t microstart = micros();
    currentStatus.sd_status |= SD_STATUS_LOGGING;

    //only run logger if file is acutally open.
    if(currentStatus.sd_status & SD_STATUS_FS_READY)
    {
        //Create CSV fields.
        sd_logger_updateLogdataCSV();

        //write debugging time to sdcard to see at what point it fails (NOT used in speeduino firmware testing)
        if ((sd_logger_bufferIndex > SD_LOGGER_WRITE_TRIGGER)){
            //if buffer is filling up stop logging
            if (sd_logger_bufferIndex >= SD_LOGGER_BUFFER_SIZE-128){
                currentStatus.sd_status = SD_STATUS_ERROR_NO_WRITE;

                //Try to close the file else all data is lossed. 
                sd_logger_closeLogFile();
                
            }
            
            //Only try to write data to the sdcard when the card is ready.
            if (BSP_SD_GetCardState()==0){
                if (!sd_logger_fileNeedsFlush){
                    bytes_written = sd_logger_logFile.write(&sd_logger_LogBufferCSV[0], SD_LOGGER_WRITE_TRIGGER); 
                    sd_logger_bufferIndex -= bytes_written;
                    sd_logger_totalBytesWritten += bytes_written;
                    memcpy(&sd_logger_LogBufferCSV, &sd_logger_LogBufferCSV[bytes_written], sd_logger_bufferIndex);
                    sd_logger_bufferswritten++;
                }else
                {
                    sd_logger_logFile.flush();
                    sd_logger_fileNeedsFlush = false;
                    // Serial.println("Re-opend file");
                    // sd_logger_logFile = SD.open(sd_logger_fileName, FILE_WRITE);
                    // sd_logger_logFile.seek(sd_logger_logFile.size());
                    // sd_logger_fileNeedsFlush = false;
                }
                

            }

                        
            if (sd_logger_bufferswritten>30)
            {   //sd_logger_logFile.flush();
                // sd_logger_logFile.close();
                sd_logger_fileNeedsFlush = true;
                sd_logger_bufferswritten = 0;
                // Serial.println("Flushed file");
            }           

      }
           
    }

}
void updateCSVField(long value, bool lastValue)
{

    //Make string out of the initger values in current status
    itoa(value,sd_logger_LogBufferCSVfield,10);
    uint16_t length = strlen(sd_logger_LogBufferCSVfield);

    //Copy string from temp buffer to logbuffer 
    memcpy(&sd_logger_LogBufferCSV[sd_logger_bufferIndex], &sd_logger_LogBufferCSVfield, length);

    //increase logbuffer index to newwest position 
    sd_logger_bufferIndex += length;

    //Every field has a seperator, add this to the fix too
    sd_logger_LogBufferCSV[sd_logger_bufferIndex] = ';';
    sd_logger_bufferIndex += 1;

    //end of line charter needs to be added by last value.
    if (lastValue)
    {   
        sd_logger_LogBufferCSV[sd_logger_bufferIndex] = '\n';
        sd_logger_bufferIndex += 1;
        }
}

void sd_logger_updateLogdataCSV()
{
    
    // Looping over a struct is not possible. 
    // Using the fullStatus[] with updateFullStatus()? 
    updateCSVField(currentStatus.hasSync, false);
    updateCSVField(currentStatus.RPM, false);
    updateCSVField(currentStatus.MAP, false);
    updateCSVField(currentStatus.TPS, false);
    updateCSVField(currentStatus.tpsDOT, false);
    updateCSVField(currentStatus.mapDOT, false);
    updateCSVField(currentStatus.rpmDOT, false);
    updateCSVField(currentStatus.VE1, false);
    updateCSVField(currentStatus.VE2, false);
    updateCSVField(currentStatus.O2, false);
    updateCSVField(currentStatus.O2_2, false);
    updateCSVField(currentStatus.coolant, false);
    updateCSVField(currentStatus.IAT, false);
    updateCSVField(currentStatus.dwell, false);
    updateCSVField(currentStatus.battery10, false);
    updateCSVField(currentStatus.advance, false);
    updateCSVField(currentStatus.advance1, false);
    updateCSVField(currentStatus.advance2, false);
    updateCSVField(currentStatus.corrections, false);
    updateCSVField(currentStatus.AEamount, false);
    updateCSVField(currentStatus.egoCorrection, false);
    updateCSVField(currentStatus.wueCorrection, false);
    updateCSVField(currentStatus.batCorrection, false);
    updateCSVField(currentStatus.iatCorrection, false);
    updateCSVField(currentStatus.baroCorrection, false);
    updateCSVField(currentStatus.launchCorrection, false);
    updateCSVField(currentStatus.flexCorrection, false);
    updateCSVField(currentStatus.fuelTempCorrection, false);
    updateCSVField(currentStatus.flexIgnCorrection, false);
    updateCSVField(currentStatus.afrTarget, false);
    updateCSVField(currentStatus.idleDuty, false);
    updateCSVField(currentStatus.CLIdleTarget, false);
    updateCSVField(currentStatus.idleUpActive, false);
    updateCSVField(currentStatus.CTPSActive, false);
    updateCSVField(currentStatus.fanOn, false);
    updateCSVField(currentStatus.ethanolPct, false);
    updateCSVField(currentStatus.fuelTemp, false);
    updateCSVField(currentStatus.AEEndTime, false);
    updateCSVField(currentStatus.status1, false);
    updateCSVField(currentStatus.spark, false);
    updateCSVField(currentStatus.spark2, false);
    updateCSVField(currentStatus.engine, false);
    updateCSVField(currentStatus.PW1, false);
    updateCSVField(currentStatus.PW2, false);
    updateCSVField(currentStatus.PW3, false);
    updateCSVField(currentStatus.PW4, false);
    updateCSVField(currentStatus.PW5, false);
    updateCSVField(currentStatus.PW6, false);
    updateCSVField(currentStatus.PW7, false);
    updateCSVField(currentStatus.PW8, false);
    updateCSVField(currentStatus.runSecs, false);
    updateCSVField(currentStatus.secl, false);
    updateCSVField(currentStatus.loopsPerSecond, false);
    updateCSVField(currentStatus.launchingSoft, false);
    updateCSVField(currentStatus.launchingHard, false);
    updateCSVField(currentStatus.freeRAM, false);
    updateCSVField(currentStatus.startRevolutions, false);
    updateCSVField(currentStatus.boostTarget, false);
    updateCSVField(currentStatus.testOutputs, false);
    updateCSVField(currentStatus.testActive, false);
    updateCSVField(currentStatus.boostDuty, false);
    updateCSVField(currentStatus.idleLoad, false);
    updateCSVField(currentStatus.status3, false);
    updateCSVField(currentStatus.flexBoostCorrection, false);
    updateCSVField(currentStatus.nitrous_status, false);
    updateCSVField(currentStatus.fuelLoad, false);
    updateCSVField(currentStatus.fuelLoad2, false);
    updateCSVField(currentStatus.ignLoad, false);
    updateCSVField(currentStatus.fuelPumpOn, false);
    updateCSVField(currentStatus.syncLossCounter, false);
    updateCSVField(currentStatus.knockRetard, false);
    updateCSVField(currentStatus.knockActive, false);
    updateCSVField(currentStatus.toothLogEnabled, false);
    updateCSVField(currentStatus.compositeLogEnabled, false);
    updateCSVField(currentStatus.vvt1Angle, false);
    updateCSVField(currentStatus.vvt1Angle, false);
    updateCSVField(currentStatus.vvt1TargetAngle, false);
    updateCSVField(currentStatus.vvt1Duty, false);
    updateCSVField(currentStatus.injAngle, false);
    updateCSVField(currentStatus.ASEValue, false);
    updateCSVField(currentStatus.vss, false);
    updateCSVField(currentStatus.idleUpOutputActive, false);
    updateCSVField(currentStatus.gear, false);
    updateCSVField(currentStatus.fuelPressure, false);
    updateCSVField(currentStatus.oilPressure, false);
    updateCSVField(currentStatus.engineProtectStatus, false);
    updateCSVField(currentStatus.wmiPW, true);
}

void sd_logger_updateLogdataBIN()
{
 
}
#endif