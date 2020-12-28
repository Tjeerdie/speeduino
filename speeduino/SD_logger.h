#ifndef SD_H
#define SD_H
#include "globals.h"
#ifdef SD_CARD_LOGGER_ENABLED
#include SD_CARD_LIB_H

#define SD_STATUS_OFF               0
#define SD_STATUS_FS_READY          1
#define SD_STATUS_CARD_READY        2
#define SD_STATUS_LOGGING           4
#define SD_STATUS_ERROR_NO_WRITE    8
#define WRITE_BUFFER_SIZE           16384U
#define WRITE_TRIGGER_BYTES         512U //when to write to sdcard. minmum is 512 bytes, and always must be multiple of 512 bytes for efficiency 


#ifdef CORE_TEENSY
    #define SD_CS_PIN BUILTIN_SDCARD
#elif STM32F4
    #define SD_CS_PIN SD_DETECT_NONE
#else
    #define SD_CS_PIN 10 //This is a made up value for now
#endif

//Sd2Card SD;
//SdVolume SD_volume;

File logFile;
uint8_t LogBufferBIN[128]; 
char LogBufferCSV[WRITE_BUFFER_SIZE]; 
char LogBufferCSVfield[32];
char filename[32];
uint16_t bufferIndex;
uint16_t Bufferswritten;
uint32_t Total_bytes_written;

void updateLogdataBIN();
void SDinit();
void SDopenLogFile();
void SDcloseLogFile();
void SDwriteLogEntry();
void updateLogdataCSV();




#endif //SD_LOGGING
#endif //SD_H
