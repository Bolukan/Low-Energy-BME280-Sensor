/*
  RTCMEM.h

  A wrapper for the ESP8266 RTC memory

 */

#ifndef RTCMEM_H
#define RTCMEM_H

#include <arduino.h>

class RTCMEM
{
public:

 // initialiser - for future use
 RTCMEM();

 // clear memory and set to be expected recordLength (bytes) per record
 void reset(uint8_t recordLength);

 // load RTC memory in rtcData structure, calculate crc32 and store in _crc32
 bool loadMem();
 // Calculate crc32 and store in rtcData, copy data to RTC
 bool saveMem();
 // CRC32 of data is equal to CRC32 in memory
 bool isValid();

 uint8_t counter();
 // number of bytes per record
 uint8_t recordLength();
 // number of valid records (0-508)
 uint16_t recordCount();

 // add Record to memory
 bool addRecord(const void* record, int recordSize);

 void getRecord(void* record, const int recordnr);

 //
 bool isMemoryFull();

 // DEBUG
 void printMemory();

private:

  // Structure for data in RTC memory.
  // First field is CRC32, which is calculated based on the rest.
  struct {
    uint32_t crc32;          // 0-3
    uint8_t  counter;        // 4
    uint8_t  recordLength;   // 5
    uint16_t recordCount;    // 6-7
    uint8_t  data[504];      // 8-511
  } rtcData;

  // checksum on current data
  uint32_t calculateCRC32();

}; // class RTCMEM

#endif
