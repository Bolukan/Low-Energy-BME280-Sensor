#include <arduino.h>
#include <rtcmem.h>
#include "CRC32.h"

RTCMEM::RTCMEM()
{
}

void RTCMEM::reset(uint8_t recordLength)
{
  memset(&rtcData.data,0,sizeof(rtcData.data));
  rtcData.recordCount = 0;
  rtcData.recordLength = recordLength;
  rtcData.counter++;
}

// load RTC memory in rtcData structure, calculate crc32 and store in _crc32
bool RTCMEM::loadMem()
{
  if (ESP.rtcUserMemoryRead(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
    return isValid();
  } else {
    return false;
  }
}

 // Calculate crc32 and store in rtcData, copy data to RTC
bool RTCMEM::saveMem()
{
  rtcData.crc32 = calculateCRC32();
  return ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData));
}

// CRC32 of data is equal to CRC32 in memory
bool RTCMEM::isValid()
{
  return (rtcData.crc32 == calculateCRC32());
}

// number of mem saves
uint8_t RTCMEM::counter()
{
  return rtcData.counter;
}

// number of bytes per record
uint8_t RTCMEM::recordLength()
{
  return rtcData.recordLength;
}

// number of valid records (0-508)
uint16_t RTCMEM::recordCount()
{
  return rtcData.recordCount;
}

bool RTCMEM::isMemoryFull()
{
    return ((unsigned long long) ((rtcData.recordCount + 1) * rtcData.recordLength) > sizeof(rtcData.data));
}

bool RTCMEM::addRecord(const void* record, int recordSize)
{
  if (isMemoryFull()) {
    // memory full. Event !!
    return false;
  }

  if (recordSize != rtcData.recordLength) {
    // other length
    return false;
  }

  // Save record
  memcpy(&rtcData.data[rtcData.recordCount * rtcData.recordLength], record, recordSize);
  rtcData.recordCount++;

  // Debug memory
  //printMemory();
  return true;
}

 void RTCMEM::getRecord(void* record, const int recordnr)
 {
   memcpy(record, &rtcData.data[recordnr * rtcData.recordLength], recordLength());
 }

// private
// checksum on current data
uint32_t RTCMEM::calculateCRC32()
{
  return CRC32::calculate((uint8_t*) &rtcData.data[0], sizeof(rtcData.data));
}

//prints all rtcData, including the leading crc32
void RTCMEM::printMemory() {
  char buf[3];
  uint8_t *ptr = (uint8_t *)&rtcData;
  for (size_t i = 0; i < sizeof(rtcData); i++) {
    sprintf(buf, "%02X", ptr[i]);
    Serial.print(buf);
    if ((i + 1) % 24 == 0) {
      Serial.println();
    } else {
      Serial.print(" ");
    }
  }
  Serial.println();
}
