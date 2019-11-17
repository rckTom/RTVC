#include <SPI.h>
#include "DataFlash.h"

static const int csPin    = 10;
static const int resetPin = 8;
static const int wpPin    = 7;

DataFlash dataflash;

void setup()
{
  Serial.begin(115200);
  
  uint8_t status;
  DataFlash::ID id;

  const char* dummyMessage = "Hello world";

  SPI.begin();
  
  dataflash.setup(csPin, resetPin, wpPin);
  dataflash.begin();

  status = dataflash.status();
  dataflash.readID(id);
  // For a brand new AT45DB161D dataflash
  //  status = BIN(00101100) 0b00101100?
  //  id.manufacturer       = 0x1F;
  //  id.device[0]          = 0x26;
  //  id.device[1]          = 0x00;
  //  id.extendedInfoLength = 0x00;
  // For a brand new AT45DB321D dataflash
  //  status = 180 0b00101101 (little-endian)
  //  id.manufacturer       = 0x1F;
  //  id.device[0]          = 0x27;
  //  id.device[1]          = 0x01;
  //  id.extendedInfoLength = 0x00;
  Serial.println(status);
  Serial.println(id.manufacturer);
  Serial.println(id.device[0]);
  Serial.println(id.device[1]);
  Serial.println(id.extendedInfoLength);

  Serial.print("Page size: ");
  Serial.println(DF_45DB321_PAGESIZE);

  // Write "Hello world" to buffer 1.
  dataflash.bufferWrite(1, 0);
  for(int i=0; dummyMessage[i] != '\0'; i++)
  {
    SPI.transfer(dummyMessage[i]);
  }

  // Transfer buffer 1 to page 7.
  dataflash.bufferToPage(1, 7);

  // Read page 5.
  dataflash.pageRead(5, 0);
  for(int i=0; i<DF_45DB321_PAGESIZE; i++)
  {
    uint8_t data = SPI.transfer(0xff);
  }

  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
//  uint8_t buffer256[256];
//  for(int i = 0; i <= 255; i++) {
//    buffer256[i] = i;
//  }
  long start = micros();
  dataflash.bufferWrite(0, 0);
  for(int i = 0; i <= 255; i++) {
    SPI.transfer(i);
  }
  // SPI.transfer(buffer256, 256);
  long afterBuffer = micros();
  dataflash.bufferToPage(0, 1);
  long afterToPage = micros();
  SPI.endTransaction();

  Serial.print("Writing 256 values to buffer: ");
  Serial.println(afterBuffer - start);
  Serial.print("Writing buffer to page: ");
  Serial.println(afterToPage - afterBuffer);

//  dataflash.pageToBuffer(1, 0);
//  dataflash.bufferRead(0, 0);
//  for(int i = 0; i <= 300; i++) {
//    uint8_t data = SPI.transfer(0xff);
//    Serial.print(data);
//    Serial.print(" ");
//  }
}

void loop()
{
  // nothing
}
