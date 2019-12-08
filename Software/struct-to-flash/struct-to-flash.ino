#include <SPI.h>
#include "DataFlash.h"

static const int csPin    = 10;
static const int resetPin = 8;
static const int wpPin    = 7;

DataFlash dataflash;

struct State {
  float qw;
  float qx;
  float qy;
  float qz;
  float dx;
  float dz;
  float ux;
  float uz;
};

void setup() {
  Serial.begin(115200);
  Serial.println(sizeof(State));

  SPI.begin();
  dataflash.setup(csPin, resetPin, wpPin);
  Serial.println("Dataflash set up");
  dataflash.begin();
  Serial.println("Dataflash started");

  State state = { 0.865, 0.501, 0.328, 0.761, -0.321, 0.086, 12.3, -3.2 };
  State emptyState = { 0, 0, 0, 0, 0, 0, 0, 0 };

  byte* p = (byte*) &state;
  byte* e = (byte*) &emptyState;

  Serial.println(emptyState.qw);
  Serial.println(emptyState.qx);
  Serial.println(emptyState.qy);
  Serial.println(emptyState.qz);
  Serial.println(emptyState.dx);
  Serial.println(emptyState.dz);
  Serial.println(emptyState.ux);
  Serial.println(emptyState.uz);

  Serial.println("State variables and pointer initialized");

  long start = micros();
  dataflash.bufferWrite(1, 0);
  for(int i = 0; i < sizeof(State); i++) {
    SPI.transfer(*p++);
  }
  dataflash.bufferToPage(1, 3);
  long end = micros();
  long duration = end - start;
  Serial.print("Duration of bufferWrite, transfer and bufferToPage operations: ");
  Serial.println(duration);

  dataflash.pageToBuffer(3, 2);
  dataflash.bufferRead(2, 0);
  for(int i = 0; i < sizeof(State); i++) {
    uint8_t data = SPI.transfer(0xff);
    *e = data;
    e++;
    Serial.print(data);
    Serial.print(" ");
  }  

  Serial.println();
  Serial.println(emptyState.qw);
  Serial.println(emptyState.qx);
  Serial.println(emptyState.qy);
  Serial.println(emptyState.qz);
  Serial.println(emptyState.dx);
  Serial.println(emptyState.dz);
  Serial.println(emptyState.ux);
  Serial.println(emptyState.uz);

  SPI.end();
}

void loop() {
  // put your main code here, to run repeatedly:

}
