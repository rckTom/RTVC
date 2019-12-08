#include <SPI.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif
#include "DataFlash.h"
#include "MS5611.h"

#define ON_RAMP 0 // constants for states of state machine
#define IN_FLIGHT 1
#define RECOVERY 2

int currentState = 0;

// variables for handling interrupts
bool active = false; // only count if true
bool intr = false; // set to true by interrupt, act on by loop()
int ovf_ct = 0; // keep track of how often interrupt overflowed

MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements

MS5611 ms5611;

long referencePressure;

DataFlash dataflash;

static const int csPin    = 10;
static const int resetPin = 8;
static const int wpPin    = 7;

uint8_t cycles;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// PID controller gains
#define K_P -2.0 // DRAGONS
#define K_I -1.8 // DRAGONS
#define K_D -0.3 // DRAGONS

// mechanical linkage "gain" between desired gain and servo command
#define K_X_mechanic 1.0
#define K_Z_mechanic 1.0

float displacementX = 0.0;
float displacementZ = 0.0;
float oldDisplacementX = 0.0;
float oldDisplacementZ = 0.0;

float X_axis_integral_error = 0.0;
float Z_axis_integral_error = 0.0;
float X_axis_differential_error = 0.0;
float Z_axis_differential_error = 0.0;

#define TIMESTEP 5 // timestep in multiples of 2.048 ms
float delta_t = 0.01024; // in seconds

float responseX = 0.0;
float responseZ = 0.0;

#define SIZE_OF_STATE 36
struct State { // total size 36 bytes
  long p;
  float qw;
  float qx;
  float qy;
  float qz;
  float dx;
  float dz;
  float ux;
  float uz;
};
State state;
byte* pState = (byte*) &state;

int currentCycle = 0;
int currentPage = 0;
int currentBuffer = 1;
#define CYCLES_PER_PAGE 14 // floor(528 / 36)

long flightStart;

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
 
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
  
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  SPI.begin();
  dataflash.setup(csPin, resetPin, wpPin);
  dataflash.begin();
  SPI.end();
  
  ms5611.begin(MS5611_ULTRA_LOW_POWER);
  referencePressure = ms5611.readPressureAtConstantTemperature();

  currentState = IN_FLIGHT;
  flightStart = millis();
  Serial.print("Flight started at "); Serial.println(flightStart);

  // setup Timer2 for interrupts
  setupTimer2();
  active = true;
}

void on_ramp() {
  intr = false;
}

void in_flight() {

  // GET MPU DATA
  
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
  
    mpu.dmpGetQuaternion(&q, fifoBuffer);
  }

  // GETTING MPU DATA: 2.1 milliseconds

  long currentPressure = ms5611.readPressureAtConstantTemperature();

  // GETTING PRESSURE DATA AT CONST TEMPERATURE: 4.2 milliseconds

  // direction of flight is assumed to be Y
  // back of breakout board
  //
  //          VCC_IN
  //  O       3.3V
  //      Y   GND
  //      ^   SCL
  //   X  |   SDA
  //   <--X Z FSYNC
  //          INTA
  // GY-86    DRDY

  displacementX = 2.0 * q.w * q.z + 2.0 * q.x * q.y;
  displacementZ = -2.0 * q.w * q.x + 2.0 * q.y * q.z;

  X_axis_integral_error += displacementX * delta_t;
  X_axis_differential_error = (displacementX - oldDisplacementX) / delta_t;

  Z_axis_integral_error += displacementZ * delta_t;
  Z_axis_differential_error = (displacementZ - oldDisplacementZ) / delta_t;

  responseX = K_X_mechanic * (K_P * displacementX + K_I * X_axis_integral_error + K_D * X_axis_differential_error);
  responseZ = K_Z_mechanic * (K_P * displacementZ + K_I * Z_axis_integral_error + K_D * Z_axis_differential_error);

  oldDisplacementX = displacementX;
  oldDisplacementZ = displacementZ;  

  // CALCULATING RESPONSE FROM QUATERNIONS 0.3 milliseconds 

  state.p = currentPressure;
  state.qw = q.w;
  state.qx = q.x;
  state.qy = q.y;
  state.qz = q.z;
  state.dx = displacementX;
  state.dz = displacementZ;
  state.ux = responseX;
  state.uz = responseZ;

  pState = (byte*) &state;
  SPI.begin();
  dataflash.bufferWrite(currentBuffer, currentCycle * SIZE_OF_STATE);
  for(int i = 0; i < SIZE_OF_STATE; i++) {
    SPI.transfer(*pState++);
  }
  dataflash.bufferToPage(currentBuffer, currentPage);
  SPI.end();

  currentCycle = ++currentCycle % CYCLES_PER_PAGE;
  if(currentCycle == 0) {
    currentPage++;
  }

  if((millis() - flightStart) > 5000) {
    currentState = RECOVERY;
  }

  // SAVE STATE TO FLASH 0.3 milliseconds

  intr = false;
}

void recovery() {
  active = false;
  intr = false;

  currentCycle = 0;
  currentPage = 0;

  State readState;
  byte* pReadState = (byte*) &readState;

  for(int i = 0; i < 500; i++) {

    SPI.begin();
    pReadState = (byte*) &readState;
    dataflash.pageToBuffer(currentPage, 1);
    dataflash.bufferRead(1, currentCycle * SIZE_OF_STATE);
    for(int j = 0; j < SIZE_OF_STATE; j++) {
      uint8_t data = SPI.transfer(0xff);
      Serial.print(data); Serial.print(" ");
      *pReadState = data;
      pReadState++;
    }
    SPI.end();

    Serial.print(readState.qw); Serial.print(" ");
    Serial.print(readState.qx); Serial.print(" ");
    Serial.print(readState.qy); Serial.print(" ");
    Serial.print(readState.qz); Serial.print(" ");
    Serial.print(readState.dx); Serial.print(" ");
    Serial.print(readState.dz); Serial.print(" ");
    Serial.print(readState.ux); Serial.print(" ");
    Serial.println(readState.uz);

    currentCycle = ++currentCycle % CYCLES_PER_PAGE;
    if(currentCycle == 0) {
      currentPage++;
    }
  }

  SPI.end();
  
}

void loop() {
  if(intr) { 
    switch(currentState) {
      case ON_RAMP: on_ramp(); break;
      case IN_FLIGHT: in_flight(); break;
      case RECOVERY: recovery(); break;
    } 
  }
}

void setupTimer2()
{
  noInterrupts();
  
  TCCR2A = 0;
  TCCR2B = 0;

  TCCR2B |= 0x05; // prescale 128
  TIMSK2 |= 0x01; // overflow interrupt enable

  interrupts();
}

ISR(TIMER2_OVF_vect)
{
  if(active) { ovf_ct++; };
  if(ovf_ct==TIMESTEP) { 
    intr = true;
    ovf_ct = 0;
  }
}
