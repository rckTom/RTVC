#include <Dataflash.h> // Library for communicating with a flash chip
#include <Servo_dig.h> // Servo library with 300 Hz for digital servos
#include <Wire.h> // I2C communication library

// define constants necessary for reading AS5048B magnetic absolute encoder
#define PROG_CTRL_REG 3
#define I2C_ADDR_REG 21
#define OTP_ZERO_POS_HI_REG 22
#define OTP_ZERO_POS_LO_REG 23
#define AUTO_GAIN_CTRL_REG 250
#define DIAGN_REG 251
#define MAGN_HI_REG 252
#define MAGN_LO_REG 253
#define ANGLE_HI_REG 255
#define ANGLE_LO_REG 254

#define INT2ANG 0.021973 // constant to convert 14-bit integer to angle (0°-360°)

int AS5048B_addr = 64; // I2C address
int ovf_ct = 0; // overflow counter for timing
int buf_ct = 0; // buffer position counter
int page_ct = 0; // flash memory page counter

bool active = false;
bool intr_angle = false;

#define n_angles 11
int angles[n_angles] = {40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80};
unsigned long t1 = 0;
unsigned long t2 = 0;
byte angle_ct = 0;
#define period 100

// create objects for servo and flash memory
Servo_dig myservo;
Dataflash dflash;

void setup()
{
  Serial.begin(115200); // initiate communication over serial interface to host computer
  Serial.println("servopruefstand 2017-10-22");
  Wire.begin(); // initiate I2C communication
  dflash.init(); //initialize the memory (pins are defined in dataflash.cpp
  myservo.attach(5, 900, 2100); // servo is attached to pin 5 (D5)
  myservo.write(angles[angle_ct]);
  setupTimer2();

  delay(1000); // wait for sensor(s) to be ready
  Serial.println("Initialization complete.");
  
  active = true;

  t1 = millis();
  myservo.write(angles[angle_ct]);
  Serial.print("Servo moved to position ");
  Serial.println(angles[angle_ct]);
}

void loop()
{
  if(intr_angle) { readAndSaveAngle(); }
  t2 = millis();
  if(((t2 - t1) > period) and (angle_ct < n_angles)) {
    angle_ct++;
    myservo.write(angles[angle_ct]);
    t1 = t2;
  }
  if(angle_ct == n_angles) {
    Serial.println("Servo movement finished.");
    active = false;
    dflash.Buffer_To_Page(1, page_ct);
    page_ct++;
    for(int i=0;i<page_ct;i++) {
      dflash.Page_To_Buffer(i, 1);
      Serial.print("page "); Serial.println(i);
      for(int j=0;j<264;j++) {
        byte hi = dflash.Buffer_Read_Byte(1, 2*j);
        byte lo = dflash.Buffer_Read_Byte(1, 2*j+1);
        Serial.println((hi << 8) | lo);
      }
    }
    angle_ct++;
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
  if(ovf_ct==2) { // every 4.096 ms do stuff
  //if(ovf_ct==244) { // every ~0.5 s do stuff
    intr_angle = true;
    ovf_ct = 0;
  }
}

void readAndSaveAngle()
{
  int angle = readAngle(AS5048B_addr, ANGLE_LO_REG, ANGLE_HI_REG);
  byte angle_hi = (angle >> 8);
  byte angle_lo = (angle & 0xff);
  dflash.Buffer_Write_Byte(1, buf_ct,   angle_hi);
  dflash.Buffer_Write_Byte(1, buf_ct+1, angle_lo);
  buf_ct = buf_ct + 2;
  if(buf_ct >= 528) {
    dflash.Buffer_To_Page(1, page_ct);
    page_ct++;
    buf_ct = 0;
  }
  intr_angle = false;
}

int readAngle(int deviceAddress, byte lowRegister, byte highRegister)
{
  byte alpha_hi = readRegister(deviceAddress, highRegister);
  byte alpha_lo = readRegister(deviceAddress, lowRegister);
  int alpha_dec = ((alpha_hi << 6) | (alpha_lo & 0b00111111));
  return alpha_dec;
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}
