#include <Servo_dig.h>
#include <GY80.h>
#include <Wire.h>

// define constants
#define Servo_X_pin 5
#define Servo_Y_pin 6

#define Ralt 0 // DRAGONS
#define Rrot 0 // DRAGONS
#define Qalt 0 // DRAGONS
#define Qrot 0 // DRAGONS

#define TIMESTEP 5 // timestep in multiples of 2.048 ms

// objects for servos and sensors
Servo_dig servo_x, servo_y;
GY80 gy80;

// state variables
int h, v, a;
int gamma, gammadot, gammaddot;
int phi, phidot;

// variables for handling interrupts
bool active = false; // only count if true
bool intr = false; // set to true by interrupt, act on by loop()
int ovf_ct = 0; // keep track of how often interrupt overflowed

void setup() {
  Serial.begin(115200);
  Serial.println("RTVC firmware 2018-01-05");
  Serial.println("Determining reading speed ...");
  
  // attach two servos for x and y axis
  servo_x.attach(Servo_X_pin, 900, 2100);
  servo_y.attach(Servo_Y_pin, 900, 2100);

  // attach sensors over I2C and set measuring ranges
  Wire.begin();
  gy80.a_set_scale(GY80_a_scale_4);
  gy80.a_set_bw(GY80_a_bw_200);
  gy80.g_set_scale(GY80_g_scale_250);

  // initialize state
  h = 0;
  v = 0;
  a = 0;
  gamma = 0;
  gammadot = 0;
  gammaddot = 0;
  phi = 0;
  phidot = 0;

  // variables for reading gyro rates and acceleration
  // reading takes about 4.6 ms
  // additionally reading pressure would take extra 6.1 ms
  GY80_single_scaled accel;
  GY80_single_scaled gyro_rates;

  unsigned long T1 = millis();

  for(int i=0;i<1000;i++) {
    accel = gy80.a_read_scaled();
    gyro_rates = gy80.g_read_scaled();
  }

  unsigned long T2 = millis();
  Serial.print("Elapsed time for 1000 float readings of accel, gyro rates and altitude: ");
  Serial.println(T2-T1);

/*  // setup Timer2 for interrupts
  setupTimer2();
  active = true; */
}

void loop() {
/*  if(intr) { 
    // DRAGONS
 } */
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

