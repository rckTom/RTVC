#include <Servo_dig.h>
#include <GY80.h>
#include <Wire.h>

// define constants
#define Servo_X_pin 5
#define Servo_Y_pin 6
#define Servo_X_mid 1500
#define Servo_Y_mid 1500

#define Eject_pin 12 // to connect the ejection charge to
#define LED_pin 13 // for debugging purposes on the ground

#define R_accel 0 // DRAGONS
#define R_gyro 0 // DRAGONS
#define Q_accel 0.1 // DRAGONS
#define Q_gyro 0 // DRAGONS
#define K_accel  0.92 // DRAGONS?

#define TIMESTEP 5 // timestep in multiples of 2.048 ms
#define delta_t 0.01024

#define ON_RAMP 0 // constants for states of state machine
#define IN_FLIGHT 1
#define RECOVERY 2

// objects for servos and sensors
Servo_dig servo_x, servo_y;
GY80 gy80 = GY80();

// state variables
float h = 0.0, v = 0.0, a = 0.0;
float gamma_x = 0.0, gamma_x_dot = 0.0, gamma_x_ddot = 0.0;
float phi_x = 0.0, phi_x_dot = 0.0;
float gamma_y = 0.0, gamma_y_dot = 0.0, gamma_y_ddot = 0.0;
float phi_y = 0.0, phi_y_dot = 0.0;

float h_max = 0.0;

// variable to keep track of state for state machine
int current_state = 0;

// variables for reading gyro rates and acceleration
// reading takes about 4.6 ms
// additionally reading pressure would take extra 6.1 ms
GY80_single_scaled accel;
GY80_single_scaled gyro_rates;

// variables to determine bias and variance of measurements
int n_meas = 0;
int n_x_meas = 0;
float mu_sum_accel_z = 0.0, mu_sum_gyro_x = 0.0, mu_sum_gyro_y = 0.0;
float mu_accel_z = 0.0, mu_gyro_x = 0.0, mu_gyro_y = 0.0;
float old_mu_accel_z = 0.0, old_mu_gyro_x = 0.0, old_mu_gyro_y = 0.0;
float old_accel_z = 0.0, old_gyro_x = 0.0, old_gyro_y = 0.0;
float var_sum_accel_z = 0.0, var_sum_gyro_x = 0.0, var_sum_gyro_y = 0.0;

float min_accel_z = 2.0, max_accel_z = -1.0;
float min_gyro_x = 250.0, max_gyro_x = -250.0;
float min_gyro_y = 250.0, max_gyro_y = -250.0;

// variables for handling interrupts
bool active = false; // only count if true
bool intr = false; // set to true by interrupt, act on by loop()
int ovf_ct = 0; // keep track of how often interrupt overflowed

void setup() {
  Serial.begin(115200);
  Serial.println("RTVC firmware 2018-01-05");
  Serial.println("Determining sensor bias and variance ...");
  
  // attach two servos for x and y axis
  servo_x.attach(Servo_X_pin, 900, 2100);
  servo_y.attach(Servo_Y_pin, 900, 2100);

  // define ejection charge pin as output
  pinMode(Eject_pin, OUTPUT);
  pinMode(LED_pin, OUTPUT);

  // attach sensors over I2C and set measuring ranges
  gy80.begin();
  gy80.a_set_scale(GY80_a_scale_4);
  gy80.a_set_bw(GY80_a_bw_200);
  gy80.g_set_scale(GY80_g_scale_250);

  // setup Timer2 for interrupts
  setupTimer2();
  active = true;
}

void on_ramp() {
  old_mu_accel_z = mu_accel_z;
  old_mu_gyro_x = mu_gyro_x;
  old_mu_gyro_y = mu_gyro_y;

  n_meas++;
  
  accel = gy80.a_read_scaled();
  gyro_rates = gy80.g_read_scaled();

  mu_sum_accel_z += accel.z;
  mu_accel_z = mu_sum_accel_z / n_meas;
  var_sum_accel_z += (accel.z - mu_accel_z) * (accel.z - old_mu_accel_z);

  if(abs(gyro_rates.x)>3.0) {
    n_x_meas++;
    mu_sum_gyro_x += old_mu_gyro_x;
    mu_gyro_x = mu_sum_gyro_x / n_meas;
    var_sum_gyro_x += (old_gyro_x - mu_gyro_x) * (old_gyro_x - old_mu_gyro_x);
  } else {
    mu_sum_gyro_x += gyro_rates.x;
    mu_gyro_x = mu_sum_gyro_x / n_meas;
    var_sum_gyro_x += (gyro_rates.x - mu_gyro_x) * (gyro_rates.x - old_mu_gyro_x);
    old_gyro_x = gyro_rates.x;
  }
  
  mu_sum_gyro_y += gyro_rates.y;
  mu_gyro_y = mu_sum_gyro_y / n_meas;
  var_sum_gyro_y += (gyro_rates.y - mu_gyro_y) * (gyro_rates.y - old_mu_gyro_y);

  min_accel_z = min(min_accel_z, accel.z);
  max_accel_z = max(max_accel_z, accel.z);
  min_gyro_x = min(min_gyro_x, gyro_rates.x);
  max_gyro_x = max(max_gyro_x, gyro_rates.x);
  min_gyro_y = min(min_gyro_y, gyro_rates.y);
  max_gyro_y = max(max_gyro_y, gyro_rates.y);
  
  if(n_meas >= 1000) { current_state = RECOVERY; }
  // DRAGONS

  if(accel.z > 20.0) { 
    digitalWrite(LED_pin, HIGH);
    // Serial.println("Launch detected!");
    // Serial.print("Assumed avg. acceleration at rest: ");
    // Serial.println(mu_accel_z);
    current_state = IN_FLIGHT;
    n_meas = 0;
  }
  
  intr = false;
}

void in_flight() {
  accel = gy80.a_read_scaled();
  gyro_rates = gy80.g_read_scaled();

  // predict altitude state
  h = h + delta_t * v + 0.5 * delta_t * delta_t * a;
  h_max = max(h, h_max);
  v = v + delta_t * a;
  a = (1.0 - K_accel) * a + K_accel * accel.z - mu_accel_z;

  // if launch is detected and altitude is at least 10 m, 
  // but velocity is negative, assume apogee
  // DRAGONS
  if(h>=1.0 && v<0.0) {
    current_state = RECOVERY;
  }

  n_meas++;
  if(n_meas > 300) {
    current_state = RECOVERY;
  }
  
  intr = false;
}

void recovery() {
  byte LED_on = 0;

  // eject parachute when reaching this state
  // DRAGONS digitalWrite(Eject_pin, HIGH); 
  Serial.println("Parachute ejected!");
  Serial.print("Maximum altitude: ");
  Serial.println(h_max);
  Serial.print("Current altitude: ");
  Serial.println(h);
  Serial.print("Current vertical velocity: ");
  Serial.println(v);
  delay(1000);
  digitalWrite(Eject_pin, LOW);
  digitalWrite(LED_pin, LED_on);

  // return servos to central position
  servo_x.write(Servo_X_mid);
  servo_y.write(Servo_Y_mid);

  // output or blink or whatever ...
  Serial.print("Acceleration mean: ");
  Serial.println(mu_accel_z);
  Serial.print("Acceleration variance (*1000): ");
  Serial.println(var_sum_accel_z / n_meas * 1000.0);
  Serial.print("Acceleration range: ");
  Serial.print(min_accel_z);
  Serial.print("   ");
  Serial.println(max_accel_z);
  Serial.print("Gyro rate X mean: ");
  Serial.println(mu_gyro_x);
  Serial.print("Gyro rate X variance: ");
  Serial.println(var_sum_gyro_x / n_meas);
  Serial.print("Gyro rate X range: ");
  Serial.print(min_gyro_x);
  Serial.print("   ");
  Serial.println(max_gyro_x);
  Serial.print("Gyro rate X presumably faulty measurements: ");
  Serial.println(n_x_meas);
  Serial.print("Gyro rate Y mean: ");
  Serial.println(mu_gyro_y);
  Serial.print("Gyro rate Y variance: ");
  Serial.println(var_sum_gyro_y / n_meas);
  Serial.print("Gyro rate Y range: ");
  Serial.print(min_gyro_y);
  Serial.print("   ");
  Serial.println(max_gyro_y);
  while(1) {
    // wait forever and blink LED
    delay(1000);
    LED_on = 1 - LED_on;
    digitalWrite(LED_pin, LED_on);  
  }
}

void loop() {
  if(intr) { 
    switch(current_state) {
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

