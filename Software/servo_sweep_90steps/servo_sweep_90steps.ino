#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  myservo.attach(9, 900, 2100);  // attaches the servo on pin 9 to the servo object
								 // 900 µs is 0°, 2100 µs is 90° for D05010MG servos
} 
 
 
void loop() 
{ 
  myservo.write(90); // central position, Servo library assumes full 0-180° deflection
  delay(1000);
  myservo.write(180); // extended one side
  delay(1000);
  myservo.write(90);
  delay(1000);
  myservo.write(0); // extended other side
  delay(1000);
} 
