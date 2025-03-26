/**
 * 
 * Author: Ritesh Talreja, Made in China, Warehouse: Shenzhen, Guangdong.
 * 
 * Components: Arduino UNO, DYPA02YYWM  v1.0
 * 
 * Arduino UNO +5V    --> DYPA02YYUM Pin 1 Red
 * Arduino UNO GND    --> DYPA02YYUM Pin 2 Black
 * Arduino UNO Pin 11 --> DYPA02YYUM Pin 3
 * Arduino UNO Pin 10 --> DYPA02YYUM Pin 4
 * 
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Sonic Sensors
#define TRIGGER_PIN 9
#define PWM_OUTPUT_PIN 10

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define USMIN  450 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2450 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates
#define SERVO_MIDPOINT 1500

#define DELTA_FREQ 60
#define SERVO_TO_MOVE 1

// our servo # counter
uint8_t servonum = 0;
uint16_t microsec = SERVO_MIDPOINT;
uint16_t del = 1/DELTA_FREQ + 1;
int direction = 1;

long duration;
float distance;
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup()
{
  Serial.begin(57600);
  while (!Serial);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(PWM_OUTPUT_PIN, INPUT);

  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);

  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  for(int servonumber = 0; servonumber<16; servonumber++)
  {
    pwm.writeMicroseconds(servonumber, SERVO_MIDPOINT);
  }
  setallservomidpoint();
  //nh.initNode();
  delay(10);


  Serial.println("System Start");
}
void setallservomidpoint()
{
  microsec = SERVO_MIDPOINT;
  for(int servonumber = 0; servonumber<16; servonumber++)
  {
    pwm.writeMicroseconds(servonumber, microsec);
  }

}

void setalltomicrosec(uint16_t usec)
{
  pwm.writeMicroseconds(0, usec);
  pwm.writeMicroseconds(1, usec);
  pwm.writeMicroseconds(2, usec);
  pwm.writeMicroseconds(3, usec);
  pwm.writeMicroseconds(4, usec);
  pwm.writeMicroseconds(5, usec);
  pwm.writeMicroseconds(6, usec);
  pwm.writeMicroseconds(7, usec);     
  pwm.writeMicroseconds(8, usec);
  pwm.writeMicroseconds(9, usec);
  pwm.writeMicroseconds(10, usec);
  pwm.writeMicroseconds(11, usec);
  pwm.writeMicroseconds(12, usec);
  pwm.writeMicroseconds(13, usec);
  pwm.writeMicroseconds(14, usec);
  pwm.writeMicroseconds(15, usec);
}

void domicroseclogic()
{
  if(microsec > USMAX)
  {
    direction = -1;
    //setallservomidpoint();
  }
  else if( microsec < USMIN)
  {
    direction = 1;
  }

  if(direction>0)
  {
    microsec = microsec + ( DELTA_FREQ);
  }
  else
  {
    microsec = microsec - ( DELTA_FREQ);
  }

}
void loop()
{
  // The sensor is triggered by a falling edge of a HIGH pulse that 
  // is more than 60 microseconds in duration.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(TRIGGER_PIN, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  // If no object detected, fixed pulse width of 35ms is sent
  // by the sensor.
  pinMode(PWM_OUTPUT_PIN, INPUT);
  duration = pulseIn(PWM_OUTPUT_PIN, HIGH);
 
  // Convert the pulse width duration into a distance
  distance = duration;
  distance = distance * 10 / 58;
  
  Serial.print(distance);
  Serial.println(" mm");
  
  setalltomicrosec(microsec);
  domicroseclogic();
  Serial.println(microsec);
  delay(del);
}