/*************************************************** 
Servo Tester, code modified by Treggon Owens from
Default adafruit code.

Making them all move at once
 ****************************************************/
//#include <ros.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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
void setup() {
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

void loop() {

  setalltomicrosec(microsec);
  domicroseclogic();
  Serial.println(microsec);
  delay(del);

}
