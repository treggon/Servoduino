// Author Treggon Owens 4/1/2025 ish
// This code is for arduino, compiled on Mega and Uno
// Tested for ROS Noetic using serial USB connection

// It does the following stuff:
// 1) Reads ultrasonic sensor
// 2) Sets servo levels on a 16 channel servo module
// 3) Allows for ROS communication

// NOTES:
// This was to allow a cliff detector (sonic) and
// servo control for an RC robot to be controlled under ROS
// the old servo controller only had 2 channels

// Ros includes
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>

// 16 channel PWM driver
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// Pin Definitions
// These are for the sonic sensors
#define TRIGGER_PIN 9
#define PWM_OUTPUT_PIN 10

// This is the pin that arms the servo module
// OE from diagram Output Enable pin.
// This is an optional pin that can be used to disable all PWM outputs when pulled low.
#define ARM_SERVO_PIN 4

// SDA = pin 4 from diagram
// SCL = pin 5 from diagram
// https://learn.adafruit.com/16-channel-pwm-servo-driver/hooking-it-up

// PWM ultrasonics calculation variables

float distance;   // distance calculated from duration
long range_time;  // used to wait 50ms between ranges

// Name of the ultrasound frame
char ultrasoundframeid[] = "/ultrasound";
char armbuttonframeid[] = "/armbutton";
char servo0frameid[] = "/servo0";

// ROS declarations and messages
ros::NodeHandle nh;
std_msgs::String button_msg;
std_msgs::UInt16 servo0_msg;
sensor_msgs::Range range_msg;
ros::Publisher pub_range(ultrasoundframeid, &range_msg);
ros::Publisher pub_armbutton(armbuttonframeid, &button_msg);
ros::Publisher pub_servo0(servo0frameid, &servo0_msg);

// Servos
// This maintains the servo's desired state
int servoDriveArray[16];

// This maintains the servo's current drive state
int servoStateArray[16];

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define USMIN 450   // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2450  // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600

#define SERVO_FREQ 60  // Analog servos run at ~50 Hz updates
#define SERVO_MIDPOINT 1500

#define DELTA_FREQ 60

// our servo # counter
uint8_t servonum = 0;
uint16_t microsec = SERVO_MIDPOINT;
uint16_t del = 1 / DELTA_FREQ + 1;
int direction = 1;

// Arms the PWM module OE pin
void setArmedPWM() {
  digitalWrite(ARM_SERVO_PIN, HIGH);
}

// Dis-Arms the PWM modle OE pin
void setDisarmedPWM() {
  digitalWrite(ARM_SERVO_PIN, LOW);
}

// this function returns the range from the ultrasound in mm
float getRange_Ultrasound() {
  long duration;  // PWM duration measured from sonic sensor
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
  float localdistance = duration;

  // Gives the distance in mm
  localdistance = localdistance * 10 / 58;

  return localdistance;
}
void setPinModes() {

  // Setup the two pins for measuring distance

  // One pin triggers the measurement
  pinMode(TRIGGER_PIN, OUTPUT);

  // Other pin is used to measure PWM timing after trigger
  pinMode(PWM_OUTPUT_PIN, INPUT);

  // Armed servo pin
  pinMode(ARM_SERVO_PIN, OUTPUT);
}

// method to setup all the ROS node information
void setNodeHandler() {
  // Setup ros node handler and publishers
  nh.initNode();
  // Sonic range
  nh.advertise(pub_range);
  // button to 'arm' the servo controller
  nh.advertise(pub_armbutton);
  // single servo's PWM
  nh.advertise(pub_servo0);
}
void setAllServosMidpoint() {

  for (int servonumber = 0; servonumber < 16; servonumber++) {
    servoDriveArray[servonumber] = SERVO_MIDPOINT;
    servoStateArray[servonumber] = SERVO_MIDPOINT;
    pwm.writeMicroseconds(servonumber, SERVO_MIDPOINT);
  }
}
void setPWMDefaults() {
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

  // disarm the PWM unit via OE
  setDisarmedPWM();
  setAllServosMidpoint();
}

void setup() {

  // Get the pins working right
  setPinModes();

  // ROS Setup
  setNodeHandler();

  // Set the Sonic Sensor's message defaults
  setDefaultRangeMessage();

  // Setup the 16 channel PWM module for I2C comms
  setPWMDefaults();
}

// this function sets up the ultrasonic's range message
// with default values and ranges from the sensor
// the sensor reads between 3cm and 4M, reliably
void setDefaultRangeMessage() {
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id = ultrasoundframeid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 35.0;
  range_msg.max_range = 3500.0;
}

// Set the range message to the current distance
void setRangeMessage() {

  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stablize
  if (millis() >= range_time) {
    // Get the range after the 50 ms
    distance = getRange_Ultrasound();
    range_msg.range = distance;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_time = millis() + 50;
  }
}

void setalltomicrosec(uint16_t usec) {

  for (int servonumber = 0; servonumber < 16; servonumber++) {
    servoDriveArray[servonumber] = microsec;
    servoStateArray[servonumber] = microsec;
    pwm.writeMicroseconds(servonumber, usec);
  }
}

bool checkservochange() {

  bool returnbool = false;

  for (int servonumber = 0; servonumber < 16; servonumber++) {
    if( servoDriveArray[servonumber] != servoStateArray[servonumber])
    {
      returnbool = true;
    }
  }
}

void domicroseclogic() {
  if (microsec > USMAX) {
    direction = -1;
    //setallservomidpoint();
  } else if (microsec < USMIN) {
    direction = 1;
  }

  if (direction > 0) {
    microsec = microsec + (DELTA_FREQ);
  } else {
    microsec = microsec - (DELTA_FREQ);
  }
}

// This handles if there were servo changes detected
bool changesdetected = false;

// This handles the arming state of the robot's servos
bool armed = false;

// Main Loop here
// Process
// 0) Arming and safety handler
// 1) See if we were told to go to new servo levels
// Do that
// 2) Check Sonic Sensors
// Do that
// 3) Arming state logic
// 4) ROS handler and spinonce

bool handlesafety()
{

}

// Main loop for program, get ranges and sets servo levels
void loop() {
  
  setalltomicrosec(microsec);
  domicroseclogic();
  setRangeMessage();
  nh.spinOnce();
}