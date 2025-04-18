#include <ArduinoHardware.h>
#include <ArduinoTcpHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
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

#define TRIGGER_PIN 9
#define PWM_OUTPUT_PIN 10

long duration;
float distance;
ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

const int adc_pin = 0;

char frameid[] = "/ultrasound";

float getRange_Ultrasound(int pin_num){
  int val = 0;
  for(int i=0; i<4; i++) val += analogRead(pin_num);
  float range =  val;
  return range /322.519685;   // (0.0124023437 /4) ; //cvt to meters
}

void setup()
{
  Serial.begin(57600);
  while (!Serial);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(PWM_OUTPUT_PIN, INPUT);
  Serial.println("System Start");
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
  
  delay(250);
}