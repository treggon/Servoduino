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
ros::NodeHandle nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range("/ultrasound", &range_msg);

const int adc_pin = 0;

char frameid[] = "/ultrasound";

float getRange_Ultrasound() {
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
  localdistance = localdistance * 10 / 58;
  return localdistance;
}

void setup() {  
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(PWM_OUTPUT_PIN, INPUT);
  nh.initNode();
  nh.advertise(pub_range);
  
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 35.0;
  range_msg.max_range = 3500.0;
}

long range_time;

void loop() {
  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stablize
  if ( millis() >= range_time ){
    int r =0;

    distance = getRange_Ultrasound();      
    range_msg.range = distance;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_time =  millis() + 50;
  }
  
  nh.spinOnce();
}