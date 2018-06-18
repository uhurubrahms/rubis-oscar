#include <ros.h>                // header files sourced from  Step 3
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

const int esc_pin = 5;
const int servo_pin = 6;

// These values were cacluated for the specific Teensy microcontroller
const int pwm_center_value = 9830;  //  15% duty cycle - corresponds to zero velocity, zero steering
const int pwm_lowerlimit = 6554;    //  10% duty cycle - corresponds to max reverse velocity, extreme left steering
const int pwm_upperlimit = 13108;   //  20% duty cycle - corresponds to max forward velocity, extreme right steering

std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg); 

// Arduino 'map' funtion for floating point
double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void driveCallback (const geometry_msgs::Twist&  twistMsg)
{
  int pwm_vel;
  int pwm_angle;

  // ESC forward is between 0.5 and 1.0
  if (twistMsg.linear.x >= 0.5) {
    pwm_vel = fmap(twistMsg.linear.x, 0.5, 1.0, pwm_center_value, pwm_upperlimit);
  }
  else {
    pwm_vel = fmap(twistMsg.linear.x, 0.0, 1.0, pwm_lowerlimit, pwm_upperlimit);
  }
  // Check to make sure pwm_vel is within bounds
  if (pwm_vel < pwm_lowerlimit) {
    pwm_vel = pwm_lowerlimit;
  }
  if (pwm_vel > pwm_upperlimit) {
    pwm_vel = pwm_upperlimit ;
  }
  analogWrite(esc_pin, pwm_vel);

  // The following could be useful for debugging
  // str_msg.data= pwm_vel;
  // chatter.publish(&str_msg);

  // Servo mid-point is 0.5
  pwm_angle = fmap(twistMsg.angular.z, 0.0, 1.0, pwm_lowerlimit, pwm_upperlimit);
  // Check to make sure pwm_angle is within bounds
  if (pwm_angle < pwm_lowerlimit) {
    pwm_angle = pwm_lowerlimit;
  }
  if (pwm_angle > pwm_upperlimit) {
    pwm_angle = pwm_upperlimit ;
  }
  analogWrite(servo_pin, pwm_angle);

  // The following could be useful for debugging
  // str_msg.data= pwm_angle;
  // chatter.publish(&str_msg);
}

ros::Subscriber<geometry_msgs::Twist> driveSubscriber("cmd_vel", &driveCallback) ;

void setup() {
  // Setup the PWM registers
  analogWriteFrequency(esc_pin, 100); // Frequency at which PWM signals is generated
  analogWriteFrequency(servo_pin, 100);
  analogWriteResolution(16); // Resolution for the PWM signal
  analogWrite(esc_pin, pwm_center_value); // Setup zero velocity
  analogWrite(servo_pin, pwm_center_value); // Setup zero steering

  pinMode(13, OUTPUT); // Teensy's onboard LED pin
  digitalWrite(13, HIGH); // Setup LED

  nh.initNode();  // Intialize ROS node
  // Subscribe to the steering and throttle messages
  nh.subscribe(driveSubscriber);
  // Useful for debugging purposes
  nodeHandle.advertise(chatter);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
