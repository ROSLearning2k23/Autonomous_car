#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

ros::NodeHandle nh;

const int motor1PWM_pin = 9;  // PWM pin for Motor 1
const int motor1Dir_pin = 8;  // Direction pin for Motor 1
const int motor2PWM_pin = 10; // PWM pin for Motor 2
const int motor2Dir_pin = 11; // Direction pin for Motor 2

//float wheel_radius = 0.047;
float wheel_radius = 0.032;


int getRpm(float linear_vel)
{

    int rpm = static_cast<int>((linear_vel * 60) / (2 * PI * wheel_radius));
//    rpm = std::min(255, std::max(-255, rpm));
//    rpm = min(255, max(-255, rpm));
    rpm = constrain(rpm, -255, 255);
    return rpm;
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg) {
  float linear_x = cmd_vel_msg.linear.x;
  float angular_z = cmd_vel_msg.angular.z;

  // Calculate differential drive velocities
  float left_speed = linear_x - 0.165 * angular_z;
  float right_speed = linear_x + 0.165 * angular_z;

  // Map the velocities to PWM values (adjust as needed)
//  int left_pwm = static_cast<int>(left_speed * 255);
//  int right_pwm = static_cast<int>(right_speed * 255);

   int left_pwm = getRpm(left_speed);
   int right_pwm = getRpm(right_speed);

  // Set motor directions
  digitalWrite(motor1Dir_pin, (left_speed >= 0) ? HIGH : LOW);
  digitalWrite(motor2Dir_pin, (right_speed >= 0) ? HIGH : LOW);

  // Set motor speeds (use the absolute value)
  analogWrite(motor1PWM_pin, abs(left_pwm));
  analogWrite(motor2PWM_pin, abs(right_pwm));
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCallback);

void setup() {
  pinMode(motor1PWM_pin, OUTPUT);
  pinMode(motor1Dir_pin, OUTPUT);
  pinMode(motor2PWM_pin, OUTPUT);
  pinMode(motor2Dir_pin, OUTPUT);

  nh.initNode();
  nh.subscribe(cmdVelSub);
}

void loop() {
  nh.spinOnce();
  // Add your loop code here
}
