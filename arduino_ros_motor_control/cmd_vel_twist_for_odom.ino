#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

ros::NodeHandle nh;


// decleared for right motor
const int motor1PWM_pin = 9;  // PWM pin for Motor 1
const int motor1Dir_pin = 8;  // Direction pin for Motor 1

// declered for left motor
const int motor2PWM_pin = 10; // PWM pin for Motor 2
const int motor2Dir_pin = 11; // Direction pin for Motor 2

#define ENC_COUNT_REV 8123

// Encoder pin connect for the right motor  
#define ENC_IN_RIGHT_A 3
#define ENC_IN_RIGHT_B 5

// Encoder pin for the left motor
#define ENC_IN_LEFT_A 2
#define ENC_IN_LEFT_B 4

boolean Direction_right = true;
boolean Direction_left = true;

volatile long right_wheel_pulse_count = 0;
volatile long left_wheel_pulse_count = 0;

int interval = 1000;

long previousMillis = 0;
long currentMillis = 0;


// Variable for RPM measurement
float rpm_right = 0;
float rpm_left = 0;
  



float wheel_radius = 0.0485;

int getRpm(float linear_vel) {
    int rpm = static_cast<int>((linear_vel * 60) / (2 * PI * wheel_radius));
    rpm = constrain(rpm, -255, 255);
    return rpm;
}

// Callback function for cmd_vel subscriber
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg) {
    float linear_x = cmd_vel_msg.linear.x;
    float angular_z = cmd_vel_msg.angular.z;

    // Calculate differential drive velocities
    float left_speed = linear_x - 0.2105 * angular_z;
    float right_speed = linear_x + 0.2105 * angular_z;

    // Map the velocities to RPM values
    int left_rpm = getRpm(left_speed);
    int right_rpm = getRpm(right_speed);

    // Set motor directions
    digitalWrite(motor1Dir_pin, (right_speed >= 0) ? HIGH : LOW);
    digitalWrite(motor2Dir_pin, (left_speed >= 0) ? HIGH : LOW);

    // Set motor speeds (use the absolute value)
    analogWrite(motor1PWM_pin, abs(right_rpm));
    analogWrite(motor2PWM_pin, abs(left_rpm));
}



// ROS publisher for motor RPM
std_msgs::Float32MultiArray rpm_msg;
ros::Publisher rpm_pub("motor_rpm", &rpm_msg);


// Function to publish RPM values
void publishRPM() {
   
    // Publish RPM values to ROS
    rpm_msg.data_length = 2;
    rpm_msg.data[0] = rpm_right;
    rpm_msg.data[1] = rpm_left;
    rpm_pub.publish(&rpm_msg);

  
}


// ROS subscriber for cmd_vel
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCallback);



//// ROS publisher for motor RPM
//std_msgs::Float32MultiArray rpm_msg;
//ros::Publisher rpm_pub("motor_rpm", &rpm_msg, 10);
//float rpm_data[2]; // RPM values for both motors

void setup() {
    pinMode(motor1PWM_pin, OUTPUT);
    pinMode(motor1Dir_pin, OUTPUT);
    pinMode(motor2PWM_pin, OUTPUT);
    pinMode(motor2Dir_pin, OUTPUT);

    // Set pin states of the encoders
    pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
    pinMode(ENC_IN_RIGHT_B , INPUT);

    pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
    pinMode(ENC_IN_LEFT_B, INPUT);
 
    // Every time the pin goes high, this is a pulse
    attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_pulse, RISING);

    // Initialize the ROS node handle
    nh.initNode();

    // Subscribe to cmd_vel topic
    nh.subscribe(cmdVelSub);

    // Advertise the motor_rpm topic
    nh.advertise(rpm_pub);

    publishRPM();
}




void loop() {
   nh.spinOnce(); // Handle ROS callbacks

  // Record the time
  currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

  // Calculate revolutions per minute for right motor
  rpm_right = (float)(right_wheel_pulse_count * 60 / ENC_COUNT_REV);

  // Calculate revolutions per minute for left motor
  rpm_left = (float)(left_wheel_pulse_count * 60 / ENC_COUNT_REV);

  // Publish RPM values to ROS
  publishRPM();

  right_wheel_pulse_count = 0;

  left_wheel_pulse_count = 0; 
  }
  
}

void right_wheel_pulse() {
   
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
    right_wheel_pulse_count++;
  }
  else {
    right_wheel_pulse_count--;
  }
}



void left_wheel_pulse() {
  int val = digitalRead(ENC_IN_LEFT_B);

  if (val == LOW) {
    Direction_left = false; // Reverse
  } else {
    Direction_left = true; // Forward
  }

  if (Direction_left) {
    left_wheel_pulse_count++;
  } else {
    left_wheel_pulse_count--;
  }
}
