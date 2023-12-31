
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>


ros::NodeHandle nh;

const int motor1PWM_pin = 9;  // PWM pin for Motor 1
const int motor1Dir_pin = 8;  // Direction pin for Motor 1
const int motor2PWM_pin = 10; // PWM pin for Motor 2
const int motor2Dir_pin = 11; // Direction pin for Motor 2

#define ENC_COUNT_REV 8123

// Encoder pin connect for right motor  
#define ENC_IN_RIGHT_A 3
#define ENC_IN_RIGHT_B 5

// Encoder pin for left motor
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




void motorMove(int speedMotor1, int speedMotor2) {
  int Motor1_direction = (speedMotor1 >= 0) ? HIGH : LOW;
  int motor2_direction = (speedMotor2 >= 0) ? HIGH : LOW;

  // Set motor directions
  digitalWrite(motor1Dir_pin, Motor1_direction);
  digitalWrite(motor2Dir_pin, motor2_direction);

  // Set motor speeds (use the absolute value)
  analogWrite(motor1PWM_pin, abs(speedMotor1));
  analogWrite(motor2PWM_pin, abs(speedMotor2));
}


void pwmCallback(const std_msgs::Int32MultiArray& msg) {
  
  int speedMotor1 = static_cast<int>(msg.data[0]);
  int speedMotor2 = static_cast<int>(msg.data[1]);
  motorMove(speedMotor1, speedMotor2);
  
}

// ******************************* Publisher Define ********************************************

// ROS publisher for RPM values
std_msgs::Float32MultiArray rpm_msg;
ros::Publisher rpm_pub("motor_rpm", &rpm_msg);



void publishRPM() {
  // Publish RPM values
  float rpm_threshold = 0.01;

  // Check if RPM values are below the threshold and set them to zero
  if (fabs(rpm_right) < rpm_threshold) {
    rpm_right = 0.0;
  }

  if (fabs(rpm_left) < rpm_threshold) {
    rpm_left = 0.0;
  }

  // Publish rounded RPM values
  rpm_msg.data_length = 2;
  rpm_msg.data[0] = rpm_right;
  rpm_msg.data[1] = rpm_left;
  rpm_pub.publish(&rpm_msg);

}

// ****************************** Subscriber Define ******************************************

ros::Subscriber<std_msgs::Int32MultiArray> sub("motor_pwm_control", &pwmCallback);



void setup() {
  
  pinMode(motor1PWM_pin, OUTPUT);
  pinMode(motor1Dir_pin, OUTPUT);
  pinMode(motor2PWM_pin, OUTPUT);
  pinMode(motor2Dir_pin, OUTPUT);
 
  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);

  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
 
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);

  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_pulse, RISING);
  
  nh.initNode();
  nh.subscribe(sub);

  nh.advertise(rpm_pub);

  // Publish initial RPM values
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
