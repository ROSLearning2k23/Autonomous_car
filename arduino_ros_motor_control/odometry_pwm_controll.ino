#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

ros::NodeHandle nh;

// Edit -----------------------------------------------

unsigned long last_velocity_time = 0;
bool velocity_received = false;

const unsigned long TIMEOUT_DURATION = 100;
// -----------------------------------------------------


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
double robot_wheelbase = 0.415;
int pwm_range_limit = 255;


// cmd_vel subscribe function

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg)
{

    velocity_received = true;
    last_velocity_time = millis();

    double left_wheel_velocity = cmd_vel_msg.linear.x - (cmd_vel_msg.angular.z * 0.415 / 2.0);
    double right_wheel_velocity = cmd_vel_msg.linear.x + (cmd_vel_msg.angular.z * 0.415 / 2.0);

    int left_pwm = map_pwm(left_wheel_velocity, 0.0, 0.5, 0, 255);
    int right_pwm = map_pwm(right_wheel_velocity, 0.0, 0.5, 0, 255);

    digitalWrite(motor1Dir_pin, (right_wheel_velocity >= 0) ? HIGH : LOW);
    digitalWrite(motor2Dir_pin, (left_wheel_velocity >= 0) ? HIGH : LOW);

    // Set motor speeds (use the absolute value)
    analogWrite(motor1PWM_pin, abs(right_pwm));
    analogWrite(motor2PWM_pin, abs(left_pwm));

}


double map_pwm(double x, double in_min, double in_max, double out_min, double out_max) 
{
    if (in_min == in_max) {
        return out_min;
    }
    int pwm_value = static_cast<int>((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
//    pwm_value = std::max(0, std::min(pwm_value, pwm_range_limit));
//    pwm_value = constrain(pwm_value, 0, 200);
    pwm_value = max(0, min(abs(pwm_value), pwm_range_limit));  // using abs(pwm_value) for negetive velocity acceptable
    return pwm_value;
}

// edit ----------------------------------------------------------------
// if velocity key not press continuously the motor movement will stop---- function

void stopMotorsIfTimeout() {
    unsigned long current_time = millis();
    if (velocity_received && (current_time - last_velocity_time) >= TIMEOUT_DURATION) {
        // Stop motors if no velocity message received within the timeout duration
        digitalWrite(motor1PWM_pin, 0);
        digitalWrite(motor2PWM_pin, 0);
        velocity_received = false; // Reset the flag
    }
}
// -----------------------------------------------------------------------


// ROS subscriber for cmd_vel
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCallback);

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




//// ROS publisher for motor RPM
//std_msgs::Float32MultiArray rpm_msg;
//ros::Publisher rpm_pub("motor_rpm", &rpm_msg, 10);
//float rpm_data[2]; // RPM values for both motors

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
    Direction_left = true; // Reverse
  } else {
    Direction_left = false; // Forward
  }

  if (Direction_left) {
    left_wheel_pulse_count++;
  } else {
    left_wheel_pulse_count--;
  }
}

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

    nh.getHardware()->setBaud(115200);

    // Initialize the ROS node handle
    nh.initNode();

    // Subscribe to cmd_vel topic
    nh.subscribe(cmdVelSub);

    // Advertise the motor_rpm topic
    nh.advertise(rpm_pub);

//    publishRPM();
}




void loop() {
  nh.spinOnce(); // Handle ROS callbacks
  stopMotorsIfTimeout();
  // Record the time
  currentMillis = millis();

  // rpm calculate with 1 sec interval 

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
