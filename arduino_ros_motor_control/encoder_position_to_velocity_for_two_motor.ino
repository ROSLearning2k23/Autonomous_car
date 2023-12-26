
// ********************************* For variable declearation ******************************************************

const int motor1PWM_pin = 9;  // PWM pin for Motor 1
const int motor1Dir_pin = 8;  // Direction pin for Motor 1
const int motor2PWM_pin = 10; // PWM pin for Motor 2
const int motor2Dir_pin = 11; // Direction pin for Motor 2


 
// Motor encoder output pulses per 360 degree revolution (measured manually)
#define ENC_COUNT_REV 8123
 
// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_RIGHT_B 5



// encoder pin for left motor

#define ENC_IN_LEFT_A 2
#define ENC_IN_LEFT_B 4

 
// True = Forward; False = Reverse
boolean Direction_right = true;

boolean Direction_left = true;
 
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;

// Keep track of the number of left wheel pulses
volatile long left_wheel_pulse_count = 0;
 
// One-second interval for measurements
int interval = 1000;
  
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 
// Variable for RPM measuerment
float rpm_right = 0;

float rpm_left = 0;
 
// Variable for angular velocity measurement
float ang_velocity_right = 0;
float ang_velocity_right_deg = 0;

// Variable for angular velocity measurement
float ang_velocity_left = 0;
float ang_velocity_left_deg = 0;
 
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;


// ********************************************** For Setup PIN *********************************************************
 
void setup() {
 
  // Open the serial port at 9600 bps
  Serial.begin(9600); 

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

   
}


// ********************************* For Motor PWM and direction control *********************************************


void motorMove(int speedMotor1, int speedMotor2)
{

  int Motor1_direction = (speedMotor1 >= 0) ? HIGH : LOW;
  int motor2_direction = (speedMotor2 >= 0) ? HIGH : LOW;

  // Set motor directions
  digitalWrite(motor1Dir_pin, Motor1_direction);
  digitalWrite(motor2Dir_pin, motor2_direction);

  // Set motor speeds (use the absolute value)
  analogWrite(motor1PWM_pin, abs(speedMotor1));
  analogWrite(motor2PWM_pin, abs(speedMotor2));
  
  
}

// *************************************** Loop ************************************************************
 
void loop() {
 
  // Record the time
  currentMillis = millis();

  motorMove(100,100);
 
  // If one second has passed, print the number of pulses
  if (currentMillis - previousMillis > interval) {
 
    previousMillis = currentMillis;
 
    // Calculate revolutions per minute for right motor
    rpm_right = (float)(right_wheel_pulse_count * 60 / ENC_COUNT_REV);
    ang_velocity_right = rpm_right * rpm_to_radians;   
    ang_velocity_right_deg = ang_velocity_right * rad_to_deg;
    
    // Calculate revolutions per minute for left motor
    rpm_left = (float)(left_wheel_pulse_count * 60 / ENC_COUNT_REV);
    ang_velocity_left = rpm_left * rpm_to_radians;
    ang_velocity_left_deg = ang_velocity_left * rad_to_deg;

     
    Serial.print(" Pulses: ");
    Serial.print(right_wheel_pulse_count);
    Serial.print(",");
    Serial.println(left_wheel_pulse_count);
    
    Serial.print(" Speed: ");
    Serial.print(rpm_right);
    Serial.print(",");
    Serial.print(rpm_left);
    Serial.println(" RPM");

    
    Serial.print(" Angular Velocity: ");
    Serial.print(rpm_right);
    Serial.print(",");
    Serial.print(rpm_left);
    Serial.print(" rad per second");
    Serial.print("\t");
    Serial.print(ang_velocity_right_deg);
    Serial.print(",");
    Serial.print(ang_velocity_left_deg);
    Serial.println(" deg per second");
    Serial.println();
 
    right_wheel_pulse_count = 0;

    left_wheel_pulse_count = 0;
   
  }
}


// ****************** ************** For Right encoder puls count **************************************************************

// Increment the number of pulses by 1
void right_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
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


// *************************** For left encoder motor puls count ****************************************************************


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
