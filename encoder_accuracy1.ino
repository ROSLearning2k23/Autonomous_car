const int motor1PWM_pin = 9;  // PWM pin for Motor 1
const int motor1Dir_pin = 8;  // Direction pin for Motor 1
const int motor2PWM_pin = 10; // PWM pin for Motor 2
const int motor2Dir_pin = 11; // Direction pin for Motor 2


#define ENCA 2 // YELLOW
#define ENCB 4 // WHITE


volatile long right_wheel_pulse_count = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
//volatile long left_wheel_pulse_count = 0;

void setup() {
  Serial.begin(9600);

  pinMode(motor1PWM_pin, OUTPUT);
  pinMode(motor1Dir_pin, OUTPUT);
  pinMode(motor2PWM_pin, OUTPUT);
  pinMode(motor2Dir_pin, OUTPUT);
  
  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

//  pinMode(ENC1A,INPUT_PULLUP);
//  pinMode(ENC1B,INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(ENC1A),readEncoder1,RISING);

}

void loop() {
  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/

  Serial.println("Start");
  motorMove(255,0);
  delay(3435); // for one revolution(360) , time calculation
  Serial.print(" Pulses For 255 positive: ");
  Serial.println(right_wheel_pulse_count);
  Serial.println("End");

  motorMove(0,0);
  delay(3000);
  Serial.print(" Pulses For 0: ");
  Serial.println(right_wheel_pulse_count);
//  

//  motorMove(-255,0);
//  delay(10000);
//  Serial.print(" Pulses For 255 negative : ");
//  Serial.println(right_wheel_pulse_count);

  
//  motorMove(0,0);
//  delay(2000);
//  Serial.print(" Pulses For 0: ");
//  Serial.println(right_wheel_pulse_count);
  

  
//  Serial.print(",");
//  Serial.println(left_wheel_pulse_count);
  

}


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


void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    right_wheel_pulse_count++;
  }
  else{
    right_wheel_pulse_count--;
  }
}

//void readEncoder1(){
//
//  int b1 = digitalRead(ENC1B);
//  if(b1 > 0){
//    left_wheel_pulse_count++;
//  }
//  else{
//    left_wheel_pulse_count--;
//  }
//}
