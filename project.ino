#include <Servo.h>  // Include Servo library to control the servo motor
#include <NewPing.h>  // Include NewPing library for ultrasonic sensor control

// Motor Driver Control Pins
const int LeftMotorForward = 7;   // Left motor forward pin
const int LeftMotorBackward = 6;  // Left motor backward pin
const int RightMotorForward = 4;  // Right motor forward pin
const int RightMotorBackward = 5; // Right motor backward pin
const int ENA = 9;  // PWM pin for left motor speed control
const int ENB = 3;  // PWM pin for right motor speed control

// Sensor pins for Ultrasonic sensor
#define trig_pin 11  // Ultrasonic trigger pin
#define echo_pin 13  // Ultrasonic echo pin
#define maximum_distance 700  // Max sensing distance in cm

// PID Controller Gains
float Kp = 0.6;  // Proportional Gain
float Ki = 0.2;  // Integral Gain
float Kd = 0.3;  // Derivative Gain

// Speed Control Variables
volatile int leftSpeed = 0, rightSpeed = 0;  // Measured speed of left and right wheels
int targetSpeed = 255;  // Target PWM speed (max value for full speed)
int prevErrorLeft = 0, prevErrorRight = 0;  // Previous error for PID calculation
float integralLeft = 0, integralRight = 0;  // Integral term accumulation for PID

// Obstacle Detection
boolean goesForward = false;  // Track if the robot is moving forward
int distance = 100;  // Initial obstacle distance

NewPing sonar(trig_pin, echo_pin, maximum_distance); // Create NewPing object for ultrasonic sensor
Servo servo_motor; // Create servo object

void setup() {
  Serial.begin(9600); // Initialize serial communication

  // Set motor control pins as OUTPUT
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(ENA, OUTPUT);  // Enable left motor
  pinMode(ENB, OUTPUT);  // Enable right motor

  // Enable motors (Full Speed initially)
  digitalWrite(ENA, HIGH); 
  digitalWrite(ENB, HIGH);

  servo_motor.attach(10);  // Attach servo to pin 10
  servo_motor.write(115);  // Set servo to initial forward-facing position

  delay(2000);  // Wait for setup to stabilize
  distance = readPing();  // Get initial distance reading
}

void loop() {
  int distanceRight = 50;  // Placeholder for right-side distance
  int distanceLeft = 50;   // Placeholder for left-side distance
  delay(50);  // Short delay to stabilize sensor readings

  Serial.print("Distance: ");  // Print detected distance to Serial Monitor
  Serial.println(distance);

  // Obstacle Avoidance Logic
  if (distance <= 100 && distance > 50) {  
    Serial.println("Obstacle detected ahead! Slowing down.");
    moveSlow();  // Reduce speed to avoid collision
  } 
  else if (distance <= 50) {  
    Serial.println("Obstacle too close! Stopping and checking paths.");
    moveStop();  // Stop the robot
    delay(300);
    moveBackward();  // Move back slightly to get space for turning
    delay(400);
    moveStop();
    delay(300);

    // Scan both sides to determine best turn direction
    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);
    

    // Choose the best direction (greater distance)
    if (distanceRight >= distanceLeft) {
      Serial.println("Turning right.");
      turnRight();
    } else {
      Serial.println("Turning left.");
      turnLeft();
    }
    moveStop();  // Stop after turning
  } 
  else {
    Serial.println("Moving forward.");
    moveForward();  // Continue moving forward when the path is clear
  }

  distance = readPing();  // Update the distance measurement
}

int lookRight() {  
  servo_motor.write(50);  // Rotate servo to look right
  delay(200);
  int distance = readPing();  // Measure distance on the right
  Serial.print("Right Distance: ");
  Serial.println(distance);
  delay(100);
  servo_motor.write(115);  // Reset servo to center position
  return distance;
}

int lookLeft() {
  servo_motor.write(170);  // Rotate servo to look left
  delay(500);
  int distance = readPing();  // Measure distance on the left
  Serial.print("Left Distance: ");
  Serial.println(distance);
  delay(100);
  servo_motor.write(115);  // Reset servo to center position
  return distance;
}

int readPing() {
  delay(70);  // Delay for stable sensor reading
  int cm = sonar.ping_cm();  // Get distance in cm
  if (cm == 0) {  // If no reading, assume max distance
    cm = 250;
  }
  return cm;
}

void moveStop() {
  Serial.println("Stopping.");
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void moveForward() {
  if (!goesForward) {  // Prevent redundant commands
    Serial.println("Moving forward.");
    goesForward = true;
  }

  // Compute PID speed for smooth movement
  int pidLeft = computePID(targetSpeed, leftSpeed, prevErrorLeft, integralLeft);
  int pidRight = computePID(targetSpeed, rightSpeed, prevErrorRight, integralRight);

  analogWrite(ENA, constrain(pidLeft, 0, 255));  // Apply PID-adjusted speed
  analogWrite(ENB, constrain(pidRight, 0, 255));

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

void moveBackward() {
  Serial.println("Moving backward.");
  goesForward = false;

  analogWrite(ENA, targetSpeed);
  analogWrite(ENB, targetSpeed);

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);
}

void turnRight() {
  Serial.println("Turning right.");

  analogWrite(ENA, targetSpeed);
  analogWrite(ENB, targetSpeed);

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);

  delay(300);
  moveForward();
}

void turnLeft() {
  Serial.println("Turning left.");

  analogWrite(ENA, targetSpeed);
  analogWrite(ENB, targetSpeed);

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  delay(300);
  moveForward();
}

void moveSlow() {
  Serial.println("Slowing down.");
  
  digitalWrite(ENA, LOW);  // Temporarily disable motors
  digitalWrite(ENB, LOW);
  delay(200);
  digitalWrite(ENA, HIGH);  // Restore speed after delay
  digitalWrite(ENB, HIGH);
}

// PID Controller for Speed Control
int computePID(int setpoint, int measured, int &prevError, float &integral) {
  int error = setpoint - measured;  // Calculate error
  integral += error;  // Accumulate integral error
  int derivative = error - prevError;  // Compute derivative term
  prevError = error;  // Update previous error

  return (Kp * error) + (Ki * integral) + (Kd * derivative);  // PID equation
}