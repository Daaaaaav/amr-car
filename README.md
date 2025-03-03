# Description
Autonomous Mobile Car Robot Arduino code for obstacle avoidance.

# Purpose
Our project focuses on creating a completely functional Autonomous Mobile Robot 
(AMR) using a plastic car chassis body, a servo motor and plastic DC motors as 
actuators. and the function of  avoiding obstacles that are at least 50 centimeters ahead 
and slowing down when there are obstacles at least 100 centimeters ahead using an 
ultrasonic sensor’s detection. 
The purpose of this project is to develop an obstacle-avoidance mobile robot using an 
ultrasonic sensor, a servo motor for directional scanning, and an L298N motor driver for 
movement control. The goal is to create an autonomous system that can navigate an 
environment while avoiding collisions and making real-time movement decisions. 

# Setup
1. Hardware Setup 
○ Connect the DC motors to the L298N motor driver and then to digital pins 4-7 
and PWM (Pulse Width Modulation used for Proportional-Integral-Derivative 
control) pins 3 and 9 of the Arduino Uno R3 Board. 
○ Attach the ultrasonic sensor to the front of the robot using the ultrasonic sensor 
stand. Then, connect it to the Arduino Uno R3 Board through digital pins 11 for 
trigger and 13 for echo. 
○ Connect the servo motor via digital pin 10 to rotate the ultrasonic sensor for 
scanning left and right. 
2. Software Implementation 
○ Initialize components and set pin configurations according to how they are in the 
hardware. 
○ Implement the ultrasonic sensor’s distance measurement function. 
○ Define movement functions (forward, backward, turn left, turn right, stop). 
○ Implement obstacle detection and path decision logic based on sensor readings.

# Demo Video
https://drive.google.com/file/d/1AWaDa7lpVBMvXybcC-mfV42yUtTpckWZ/view?usp=drive_link
