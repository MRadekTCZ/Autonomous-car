#include <Wire.h>  // Library for communication using I²C
#include <ZumoShield.h>  // Library from ZUMO manufacturer, containing functions for controlling individual robot components
#include <IRremote.h>  // Library for receiving infrared data from a remote control

int IR_pin = 6;  // Assigning pin 6 to the infrared receiver, which works with the remote control
IRrecv IRR(IR_pin);   // Creating an object for accessing functions and data related to the remote control
bool mode[4]; // Array of boolean variables, determining the robot's current mode 

#define QTR_THRESHOLD 1500 // Line sensor sensitivity threshold

// Definition of characteristic speed values
#define REVERSE_SPEED 200 // 0 stop, 400 full speed
#define TURN_SPEED 200
#define FORWARD_SPEED 200
#define REVERSE_DURATION 200 // ms
#define TURN_DURATION 300 // ms
#define MAX_SPEED 400   
#define LED_PIN 13  // The LED on the ZUMO shield is associated with pin 13
#define NUM_SENSORS 6 // Number of line sensors
ZumoMotors motors; // Creating an object for accessing functions related to motor speed
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN); // Creating an object for accessing functions related to line sensors
unsigned int sensor_values[NUM_SENSORS]; // Array to store values read by the line sensors
int lastError = 0; // Variable used for speed correction in line-following mode

void setup() {
  Serial.begin(9600);   // Allowing serial communication
  IRR.enableIRIn(); // Start receiving encoded data from the remote control. Allow interrupts.
  Serial.print("IR Ready ... @Pin");
  Serial.println(IR_pin);
  pinMode(LED_PIN, OUTPUT); // Set pin 13 (LED_PIN) as an output
  sensors.init(QTR_NO_EMITTER_PIN); // Initialize the line sensor module
  mode[0] = false; 
  mode[1] = false;
  mode[2] = false;
  mode[3] = false; // Reset the mode-related arrays. Initially, the robot is not in any mode.
}

void loop() {
if (IRR.decode()) { // The decode() function returns true when a signal from the remote control is received
    Serial.println(IRR.results.value, HEX); // Display the received value in hexadecimal on the serial monitor
    if (IRR.results.value == 0xFF6897) { // 0xFF6897 (button 0 on the remote control) toggles mode 0, which controls the LED
    mode[0] = !mode[0];
    if (mode[0] == true) digitalWrite(LED_PIN, HIGH); 
    if (mode[0] == false) digitalWrite(LED_PIN, LOW); 
    }
  
    if (IRR.results.value == 0xFF629D) {  // 0xFF629D corresponds to the CH button. Mode[2] is the line tracking mode
    motors.setSpeeds(0, 0);
    mode[2] = !mode[2];
    mode[1] = false;
    mode[3] = false;
    
    if (mode[2]) {
      // Start calibration
    digitalWrite(13, HIGH);     // Turn on the LED
    delay(1000); // Wait for a second
    for(int i = 0; i < 80; i++) {
    if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
      motors.setSpeeds(-200, 200);
    else
      motors.setSpeeds(200, -200);
      sensors.calibrate();
    delay(20);
    // Total calibration time - 1600ms
    }
    motors.setSpeeds(0, 0); // Halt the motors
    digitalWrite(13, LOW);     // Turn off the LED
    } 
    else {
    motors.setSpeeds(0, 0);}
    }
      
    if (IRR.results.value == 0xFFA25D) {   // 0xFFA25D corresponds to the CH- button. Mode[1] is the boundary line detection mode
    mode[1] = !mode[1];
    mode[2] = false;
    mode[3] = false;
    motors.setSpeeds(0, 0);
    }
    
    if (IRR.results.value == 0xFFE21D) {  // 0xFFE21D corresponds to the CH+ button. Mode[3] is manual control mode
    motors.setSpeeds(0, 0);
    mode[3] = !mode[3];
    mode[2] = false;
    mode[1] = false;
    }
  
    if ((IRR.results.value == 0xFF18E7) && mode[3]) { // 0xFF18E7 corresponds to forward movement. The && mode[3] condition ensures that keys 2, 4, 6, 8 perform actions only in mode 3
    do {
    IRR.resume(); // Reset and prepare the remote control for receiving the next signal
    IRR.results.value = 0x0; // Reset the last stored value
    delay(150); // Required delay
    if (IRR.decode()) {
    Serial.println(IRR.results.value, HEX);
    motors.setSpeeds(MAX_SPEED, MAX_SPEED);} // MAX SPEED = 400
    } while (IRR.results.value == 0xFFFFFFFF); // Holding any button cyclically sends the value FFFFFFFF
    motors.setSpeeds(0, 0); // Halt the motors
    }
  
    if ((IRR.results.value == 0xFF4AB5) && mode[3]) { // Reverse movement
    do {
    IRR.resume();
    IRR.results.value = 0x0;
    delay(150);
    if (IRR.decode()) {
    Serial.println(IRR.results.value, HEX);
    motors.setSpeeds(-MAX_SPEED, -MAX_SPEED); // Negative MAX SPEED for reverse movement
    } while (IRR.results.value == 0xFFFFFFFF);
    motors.setSpeeds(0, 0);
    }
  
    // if remote control button 4
    if ((IRR.results.value == 0xFF10EF) && mode[3]) { // Counterclockwise rotation in place
    do {
    IRR.resume();
    IRR.results.value = 0x0;
    delay(150);
    if (IRR.decode()) {
    Serial.println(IRR.results.value, HEX);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED); // Left tread rotates backward, right tread forward, TURN_SPEED = 200
    } while (IRR.results.value == 0xFFFFFFFF);
    motors.setSpeeds(0, 0); 
    }
    
    // if remote control button 6
    if ((IRR.results.value == 0xFF5AA5) && mode[3]) { // Clockwise rotation in place
    do {
    IRR.resume();
    IRR.results.value = 0x0;
    delay(150);
    if (IRR.decode()) {
    Serial.println(IRR.results.value, HEX);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);} //lewa gąsienica obraca się do przodu, prawa do tyłu, TURN_SPEED = 200
        } while (IRR.results.value == 0xFFFFFFFF);
    motors.setSpeeds(0, 0);
    }

    IRR.resume(); // Reset and prepare the remote control for receiving the next signal
}

 if (mode[1]) {
    sensors.read(sensor_values);
    
    if (sensor_values[0] > QTR_THRESHOLD) {
    // If the leftmost sensor detects the line, reverse and turn to the right
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    }
    
    else if (sensor_values[5] > QTR_THRESHOLD)  {
    // If the rightmost sensor detects the line, reverse and turn to the left
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    }
    else  {
    // Otherwise, go straight
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    }
 }

 if (mode[2]) {
    int position = sensors.readLine(sensor_values); // Detect and record the line position and thickness, read data from the line sensors
    
    int error = position - 2500; // The variable "error" stores the distance (in units) from the center of the line, with 2500 being a heuristic value. 
    // It can take both positive and negative values, depending on the direction of deviation from the line center.

    // Speed correction in relation to the distance from the line is based on a PID controller principle. The speed correction is calculated as
    // the sum of the current deviation from the center (divided by 4) and the difference between the current error and the error measured in the last program loop (multiplied by 6).
    // Values 4 and 6 are chosen heuristically.
    int speedDifference = error / 4 + 6 * (error - lastError);

    lastError = error; // Assign the value of the current error (error) to a variable (lastError) which will be used in the next program loop.

    int m1Speed = MAX_SPEED + speedDifference; // Speed correction has the opposite sign for each of the motors
    int m2Speed = MAX_SPEED - speedDifference; // Speed cannot exceed 400, in most cases, one of the motors runs at maximum speed while the other runs at less than maximum speed, resulting in a curved path.

    if (m1Speed < 0) // Lock the motors to prevent reversing - it could happen if the "error" variable takes a large value
        m1Speed = 0;
    if (m2Speed < 0)
        m2Speed = 0;
    if (m1Speed > MAX_SPEED) // Limit the maximum speed of Zumo to 400
        m1Speed = MAX_SPEED;
    if (m2Speed > MAX_SPEED)
        m2Speed = MAX_SPEED;
    motors.setSpeeds(m1Speed, m2Speed); // Set the motors to the specified rotational speed after considering the correction
 }
}
