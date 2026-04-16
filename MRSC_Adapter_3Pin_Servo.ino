/* * =======================================================================================================
 * ADVANCED MRSC (Micro RC Stability Control) - V0.4.1 (16MHz & Fast I2C Edition)
 * =======================================================================================================
 * ORIGINAL AUTHOR: TheDIYGuy999
 * ORIGINAL REPOSITORY: https://github.com/TheDIYGuy999/MRSC_Adapter_3Pin_Servo
 * * UPGRADE NOTE: Version 0.4.1 was upgraded from the original V0.3 by an AI Assistant. 
 * The code was optimized for 16MHz Arduino boards and heavily modified to include 
 * advanced safety features (Atomic Reads, Smart Failsafe), dynamic cornering, and 
 * 400kHz Fast I2C communication for high-speed RC applications.
 * * * Hardware Requirements:
 * - MCU: Atmega 328P (5V, 16MHz) - Arduino Pro Mini
 * - IMU: MPU-6050 (GY-521 Module)
 * - Input: PWM signal from RC Receiver
 * * * Advanced Features Included:
 * 1. Heading Hold Mode: Locks the yaw angle during straight runs to prevent spin-outs under heavy acceleration.
 * 2. Dynamic Cornering: Reduces gyro interference during cornering for natural steering feel.
 * 3. Remote Gain Control: Adjust gyro sensitivity on-the-fly via a spare transmitter channel (e.g., dial/pot).
 * 4. Atomic Read (Anti-Jitter): Suspends interrupts momentarily to ensure glitch-free 32-bit variable reading.
 * 5. Smart Failsafe: Automatically centers the steering if the receiver signal is lost or corrupted.
 * 6. Auto-Calibration: Learns servo end-points automatically upon the first full left/right steering input.
 * 7. Fast I2C Mode: 400kHz communication speed for ultra-low latency gyro feedback.
 * * * Pin Connections:
 * - Pin 4  -> Steering PWM Input (From Receiver CH2)
 * - Pin 5  -> Remote Gain PWM Input (From Receiver CH4)
 * - Pin A0 -> Steering Servo PWM Output
 * - Pin A2 -> Gyro Inversion Switch (Connect to GND to reverse gyro compensation direction)
 * - Pin A4 -> MPU-6050 SDA (I2C Data)
 * - Pin A5 -> MPU-6050 SCL (I2C Clock)
 * * =======================================================================================================
 */

const float codeVersion = 0.41; // Firmware revision

// =======================================================================================================
// LIBRARIES
// =======================================================================================================

#include <Wire.h>  // I2C library for MPU-6050 communication
#include <Servo.h> // Standard servo control library
#include "mpu.h"   // Custom MPU-6050 handler (.h file must be in the sketch folder)

// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================

// Interrupt variables for reading PWM signals
volatile uint8_t prev;               // Remembers the state of input bits from the previous interrupt
volatile uint32_t risingEdge[2];     // Timestamps of the last rising edge for each channel
volatile uint32_t uSec[2];           // The latest measured pulse width (in microseconds) for each channel

// Atomic Read variables to prevent servo jitter (glitches) during interrupt overlaps
uint32_t safe_uSec[2];

// Servo Object Initialization
Servo servoSteering; 

// Servo Limits (Initial value = center position = 90° / 1500uSec)
// These limits are dynamically adjusted during the first steering operation!
byte limSteeringL = 90, limSteeringR = 90; // Typical range: 45° - 135°
int limuSecL = 1500, limuSecR = 1500;      // Typical range: 1000 - 2000 uSec

// Stability Control Parameters
byte mrscGain = 80;            // Default gain value (Overwritten by Remote Gain)
float headingMultiplier = 2.0; // Heading hold multiplier (Keeps wheels parallel to the ground during drifts)

// Hardware Switch States
boolean mpuInversed = false;   // Determines if gyro compensation direction is reversed

// Pin Definitions (Hardcoded for Port D interrupts, do not change input pins)
#define INPUT_STEERING 4
#define INPUT_GAIN 5
#define OUTPUT_STEERING A0
#define INVERSE_MPU_DIRECTION A2

// =======================================================================================================
// SERVO INPUT PIN CHANGE INTERRUPT ROUTINE (PCINT)
// =======================================================================================================

// Fast interrupt routine to read incoming PWM signals without blocking the main loop
ISR(PCINT2_vect) { 
  uint32_t now = micros();
  uint8_t curr = PIND;           // Current state of Port D pins
  uint8_t changed = curr ^ prev; // Bitwise XOR to find which pin changed state
  uint8_t channel = 0;
  
  // Check Pin 4 and Pin 5 using bitmasks
  for (uint8_t mask = B00010000; mask <= B00100000 ; mask <<= 1) { 
    if (changed & mask) { 
      if (curr & mask) { 
        risingEdge[channel] = now; // Rising edge detected, start timer
      }
      else { 
        uSec[channel] = now - risingEdge[channel]; // Falling edge detected, calculate pulse width
      }
    }
    channel++;
  }
  prev = curr; // Save current state for the next interrupt
}

// =======================================================================================================
// MAIN ARDUINO SETUP (Runs once on boot)
// =======================================================================================================

void setup() {
  // Configure inversion switch with internal pull-up
  pinMode(INVERSE_MPU_DIRECTION, INPUT_PULLUP);
  
  // Activate internal pull-up resistors for PWM inputs to prevent floating signals
  pinMode(INPUT_STEERING, INPUT_PULLUP);
  pinMode(INPUT_GAIN, INPUT_PULLUP);

  // Configure Pin Change Interrupts for Port D (Pins 4 and 5)
  PCMSK2 |= B00110000; // Enable mask for pins 4 and 5
  PCICR |= B00000100;  // Enable interrupt for Port D

  // Initialize steering servo and set it to absolute center
  servoSteering.attach(OUTPUT_STEERING);
  servoSteering.write((limSteeringL + limSteeringR) / 2); 

  // Initialize MPU-6050 accelerometer and gyroscope
  setupMpu6050();

  // --- FAST I2C ENABLE ---
  // Boost I2C speed to 400kHz to minimize latency between the MPU-6050 and the Arduino
  Wire.setClock(400000); 
}

// =======================================================================================================
// DYNAMIC SERVO END-POINT CALIBRATION
// =======================================================================================================

// Learns the maximum left and right travel limits to prevent servo burnout
void detectSteeringRange() {
  int steeringuSec = safe_uSec[0];
  
  // Expand input limits based on live receiver data
  if (steeringuSec > 500 && steeringuSec < limuSecL) limuSecL = steeringuSec;
  if (steeringuSec < 2500 && steeringuSec > limuSecR) limuSecR = steeringuSec;

  // Map input limits to physical servo angles
  int servoAngle = map(safe_uSec[0], 1000, 2000, 45, 135);
  if (servoAngle > 20 && servoAngle < limSteeringL) limSteeringL = servoAngle;
  if (servoAngle < 160 && servoAngle > limSteeringR) limSteeringR = servoAngle;
}

// =======================================================================================================
// HARDWARE INPUT READER
// =======================================================================================================

void readInputs() {
  // Check if the user has grounded Pin A2 to reverse the gyro direction
  mpuInversed = !digitalRead(INVERSE_MPU_DIRECTION);
}

// =======================================================================================================
// STABILITY CONTROL CORE ALGORITHM (MRSC)
// =======================================================================================================

void mrsc() {
  int steeringAngle;
  long gyroFeedback;

  // Convert remote gain PWM signal (1000-2000uS) to a percentage (0-100%)
  mrscGain = map(safe_uSec[1], 1000, 2000, 0, 100);
  
  // Fetch fresh yaw rate and angle from the IMU
  readMpu6050Data();

  // Calculate driver's intended steering position
  int turnRateSetPoint = map(safe_uSec[0], limuSecL, limuSecR, -50, 50);
  int steering = abs(turnRateSetPoint); // Absolute value for gain mathematics
  
  // Dynamic Gain: Reduce gyro intervention during tight corners to prevent fighting the driver
  int gain = map(steering, 0, 50, mrscGain, (mrscGain / 5));

  // --- HEADING HOLD LOGIC ---
  if (steering < 5 && mrscGain > 85) {
    // If driving straight with high gain, lock the heading angle (Heading Hold Mode)
    gyroFeedback = yaw_angle * headingMultiplier;
  }
  else { 
    // If cornering, apply standard yaw rate damping (Normal Mode)
    gyroFeedback = yaw_rate * 50;
    yaw_angle = 0; // Reset heading memory for the next straight run
  }

  // Apply gyro compensation to the driver's steering input
  if (mpuInversed) steeringAngle = turnRateSetPoint + (gyroFeedback * gain / 100);
  else steeringAngle = turnRateSetPoint - (gyroFeedback * gain / 100);
  
  // Constrain the final signal to prevent mechanical binding
  steeringAngle = constrain (steeringAngle, -50, 50); 
  
  // Send the stabilized signal to the physical servo
  servoSteering.write(map(steeringAngle, -50, 50, limSteeringL, limSteeringR) );
}

// =======================================================================================================
// MAIN CONTROL LOOP
// =======================================================================================================

void loop() {
  // 1. ATOMIC READ (Anti-Jitter Protection)
  // Suspends interrupts briefly to copy 32-bit variables safely. Prevents servo twitching.
  uint32_t edge0, edge1;
  noInterrupts(); 
  safe_uSec[0] = uSec[0];
  safe_uSec[1] = uSec[1];
  edge0 = risingEdge[0];
  edge1 = risingEdge[1];
  interrupts(); 

  // 2. SMART FAILSAFE (Signal Loss Protection)
  // If the receiver disconnects or loses signal for >50ms, return steering to absolute center.
  uint32_t now = micros();
  if (safe_uSec[0] < 800 || safe_uSec[0] > 2200 || (now - edge0 > 50000)) safe_uSec[0] = 1500; // Center steering
  if (safe_uSec[1] < 800 || safe_uSec[1] > 2200 || (now - edge1 > 50000)) safe_uSec[1] = 1500; // Center gain

  // 3. EXECUTE SUBSYSTEMS
  readInputs();          // Check hardware switches (Inversion)
  detectSteeringRange(); // Continuously update servo travel limits
  mrsc();                // Calculate IMU physics and drive the servo
}
