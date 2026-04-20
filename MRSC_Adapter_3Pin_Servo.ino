/* * =======================================================================================================
 * ADVANCED MRSC (Micro RC Stability Control) - V0.4.2 (16MHz & Fast I2C Edition)
 * =======================================================================================================
 * ORIGINAL AUTHOR: TheDIYGuy999
 * ORIGINAL REPOSITORY: https://github.com/TheDIYGuy999/MRSC_Adapter_3Pin_Servo
 * * UPGRADE NOTE (V0.4.2): 
 * This version was heavily optimized for 16MHz Arduino boards and modified to include 
 * advanced safety features, dynamic cornering, and 400kHz Fast I2C communication 
 * for high-speed RC applications (like OpenRC F1).
 * * * * Hardware Requirements:
 * - MCU: Atmega 328P (5V, 16MHz) - Arduino Pro Mini / Nano / Uno
 * - IMU: MPU-6050 (GY-521 Module)
 * - Input: PWM signal from standard RC Receiver
 * * * * Advanced Features Included in V0.4.2:
 * 1. Software Gyro Inversion: No physical switch required. Change 'mpuInversed' variable to reverse gyro.
 * 2. Stick Deadband: Filters out mechanical radio spring slop, ensuring a dead-straight center tracking.
 * 3. Safe Auto-Calibration: Pre-loads safe travel limits to prevent "Division by Zero" crashes on boot.
 * 4. Atomic Reads (Anti-Jitter): Suspends interrupts momentarily to ensure glitch-free 32-bit variable reading.
 * 5. Smart Failsafe: Automatically centers the steering if the receiver signal is lost for more than 50ms.
 * 6. Slider Protection: Constrains the CH3/CH4 Gain input to strictly 0-100% to prevent math overflow.
 * 7. Fast I2C Mode: 400kHz communication speed for ultra-low latency gyro feedback.
 * 8. Heading Hold Mode: Locks the yaw angle during straight runs to prevent spin-outs under heavy acceleration.
 * * * * Pin Connections:
 * - Pin 4  -> Steering PWM Input (From Receiver CH2)
 * - Pin 5  -> Remote Gain PWM Input (From Receiver CH3 Slider / CH4 Dial)
 * - Pin A0 -> Steering Servo PWM Output
 * - Pin A4 -> MPU-6050 SDA (I2C Data)
 * - Pin A5 -> MPU-6050 SCL (I2C Clock)
 * * =======================================================================================================
 */

const float codeVersion = 0.42; // Firmware revision

// =======================================================================================================
// LIBRARIES
// =======================================================================================================

#include <Wire.h>  // I2C library for MPU-6050 communication
#include <Servo.h> // Standard servo control library
#include "mpu.h"   // Custom MPU-6050 handler (.h file must be in the same sketch folder)

// =======================================================================================================
// USER CONFIGURATION (Adjust these values to tune your vehicle)
// =======================================================================================================

// 1. GYRO DIRECTION INVERSION
// If the steering servo corrects in the wrong direction when you slide the car, change this to 'true'.
// (Default is true based on previous setup)
boolean mpuInversed = true; 

// 2. STICK DEADBAND (Radio Center Spring Filter)
// If the servo twitches or wanders when you let go of the steering wheel, increase this value.
// It ignores small inputs around the center point (Range: 0 to 5 is ideal).
int deadbandStick = 3;

// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================

// Interrupt variables for reading incoming PWM signals from the receiver
volatile uint8_t prev;               // Remembers the state of input bits from the previous interrupt
volatile uint32_t risingEdge[2];     // Timestamps of the last rising edge for each channel
volatile uint32_t uSec[2];           // The latest measured pulse width (in microseconds) for each channel

// Atomic Read variables to prevent servo jitter (glitches) during interrupt overlaps
uint32_t safe_uSec[2];
uint32_t safe_risingEdge[2];

// Servo Object Initialization
Servo servoSteering; 

// SAFE AUTO-CALIBRATION LIMITS
// These safe defaults prevent Arduino math crashes (division by zero) before the first steering input.
// Note: Always turn the steering wheel fully left and right once after powering on the car!
byte limSteeringL = 60, limSteeringR = 120; // Safe physical servo angles
int limuSecL = 1200, limuSecR = 1800;       // Safe receiver PWM boundaries

// Stability Control Parameters
byte mrscGain = 80;            // Default gain value (Overwritten dynamically by CH3 Slider)
float headingMultiplier = 2.0; // Heading hold multiplier (Keeps wheels parallel to the ground during drifts)

// Pin Definitions (Hardcoded for Port D interrupts, do not change input pins)
#define INPUT_STEERING 4
#define INPUT_GAIN 5
#define OUTPUT_STEERING A0

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
  // Activate internal pull-up resistors for PWM inputs to prevent floating signals
  pinMode(INPUT_STEERING, INPUT_PULLUP);
  pinMode(INPUT_GAIN, INPUT_PULLUP);

  // Configure Pin Change Interrupts for Port D (Pins 4 and 5)
  PCMSK2 |= B00110000; // Enable mask for pins 4 and 5 (Registers)
  PCICR |= B00000100;  // Enable interrupt for Port D

  // Initialize steering servo and set it to absolute center to prevent mechanical binding on boot
  servoSteering.attach(OUTPUT_STEERING);
  servoSteering.write(90); 

  // Initialize MPU-6050 accelerometer and gyroscope
  setupMpu6050();

  // --- FAST I2C ENABLE ---
  // Boost I2C speed to 400kHz to minimize latency between the MPU-6050 and the Arduino
  Wire.setClock(400000); 
}

// =======================================================================================================
// DYNAMIC SERVO END-POINT CALIBRATION
// =======================================================================================================

// Learns the maximum left and right travel limits to maximize steering resolution and prevent servo burnout.
void detectSteeringRange() {
  int steeringuSec = safe_uSec[0];
  
  // Expand input limits based on live receiver data (e.g., captures 1000uS and 2000uS endpoints)
  if (steeringuSec > 500 && steeringuSec < limuSecL) limuSecL = steeringuSec;
  if (steeringuSec < 2500 && steeringuSec > limuSecR) limuSecR = steeringuSec;

  // Map the new input limits to physical servo angles (typically between 45° and 135°)
  int servoAngle = map(safe_uSec[0], 1000, 2000, 45, 135);
  if (servoAngle > 20 && servoAngle < limSteeringL) limSteeringL = servoAngle;
  if (servoAngle < 160 && servoAngle > limSteeringR) limSteeringR = servoAngle;
}

// =======================================================================================================
// SMART FAILSAFE & SIGNAL VALIDITY CHECK
// =======================================================================================================

// Constantly monitors the health of the radio link. Centers the steering if connection is lost.
void checkValidity() {
  uint32_t now = micros();
  
  for (int i = 0; i <= 1; i++) {
    // 1. PWM Range Check: If signal is corrupted (below 800uS or above 2200uS), force to neutral 1500uS
    if (safe_uSec[i] < 800 || safe_uSec[i] > 2200) safe_uSec[i] = 1500;
    
    // 2. Timeout Check: If the receiver stops sending pulses for more than 50ms (Radio off / out of range)
    if (now - safe_risingEdge[i] > 50000) safe_uSec[i] = 1500;
  }
}

// =======================================================================================================
// STABILITY CONTROL CORE ALGORITHM (MRSC)
// =======================================================================================================

void mrsc() {
  int steeringAngle;
  long gyroFeedback;

  // Convert remote gain PWM signal (1000-2000uS) to a percentage (0-100%)
  // The 'constrain' function ensures that pushing the slider past 100% doesn't break the math.
  mrscGain = constrain(map(safe_uSec[1], 1000, 2000, 0, 100), 0, 100);
  
  // Fetch fresh yaw rate and angle from the IMU
  readMpu6050Data();

  // Calculate driver's intended steering position (-50 to +50 scale)
  int turnRateSetPoint = map(safe_uSec[0], limuSecL, limuSecR, -50, 50);  

  // --- STICK DEADBAND LOGIC ---
  // Eliminate radio spring centering errors. If input is close to center, snap it strictly to 0.
  if (abs(turnRateSetPoint) < deadbandStick) {
    turnRateSetPoint = 0;
  }

  int steering = abs(turnRateSetPoint); // Absolute value required for gain mathematics
  
  // Dynamic Gain: Reduce gyro intervention during tight corners to prevent fighting the driver
  int gain = map(steering, 0, 50, mrscGain, (mrscGain / 5)); 

  // --- HEADING HOLD LOGIC ---
  if (steering < 5 && mrscGain > 85) {
    // If driving straight with very high gain, lock the heading angle (Heading Hold Mode)
    gyroFeedback = yaw_angle * headingMultiplier; 
  }
  else { 
    // If cornering or using standard gain, apply standard yaw rate damping (Normal Rate Mode)
    gyroFeedback = yaw_rate * 50; 
    yaw_angle = 0; // Reset heading memory for the next straight run
  }

  // Apply gyro compensation to the driver's steering input based on the chosen inversion direction
  if (mpuInversed) steeringAngle = turnRateSetPoint + (gyroFeedback * gain / 100); 
  else steeringAngle = turnRateSetPoint - (gyroFeedback * gain / 100);

  // Constrain the final computed signal to prevent mechanical binding of the steering rack
  steeringAngle = constrain(steeringAngle, -50, 50); 
  
  // Send the stabilized signal to the physical servo
  servoSteering.write(map(steeringAngle, -50, 50, limSteeringL, limSteeringR)); 
}

// =======================================================================================================
// MAIN CONTROL LOOP
// =======================================================================================================

void loop() {
  // 1. ATOMIC READ (Anti-Jitter Protection)
  // Suspends interrupts briefly to copy 32-bit variables safely. 
  // Prevents the servo from twitching if a new PWM signal arrives exactly while reading the data.
  noInterrupts();           
  safe_uSec[0] = uSec[0];   
  safe_uSec[1] = uSec[1];   
  safe_risingEdge[0] = risingEdge[0];
  safe_risingEdge[1] = risingEdge[1];
  interrupts();             

  // 2. EXECUTE SUBSYSTEMS
  checkValidity();       // Ensure radio signal is present and valid
  detectSteeringRange(); // Continuously update and monitor servo travel limits
  mrsc();                // Calculate IMU physics and drive the servo
}
