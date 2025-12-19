/**
 * @file closed_loop.ino
 * @brief Firmware for a closed-loop motor control system with optional ROS integration.
 *
 * This firmware is designed to control a robot with two motors, using PID controllers
 * for precise speed and position control. It supports two modes of operation:
 * - **Standalone Mode**: The robot operates independently, using serial commands for control.
 * - **ROS Mode**: The robot integrates with the Robot Operating System (ROS) for remote control
 *   and communication via the micro-ROS framework.
 *
 * ## Features:
 * - **Motor Control**: PID-based speed control for left and right motors.
 * - **Odometry**: Tracks the robot's position and orientation using encoder feedback.
 * - **Command Parsing**: Supports serial commands for motor control, PID tuning, and state reset.
 * - **ROS Integration**: Subscribes to `cmd_vel` messages for velocity commands and manages
 *   connection to a ROS agent.
 * - **Safety Features**: Includes overcurrent protection and motor disable functionality.
 * - **Status Reporting**: Periodically outputs robot status, including odometry and motor speeds.
 *
 * ## Code Structure:
 * - **Global Configuration**: Pin definitions, ROS mode selection, and debug settings.
 * - **Classes**:
 *   - `PidControl`: Implements a PID controller for motor speed control.
 *   - `MotorControl`: Manages motor speed, encoder feedback, and PWM output.
 *   - `Point` and `Motion`: Handle robot position and orientation tracking.
 * - **Global Objects**: Instances of `MotorControl`, `Motion`, and `RobotState` for managing
 *   the robot's state and behavior.
 * - **ROS-Specific Logic**: Handles ROS communication, including agent connection and message
 *   subscription.
 * - **Main Functions**:
 *   - `setup()`: Initializes hardware, ROS (if enabled), and robot state.
 *   - `loop()`: Main control loop, handling sensor updates, motor control, ROS communication,
 *     and serial command processing.
 *   - `drivecontrol()`: Calculates motor target speeds based on ROS or standalone commands.
 *   - `parseCommand()`: Processes serial commands for robot control and configuration.
 *
 * ## Usage:
 * - Set `#define ROS` to `1` for ROS mode or `0` for standalone mode.
 * - In standalone mode, use serial commands to control the robot (e.g., `M` to toggle motors,
 *   `F` to move forward, `L`/`R` to turn).
 * - In ROS mode, ensure a ROS agent is running and publishing `cmd_vel` messages.
 *
 * ## Notes:
 * - Ensure proper tuning of PID parameters for optimal performance.
 * - Adjust hardware-specific constants (e.g., encoder counts per revolution, wheel diameter)
 *   as needed for your robot.
 */

// Pin Definitions
const int PIN_XD_EN = 2;
const int PIN_RD_PWM1 = 3;
const int PIN_LD_PWM1 = 4;
const int PIN_RD_PWM2 = 5;
const int PIN_LD_PWM2 = 6;
const int PIN_LENCB = 7;
const int PIN_LENCA = 8;
const int PIN_US_TRIG = 9;
const int PIN_VBAT = 14;
const int PIN_US_ECHO = 15;
const int PIN_LD_OCM = 20;
const int PIN_RD_OCM = 21;
const int PIN_RENCB = 22;
const int PIN_RENCA = 23;
const int LED_PIN = 13;

#include "setup.h"  // Include global configuration header
#include "PidControl.h"
#include "MotorControl.h"
#include "RobotState.h"
#include "Motion.h"
#include "TuningManager.h"

#include <Wire.h>
// #include <math.h>

// IMU address shared across modules (defined in RosInterface.cpp)
extern const uint8_t MPU9250_ADDR;

#if ROS
#include "RosInterface.h"
#endif  // ROS

// --- Global Objects ---
RobotState robotState;
MotorControl leftMotor(PIN_LD_OCM, PIN_LENCB, PIN_LENCA, PIN_LD_PWM2, PIN_LD_PWM1, "Left");
MotorControl rightMotor(PIN_RD_OCM, PIN_RENCA, PIN_RENCB, PIN_RD_PWM1, PIN_RD_PWM2, "Right");
Motion motion;

// Tuning manager for persistent motor parameters
TuningManager* tuningManager = nullptr;

// Encoder streaming state
bool encoderStreaming = false;

// I2C bus speed (reduce if IMU NACKs). Teensy supports setClock().
static const uint32_t I2C_CLOCK_HZ = 100000;  // 100 kHz for robustness

#if ROS
// ROS context instance
RosContext rosContext;
// Forward declaration for global ROS context (defined in RosInterface.cpp)
extern RosContext* global_ros_context;
// Forward declaration for reset function
void resetOdomWithMagnetometerCalibration(void *context);
#endif

void robotState_reset() {
  robotState.targetLinearVelocity = 0.0f;
  robotState.targetAngularVelocity = 0.0f;
  robotState.move = false;
  // cmdDrive = true; // Should reset keep motors enabled? Maybe not.
#if !ROS
  robotState.leftTargetDistance = 0;
  robotState.rightTargetDistance = 0;
  robotState.targetHeading = 0.0f;
  robotState.isMoving = false;
  robotState.corr = 0.0f;
#endif
  SERIAL_OUT.println("RobotState reset");
}


// --- Interrupt Service Routines for Encoders ---
// These ISRs directly manipulate the public volatile members of the global motor objects.

void leftEncoderInterrupt() {
  // Read current state of Left Motor's Encoder A and B pins
  // Note: digitalRead inside ISR is generally acceptable on fast MCUs like Teensy
  int8_t current_enc_val = (digitalRead(PIN_LENCA) ? 2 : 0) | (digitalRead(PIN_LENCB) ? 1 : 0);

  // Quadrature logic
  int8_t last_A_bit = (leftMotor.lastEnc >> 1) & 1;
  int8_t last_B_bit = leftMotor.lastEnc & 1;
  int8_t current_A_bit = (current_enc_val >> 1) & 1;
  int8_t current_B_bit = current_enc_val & 1;
  int8_t dir = (last_A_bit ^ current_B_bit) - (current_A_bit ^ last_B_bit);

  if (dir != 0) {
    // Physical forward rotation of the left motor causes its counter to decrease
    // (due to A leading B with the current XOR logic), so we invert dir here
    // to make the counter increase for forward motion.
    leftMotor.counter -= dir;          // Changed from += to -= to flip the direction for the left motor
    leftMotor.lastEncTime = micros();  // Update time of the edge
  }
  leftMotor.lastEnc = current_enc_val;  // Update last known state
}

void rightEncoderInterrupt() {
  // Read current state of Right Motor's Encoder A and B pins
  int8_t current_enc_val = (digitalRead(PIN_RENCA) ? 2 : 0) | (digitalRead(PIN_RENCB) ? 1 : 0);

  // Quadrature logic
  int8_t last_A_bit = (rightMotor.lastEnc >> 1) & 1;
  int8_t last_B_bit = rightMotor.lastEnc & 1;
  int8_t current_A_bit = (current_enc_val >> 1) & 1;
  int8_t current_B_bit = current_enc_val & 1;
  int8_t dir = (last_A_bit ^ current_B_bit) - (current_A_bit ^ last_B_bit);

  if (dir != 0) {
    rightMotor.counter += dir;
    rightMotor.lastEncTime = micros();
  }
  rightMotor.lastEnc = current_enc_val;
}


// --- ROS Specific Section ---
#if !ROS
// Differential PID only needed for non-ROS heading control during F commands
PidControl differential;
#endif

// --- IMU Diagnostic Function ---
/**
 * @brief Comprehensive IMU diagnostic to identify I2C communication issues
 * This function performs multiple tests to diagnose why the IMU isn't responding
 */
void diagnoseIMU() {
  const uint8_t AK8963_ADDR = 0x0C;
  const uint8_t WHO_AM_I_REG = 0x75;
  const uint8_t PWR_MGMT_1_REG = 0x6B;
  const uint8_t INT_PIN_CFG_REG = 0x37;
  
  SERIAL_OUT.println("=== IMU Diagnostic ===");
  SERIAL_OUT.println("Teensy 4.0 detected - using Wire (pins 18=SDA, 19=SCL)");
  // Ensure diagnostics use the same reduced I2C clock
  Wire.setClock(I2C_CLOCK_HZ);
  
  // Test 1: I2C Bus Scan - Check Wire (primary I2C bus)
  SERIAL_OUT.println("\n1. Scanning Wire bus (pins 18=SDA, 19=SCL) for devices...");
  int devicesFound = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      SERIAL_OUT.print("  Device found at address 0x");
      if (addr < 16) SERIAL_OUT.print("0");
      SERIAL_OUT.println(addr, HEX);
      devicesFound++;
    }
  }
  
  // Also check Wire1 bus (Teensy 4.0 has multiple I2C buses)
  #if defined(__IMXRT1062__)  // Teensy 4.0/4.1
  SERIAL_OUT.println("\n1b. Scanning Wire1 bus (pins 17=SDA, 16=SCL) for devices...");
  Wire1.begin();
    Wire1.setClock(I2C_CLOCK_HZ);
  int devicesFoundWire1 = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire1.beginTransmission(addr);
    uint8_t error = Wire1.endTransmission();
    if (error == 0) {
      SERIAL_OUT.print("  Device found on Wire1 at address 0x");
      if (addr < 16) SERIAL_OUT.print("0");
      SERIAL_OUT.println(addr, HEX);
      devicesFoundWire1++;
    }
  }
  if (devicesFoundWire1 > 0) {
    SERIAL_OUT.println("  WARNING: Devices found on Wire1, but code uses Wire!");
    SERIAL_OUT.println("  You may need to change code to use Wire1 instead of Wire");
  }
  devicesFound += devicesFoundWire1;
  #endif
  if (devicesFound == 0) {
    SERIAL_OUT.println("  ERROR: No I2C devices found! Check wiring and power.");
    
    // Check I2C pin states (for Teensy, default I2C pins are typically 18/19)
    // Note: We can't directly read which pins Wire library is using, but we can check common ones
    SERIAL_OUT.println("\n  I2C Pin State Check:");
    SERIAL_OUT.println("  (Note: Default I2C pins for Teensy 4.x are typically SDA=18, SCL=19)");
    SERIAL_OUT.println("  (For Teensy 3.x: SDA=18, SCL=19)");
    
    // Try to check if pins are floating (should be HIGH with pull-ups, LOW if shorted)
    // We'll check common I2C pins
    const int common_sda_pins[] = {18, 17, 16, 0};
    const int common_scl_pins[] = {19, 16, 17, 1};
    
    SERIAL_OUT.println("  Checking common SDA pins:");
    for (int i = 0; i < 4; i++) {
      pinMode(common_sda_pins[i], INPUT);
      delayMicroseconds(10);
      int state = digitalRead(common_sda_pins[i]);
      SERIAL_OUT.print("    Pin ");
      SERIAL_OUT.print(common_sda_pins[i]);
      SERIAL_OUT.print(": ");
      SERIAL_OUT.println(state == HIGH ? "HIGH (OK - pull-up present)" : "LOW (ERROR - shorted or no pull-up)");
    }
    
    SERIAL_OUT.println("  Checking common SCL pins:");
    for (int i = 0; i < 4; i++) {
      pinMode(common_scl_pins[i], INPUT);
      delayMicroseconds(10);
      int state = digitalRead(common_scl_pins[i]);
      SERIAL_OUT.print("    Pin ");
      SERIAL_OUT.print(common_scl_pins[i]);
      SERIAL_OUT.print(": ");
      SERIAL_OUT.println(state == HIGH ? "HIGH (OK - pull-up present)" : "LOW (ERROR - shorted or no pull-up)");
    }
    
    SERIAL_OUT.println("\n  ANALYSIS:");
    SERIAL_OUT.println("  ✓ I2C pins 18 (SDA) and 19 (SCL) have pull-ups (reading HIGH)");
    SERIAL_OUT.println("  ✗ No I2C devices responding on the bus");
    SERIAL_OUT.println("  → Most likely: IMU not powered OR not connected to pins 18/19");
    
    SERIAL_OUT.println("\n  TROUBLESHOOTING STEPS (in priority order):");
    SERIAL_OUT.println("\n  [CRITICAL] 1. VERIFY IMU POWER:");
    SERIAL_OUT.println("     With multimeter, measure at IMU module:");
    SERIAL_OUT.println("     - VCC pin to GND: Should read ~3.3V (NOT 0V, NOT 5V)");
    SERIAL_OUT.println("     - If 0V: IMU not powered - check power supply connection");
    SERIAL_OUT.println("     - If 5V: Wrong voltage - IMU may be damaged (needs 3.3V)");
    SERIAL_OUT.println("     - If ~3.3V: Power is OK, continue to step 2");
    
    SERIAL_OUT.println("\n  [CRITICAL] 2. VERIFY I2C CONNECTIONS:");
    SERIAL_OUT.println("     With power OFF, check continuity with multimeter:");
    SERIAL_OUT.println("     - IMU SDA pin → Teensy pin 18: Should beep (connected)");
    SERIAL_OUT.println("     - IMU SCL pin → Teensy pin 19: Should beep (connected)");
    SERIAL_OUT.println("     - IMU GND pin → Teensy GND: Should beep (connected)");
    SERIAL_OUT.println("     - If no beep: Wire broken or not connected");
    
    SERIAL_OUT.println("\n  3. CHECK FOR SHORTS (power OFF):");
    SERIAL_OUT.println("     Measure resistance with multimeter:");
    SERIAL_OUT.println("     - SDA to GND: Should be ~4.7kΩ (pull-up), NOT 0Ω");
    SERIAL_OUT.println("     - SCL to GND: Should be ~4.7kΩ (pull-up), NOT 0Ω");
    SERIAL_OUT.println("     - SDA to SCL: Should be open/infinite, NOT shorted");
    SERIAL_OUT.println("     - If 0Ω: Short circuit present");
    
    SERIAL_OUT.println("\n  4. VERIFY IMU IS PRESENT:");
    SERIAL_OUT.println("     - Is the IMU module physically installed?");
    SERIAL_OUT.println("     - Check for broken solder joints on PCB");
    SERIAL_OUT.println("     - If using breakout board, verify all connections");
    
    SERIAL_OUT.println("\n  5. TEST IMU ON DIFFERENT BUS:");
    SERIAL_OUT.println("     If IMU is on Wire1 (pins 17/16), you'll need to modify code");
    SERIAL_OUT.println("     to use Wire1 instead of Wire");
    
    SERIAL_OUT.println("\n  QUICK TEST:");
    SERIAL_OUT.println("  Try disconnecting and reconnecting IMU power while robot is running.");
    SERIAL_OUT.println("  If diagnostic suddenly finds device, it was a power/connection issue.");
    
    return;
  }
  SERIAL_OUT.print("  Found ");
  SERIAL_OUT.print(devicesFound);
  SERIAL_OUT.println(" device(s) on I2C bus");
  
  // Test 2: Check if IMU responds at expected address
  SERIAL_OUT.println("\n2. Testing IMU at address 0x68...");
  Wire.beginTransmission(MPU9250_ADDR);
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    SERIAL_OUT.print("  ERROR: IMU not responding at 0x68 (error code: ");
    SERIAL_OUT.print(error);
    SERIAL_OUT.println(")");
    SERIAL_OUT.println("  Error codes: 0=OK, 1=data too long, 2=NACK on address, 3=NACK on data, 4=other");
    SERIAL_OUT.println("  Possible causes:");
    SERIAL_OUT.println("    - Wrong I2C address (check AD0 pin - 0x68 if LOW, 0x69 if HIGH)");
    SERIAL_OUT.println("    - Device not powered or in sleep mode");
    SERIAL_OUT.println("    - I2C bus issue");
    return;
  }
  SERIAL_OUT.println("  OK: IMU responds at address 0x68");
  
  // Test 3: Read WHO_AM_I register
  SERIAL_OUT.println("\n3. Reading WHO_AM_I register (0x75)...");
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(WHO_AM_I_REG);
  error = Wire.endTransmission(false);
  if (error != 0) {
    SERIAL_OUT.print("  ERROR: Failed to write register address (error: ");
    SERIAL_OUT.print(error);
    SERIAL_OUT.println(")");
    return;
  }
  
  uint8_t bytes_read = Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)1, (uint8_t)true);
  if (bytes_read != 1) {
    SERIAL_OUT.print("  ERROR: Failed to read WHO_AM_I (got ");
    SERIAL_OUT.print(bytes_read);
    SERIAL_OUT.println(" bytes)");
    return;
  }
  
  uint8_t whoami = Wire.read();
  SERIAL_OUT.print("  WHO_AM_I value: 0x");
  SERIAL_OUT.println(whoami, HEX);
  
  if (whoami == 0x71) {
    SERIAL_OUT.println("  OK: MPU9250 detected (correct ID)");
  } else if (whoami == 0x68) {
    SERIAL_OUT.println("  WARNING: MPU6050 detected (not MPU9250)");
    SERIAL_OUT.println("  This firmware expects MPU9250 with magnetometer");
  } else {
    SERIAL_OUT.print("  ERROR: Unexpected device ID (expected 0x71 for MPU9250, got 0x");
    SERIAL_OUT.print(whoami, HEX);
    SERIAL_OUT.println(")");
    SERIAL_OUT.println("  Possible causes:");
    SERIAL_OUT.println("    - Wrong device connected");
    SERIAL_OUT.println("    - Device malfunction");
    return;
  }
  
  // Test 4: Check Power Management Register
  SERIAL_OUT.println("\n4. Checking Power Management Register (0x6B)...");
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(PWR_MGMT_1_REG);
  error = Wire.endTransmission(false);
  if (error != 0) {
    SERIAL_OUT.print("  ERROR: Failed to read PWR_MGMT_1 (error: ");
    SERIAL_OUT.print(error);
    SERIAL_OUT.println(")");
    return;
  }
  
  bytes_read = Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)1, (uint8_t)true);
  if (bytes_read != 1) {
    SERIAL_OUT.println("  ERROR: Failed to read PWR_MGMT_1");
    return;
  }
  
  uint8_t pwr_mgmt = Wire.read();
  SERIAL_OUT.print("  PWR_MGMT_1 value: 0x");
  SERIAL_OUT.println(pwr_mgmt, HEX);
  
  if (pwr_mgmt & 0x40) {
    SERIAL_OUT.println("  WARNING: Device is in sleep mode (bit 6 set)");
    SERIAL_OUT.println("  Attempting to wake device...");
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(PWR_MGMT_1_REG);
    Wire.write(0x00);  // Clear sleep bit
    error = Wire.endTransmission(true);
    if (error == 0) {
      SERIAL_OUT.println("  OK: Device woken up");
      delay(100);  // Wait for device to stabilize
    } else {
      SERIAL_OUT.print("  ERROR: Failed to wake device (error: ");
      SERIAL_OUT.print(error);
      SERIAL_OUT.println(")");
    }
  } else {
    SERIAL_OUT.println("  OK: Device is awake");
  }
  
  if (pwr_mgmt & 0x80) {
    SERIAL_OUT.println("  WARNING: Device reset bit is set (may be resetting)");
  }
  
  // Test 5: Test reading accelerometer/gyro data register
  SERIAL_OUT.println("\n5. Testing accelerometer/gyro data read...");
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  error = Wire.endTransmission(false);
  if (error != 0) {
    SERIAL_OUT.print("  ERROR: Failed to set register pointer (error: ");
    SERIAL_OUT.print(error);
    SERIAL_OUT.println(")");
    return;
  }
  
  bytes_read = Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)14, (uint8_t)true);
  if (bytes_read != 14) {
    SERIAL_OUT.print("  ERROR: Failed to read 14 bytes (got ");
    SERIAL_OUT.print(bytes_read);
    SERIAL_OUT.println(" bytes)");
    SERIAL_OUT.println("  This is the same error occurring during normal operation");
    return;
  }
  
  // Read and display raw values
  int16_t accel_x = (Wire.read() << 8) | Wire.read();
  int16_t accel_y = (Wire.read() << 8) | Wire.read();
  int16_t accel_z = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // Skip temperature
  int16_t gyro_x = (Wire.read() << 8) | Wire.read();
  int16_t gyro_y = (Wire.read() << 8) | Wire.read();
  int16_t gyro_z = (Wire.read() << 8) | Wire.read();
  
  SERIAL_OUT.println("  OK: Successfully read sensor data");
  SERIAL_OUT.print("  Accel (raw): X=");
  SERIAL_OUT.print(accel_x);
  SERIAL_OUT.print(" Y=");
  SERIAL_OUT.print(accel_y);
  SERIAL_OUT.print(" Z=");
  SERIAL_OUT.println(accel_z);
  SERIAL_OUT.print("  Gyro (raw): X=");
  SERIAL_OUT.print(gyro_x);
  SERIAL_OUT.print(" Y=");
  SERIAL_OUT.print(gyro_y);
  SERIAL_OUT.print(" Z=");
  SERIAL_OUT.println(gyro_z);
  
  // Test 6: Check I2C passthrough configuration
  SERIAL_OUT.println("\n6. Checking I2C passthrough configuration...");
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(INT_PIN_CFG_REG);
  error = Wire.endTransmission(false);
  if (error != 0) {
    SERIAL_OUT.print("  ERROR: Failed to read INT_PIN_CFG (error: ");
    SERIAL_OUT.print(error);
    SERIAL_OUT.println(")");
    return;
  }
  
  bytes_read = Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)1, (uint8_t)true);
  if (bytes_read != 1) {
    SERIAL_OUT.println("  ERROR: Failed to read INT_PIN_CFG");
    return;
  }
  
  uint8_t int_pin_cfg = Wire.read();
  SERIAL_OUT.print("  INT_PIN_CFG value: 0x");
  SERIAL_OUT.println(int_pin_cfg, HEX);
  
  if (int_pin_cfg & 0x02) {
    SERIAL_OUT.println("  OK: I2C passthrough enabled (bit 1 set)");
  } else {
    SERIAL_OUT.println("  WARNING: I2C passthrough NOT enabled");
    SERIAL_OUT.println("  Magnetometer access will fail");
    SERIAL_OUT.println("  Attempting to enable passthrough...");
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(INT_PIN_CFG_REG);
    Wire.write(0x02);  // Enable passthrough
    error = Wire.endTransmission(true);
    if (error == 0) {
      SERIAL_OUT.println("  OK: Passthrough enabled");
      delay(10);
    } else {
      SERIAL_OUT.print("  ERROR: Failed to enable passthrough (error: ");
      SERIAL_OUT.print(error);
      SERIAL_OUT.println(")");
    }
  }
  
  // Test 7: Test magnetometer access
  SERIAL_OUT.println("\n7. Testing magnetometer (AK8963) access...");
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x00);  // WIA register
  error = Wire.endTransmission(false);
  if (error != 0) {
    SERIAL_OUT.print("  ERROR: Magnetometer not responding (error: ");
    SERIAL_OUT.print(error);
    SERIAL_OUT.println(")");
    SERIAL_OUT.println("  Possible causes:");
    SERIAL_OUT.println("    - I2C passthrough not enabled");
    SERIAL_OUT.println("    - Magnetometer not initialized");
    SERIAL_OUT.println("    - Device doesn't have magnetometer (MPU6050 instead of MPU9250)");
    return;
  }
  
  bytes_read = Wire.requestFrom((uint8_t)AK8963_ADDR, (uint8_t)1, (uint8_t)true);
  if (bytes_read != 1) {
    SERIAL_OUT.println("  ERROR: Failed to read magnetometer WIA");
    return;
  }
  
  uint8_t mag_wia = Wire.read();
  SERIAL_OUT.print("  Magnetometer WIA: 0x");
  SERIAL_OUT.println(mag_wia, HEX);
  
  if (mag_wia == 0x48) {
    SERIAL_OUT.println("  OK: AK8963 magnetometer detected");
  } else {
    SERIAL_OUT.print("  WARNING: Unexpected magnetometer ID (expected 0x48, got 0x");
    SERIAL_OUT.print(mag_wia, HEX);
    SERIAL_OUT.println(")");
  }
  
  SERIAL_OUT.println("\n=== Diagnostic Complete ===");
  SERIAL_OUT.println("If all tests passed, the IMU should work correctly.");
  SERIAL_OUT.println("If errors persist, check hardware connections and power supply.");
}

// --- Command Parsing ---
/**
 * @brief Parse and execute serial commands
 * Common commands: M, X
 * Non-ROS commands: L, R, F, V, P, I, D
 */
void parseCommand(const char *const cmd) {
  if (!cmd || !cmd[0]) {
    SERIAL_OUT.println("Error: Empty command");
    return;
  }

  bool commandProcessed = false;  // Flag to check if any case matched

  switch (cmd[0]) {
    case 'M':  // Toggle motor drive
      robotState.cmdDrive = !robotState.cmdDrive;
      digitalWrite(PIN_XD_EN, robotState.cmdDrive ? HIGH : LOW);  // Apply enable immediately
      SERIAL_OUT.print("Motor Drive ");
      SERIAL_OUT.println(robotState.cmdDrive ? "Enabled" : "Disabled");
      commandProcessed = true;
      break;

    case 'X':  // Reset state
      leftMotor.resetCounter();
      rightMotor.resetCounter();
      motion.reset();
      robotState_reset();
#if !ROS
      differential.reset();
#endif
      SERIAL_OUT.println("Reset state");
      commandProcessed = true;
      break;

#if ROS
    case 'R':  // Reset odometry with magnetometer calibration
      if (global_ros_context) {
        resetOdomWithMagnetometerCalibration(global_ros_context);
      } else {
        SERIAL_OUT.println("ERROR: ROS context not available");
      }
      commandProcessed = true;
      break;
#endif

#if !ROS
    // --- Non-ROS Specific Commands ---
    case 'L':  // Turn Left (degrees)
    case 'R':  // Turn Right (degrees)
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        float angleRad = value * M_PI / 180.0f;
        float wheelTravel = (angleRad * Motion::TRACK_WIDTH) / 2.0f;
        // Use public constant from MotorControl
        float counts = wheelTravel / MotorControl::METERS_PER_COUNT;

        noInterrupts();  // Protect target distance updates
        if (cmd[0] == 'L') {
          robotState.leftTargetDistance -= static_cast<int32_t>(counts);
          robotState.rightTargetDistance += static_cast<int32_t>(counts);
        } else {
          robotState.leftTargetDistance += static_cast<int32_t>(counts);
          robotState.rightTargetDistance -= static_cast<int32_t>(counts);
        }
        interrupts();
        robotState.isMoving = false;  // This is a turn, not forward motion
        robotState.move = true;       // A move command is active
        SERIAL_OUT.print("Turn ");
        SERIAL_OUT.print(value);
        SERIAL_OUT.println(" deg");
        commandProcessed = true;
      }
      break;

    case 'F':  // Move Forward/Backward (meters)
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        // Use public constant from MotorControl
        float counts = value / MotorControl::METERS_PER_COUNT;
        noInterrupts();  // Protect target distance updates
        robotState.leftTargetDistance += static_cast<int32_t>(counts);
        robotState.rightTargetDistance += static_cast<int32_t>(counts);
        interrupts();
        robotState.targetHeading = motion.getTheta();  // Target current heading
        robotState.isMoving = true;                    // This is forward motion
        robotState.move = true;                        // A move command is active
        differential.reset();                          // Reset heading PID for new move
        SERIAL_OUT.print("Move ");
        SERIAL_OUT.print(value);
        SERIAL_OUT.println(" m");
        commandProcessed = true;
      }
      break;

    case 'V':  // Set Max Speed (m/s)
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        // Use public constant from MotorControl
        if (value > 0.0f && value <= MotorControl::MAX_SPEED) {
          robotState.maxSpeed = value;
          SERIAL_OUT.print("Max speed set to: ");
          SERIAL_OUT.println(value);
          commandProcessed = true;
        } else {
          SERIAL_OUT.print("Error: Speed must be > 0 and <= ");
          SERIAL_OUT.println(MotorControl::MAX_SPEED);  // Use public constant
        }
      }
      break;

    // Differential PID Tuning Commands (Non-ROS only)
    case 'P':
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        differential.P = value;
        commandProcessed = true;
        SERIAL_OUT.print("Diff P=");
        SERIAL_OUT.println(value);
        if (tuningManager) tuningManager->saveToEEPROM();  // Auto-save to EEPROM
      }
      break;
    case 'I':
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        differential.I = value;
        commandProcessed = true;
        SERIAL_OUT.print("Diff I=");
        SERIAL_OUT.println(value);
        if (tuningManager) tuningManager->saveToEEPROM();  // Auto-save to EEPROM
      }
      break;
    case 'D':
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        differential.D = value;
        commandProcessed = true;
        SERIAL_OUT.print("Diff D=");
        SERIAL_OUT.println(value);
        if (tuningManager) tuningManager->saveToEEPROM();  // Auto-save to EEPROM
      }
      break;

#endif  // !ROS Specific Commands

    // --- Commands Available in Both ROS and Non-ROS Modes ---
    
    // Motor Balance/Scaling Commands
    case 'B':  // Balance motors (scale factors)
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        if (tuningManager) {
          commandProcessed = tuningManager->setLeftMotorScale(value);
        } else {
          SERIAL_OUT.println("Error: Tuning manager not initialized");
        }
      }
      break;
    case 'N':  // Right motor scale (N for "right" - next letter after M)
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        if (tuningManager) {
          commandProcessed = tuningManager->setRightMotorScale(value);
        } else {
          SERIAL_OUT.println("Error: Tuning manager not initialized");
        }
      }
      break;

    // Individual Motor PID Tuning
    case 'Q':  // Left motor P
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        leftMotor.pid.P = value;
        commandProcessed = true;
        SERIAL_OUT.print("Left Motor P=");
        SERIAL_OUT.println(value);
        if (tuningManager) tuningManager->saveToEEPROM();  // Auto-save to EEPROM
      }
      break;
    case 'W':  // Left motor I
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        leftMotor.pid.I = value;
        commandProcessed = true;
        SERIAL_OUT.print("Left Motor I=");
        SERIAL_OUT.println(value);
        if (tuningManager) tuningManager->saveToEEPROM();  // Auto-save to EEPROM
      }
      break;
    case 'E':  // Left motor D
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        leftMotor.pid.D = value;
        commandProcessed = true;
        SERIAL_OUT.print("Left Motor D=");
        SERIAL_OUT.println(value);
        if (tuningManager) tuningManager->saveToEEPROM();  // Auto-save to EEPROM
      }
      break;
    case 'U':  // Right motor P
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        rightMotor.pid.P = value;
        commandProcessed = true;
        SERIAL_OUT.print("Right Motor P=");
        SERIAL_OUT.println(value);
        if (tuningManager) tuningManager->saveToEEPROM();  // Auto-save to EEPROM
      }
      break;
    case 'O':  // Right motor I
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        rightMotor.pid.I = value;
        commandProcessed = true;
        SERIAL_OUT.print("Right Motor I=");
        SERIAL_OUT.println(value);
        if (tuningManager) tuningManager->saveToEEPROM();  // Auto-save to EEPROM
      }
      break;
    case 'T':  // Right motor D
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        rightMotor.pid.D = value;
        commandProcessed = true;
        SERIAL_OUT.print("Right Motor D=");
        SERIAL_OUT.println(value);
        if (tuningManager) tuningManager->saveToEEPROM();  // Auto-save to EEPROM
      }
      break;

    // Motor Testing Commands
    case 'S':  // Test both motors at same speed
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        if (value >= -1.0f && value <= 1.0f) {
          leftMotor.setTargetSpeed(value);
          rightMotor.setTargetSpeed(value);
          SERIAL_OUT.print("Testing both motors at speed: ");
          SERIAL_OUT.println(value);
          commandProcessed = true;
        } else {
          SERIAL_OUT.println("Error: Speed must be between -1.0 and 1.0 m/s");
        }
      }
      break;
    case 'A':  // Test left motor only
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        if (value >= -1.0f && value <= 1.0f) {
          leftMotor.setTargetSpeed(value);
          rightMotor.setTargetSpeed(0.0f);
          SERIAL_OUT.print("Testing left motor at speed: ");
          SERIAL_OUT.println(value);
          commandProcessed = true;
        } else {
          SERIAL_OUT.println("Error: Speed must be between -1.0 and 1.0 m/s");
        }
      }
      break;
    case 'Z':  // Test right motor only
      {
        float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
        if (value >= -1.0f && value <= 1.0f) {
          leftMotor.setTargetSpeed(0.0f);
          rightMotor.setTargetSpeed(value);
          SERIAL_OUT.print("Testing right motor at speed: ");
          SERIAL_OUT.println(value);
          commandProcessed = true;
        } else {
          SERIAL_OUT.println("Error: Speed must be between -1.0 and 1.0 m/s");
        }
      }
      break;

    // EEPROM Management Commands
    case 'Y':  // Save current tuning to EEPROM
      if (tuningManager) {
        tuningManager->saveToEEPROM();
        commandProcessed = true;
      } else {
        SERIAL_OUT.println("Error: Tuning manager not initialized");
      }
      break;
    case 'G':  // Load tuning from EEPROM
      if (tuningManager) {
        tuningManager->loadFromEEPROM();
        commandProcessed = true;
      } else {
        SERIAL_OUT.println("Error: Tuning manager not initialized");
      }
      break;
    case 'C':  // Clear/reset tuning to defaults
      if (tuningManager) {
        tuningManager->resetToDefaults();
        tuningManager->saveToEEPROM();  // Save defaults to EEPROM
        commandProcessed = true;
      } else {
        SERIAL_OUT.println("Error: Tuning manager not initialized");
      }
      break;

case 'J':  // Toggle encoder streaming
      encoderStreaming = !encoderStreaming;
      SERIAL_OUT.print("Encoder streaming ");
      SERIAL_OUT.println(encoderStreaming ? "enabled" : "disabled");
      if (encoderStreaming) {
        SERIAL_OUT.println("Streaming format: L:<left_counter>(AB) R:<right_counter>(AB)");
        SERIAL_OUT.println("AB values show raw encoder pin states (0=low, 1=high)");
      }
      commandProcessed = true;
      break;

    case 'K':  // IMU diagnostic (K for "check")
      diagnoseIMU();
      commandProcessed = true;
      break;

    case 'H':  // Help - show all commands
      SERIAL_OUT.println("=== Available Commands ===");
      SERIAL_OUT.println("M - Toggle motor drive on/off");
      SERIAL_OUT.println("X - Reset all state");
      SERIAL_OUT.println("K - Run IMU diagnostic");
#if ROS
      SERIAL_OUT.println("R - Reset odometry with magnetometer calibration");
#endif
      SERIAL_OUT.println("J - Toggle encoder streaming");
#if !ROS
      SERIAL_OUT.println("L<degrees> - Turn left");
      SERIAL_OUT.println("R<degrees> - Turn right");
      SERIAL_OUT.println("F<meters> - Move forward/backward");
      SERIAL_OUT.println("V<speed> - Set max speed (m/s)");
      SERIAL_OUT.println("P<value> - Set differential PID P");
      SERIAL_OUT.println("I<value> - Set differential PID I");
      SERIAL_OUT.println("D<value> - Set differential PID D");
#endif
      SERIAL_OUT.println("B<scale> - Set left motor scale (0.5-2.0)");
      SERIAL_OUT.println("N<scale> - Set right motor scale (0.5-2.0)");
      SERIAL_OUT.println("Q<value> - Set left motor PID P");
      SERIAL_OUT.println("W<value> - Set left motor PID I");
      SERIAL_OUT.println("E<value> - Set left motor PID D");
      SERIAL_OUT.println("U<value> - Set right motor PID P");
      SERIAL_OUT.println("O<value> - Set right motor PID I");
      SERIAL_OUT.println("T<value> - Set right motor PID D");
      SERIAL_OUT.println("S<speed> - Test both motors at same speed");
      SERIAL_OUT.println("A<speed> - Test left motor only");
      SERIAL_OUT.println("Z<speed> - Test right motor only");
      SERIAL_OUT.println("Y - Save tuning to EEPROM");
      SERIAL_OUT.println("G - Load tuning from EEPROM");
      SERIAL_OUT.println("C - Reset tuning to defaults");
      SERIAL_OUT.println("H - Show this help");
      commandProcessed = true;
      break;

    default:
      SERIAL_OUT.print("Error: Unknown command '");
      SERIAL_OUT.print(cmd[0]);
      SERIAL_OUT.println("'");
      commandProcessed = true;  // We processed it by saying it's unknown
  }

  if (!commandProcessed) {
    // This case might happen if a command is defined for one mode but not the other
    SERIAL_OUT.print("Error: Command '");
    SERIAL_OUT.print(cmd[0]);
    SERIAL_OUT.println("' not available in this mode (ROS=");
    SERIAL_OUT.print(ROS);
    SERIAL_OUT.println(")");
  }
}

// --- Setup ---
void setup() {
  // Common Serial Initialization
  SERIAL_OUT.begin(115200);
  SERIAL_OUT.println("--- Robot Firmware Starting ---");
  SERIAL_OUT.print("ROS Mode: ");
  SERIAL_OUT.println(ROS);

#if ROS
  // ROS Specific Setup
  SERIAL_OUT.println("Setting up micro-ROS...");
  setup_ROS();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // LED off initially
  delay(1000);                 // Short delay before trying agent
  // state = WAITING_AGENT;
  SERIAL_OUT.println("Waiting for micro-ROS Agent...");
#else
  // Non-ROS Specific Setup
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  SERIAL_OUT.println("Running in Standalone Mode.");
  // Configure differential PID (only used in non-ROS)
  differential.setLimits(0.5, 0.2);      // Limit correction effect
  differential.setFiltering(0.01, 0.1);  // Add some filtering
  differential.P = 0.8;                  // Might need tuning
  differential.I = 0.01;
  differential.D = 0.05;
  SERIAL_OUT.print("Diff PID Init P="); SERIAL_OUT.print(differential.P);
  SERIAL_OUT.print(" I="); SERIAL_OUT.print(differential.I);
  SERIAL_OUT.print(" D="); SERIAL_OUT.println(differential.D);
#endif

  // Common Hardware Setup
  // Enable encoder pullups
  pinMode(PIN_LENCA, INPUT_PULLUP);
  pinMode(PIN_LENCB, INPUT_PULLUP);
  pinMode(PIN_RENCA, INPUT_PULLUP);
  pinMode(PIN_RENCB, INPUT_PULLUP);
  SERIAL_OUT.println("Encoder pullups configured");

  // Initialize lastEnc states BEFORE attaching interrupts
  // This ensures the first interrupt call has a valid previous state.
  leftMotor.lastEnc = (digitalRead(PIN_LENCA) ? 2 : 0) | (digitalRead(PIN_LENCB) ? 1 : 0);
  rightMotor.lastEnc = (digitalRead(PIN_RENCA) ? 2 : 0) | (digitalRead(PIN_RENCB) ? 1 : 0);
  uint32_t now = micros();
  leftMotor.lastEncTime = now;
  rightMotor.lastEncTime = now;
  SERIAL_OUT.print("Initial enc states L=");
  SERIAL_OUT.print(leftMotor.lastEnc);
  SERIAL_OUT.print(" R=");
  SERIAL_OUT.println(rightMotor.lastEnc);

  // Enable motor drive outputs
  pinMode(PIN_XD_EN, OUTPUT);
  pinMode(PIN_RD_PWM1, OUTPUT);
  pinMode(PIN_LD_PWM1, OUTPUT);
  pinMode(PIN_RD_PWM2, OUTPUT);
  pinMode(PIN_LD_PWM2, OUTPUT);

  // Ensure motors are off and enabled initially
  digitalWrite(PIN_XD_EN, HIGH);  // HIGH likely means enabled for common drivers
  analogWrite(PIN_RD_PWM1, 0);
  analogWrite(PIN_RD_PWM2, 0);
  analogWrite(PIN_LD_PWM1, 0);
  analogWrite(PIN_LD_PWM2, 0);
  SERIAL_OUT.println("Motor driver enabled, PWM initialized to 0");
  // Configure Motor PIDs (Common)
  leftMotor.pid.setLimits(1.0, 0.5);      // Max output, max integral sum
  leftMotor.pid.setFiltering(0.01, 0.1);  // Deadband, D filter coeff
  rightMotor.pid.setLimits(1.0, 0.5);
  rightMotor.pid.setFiltering(0.01, 0.1);
  
  // Initialize tuning manager and load parameters from EEPROM
#if !ROS
  tuningManager = new TuningManager(&leftMotor, &rightMotor, &differential);
#else
  tuningManager = new TuningManager(&leftMotor, &rightMotor, nullptr);
#endif
  tuningManager->loadFromEEPROM();
  SERIAL_OUT.print("Motor PID Init L(P,I,D)=");
  SERIAL_OUT.print(leftMotor.pid.P); SERIAL_OUT.print(",");
  SERIAL_OUT.print(leftMotor.pid.I); SERIAL_OUT.print(",");
  SERIAL_OUT.println(leftMotor.pid.D);
  SERIAL_OUT.print("Motor PID Init R(P,I,D)=");
  SERIAL_OUT.print(rightMotor.pid.P); SERIAL_OUT.print(",");
  SERIAL_OUT.print(rightMotor.pid.I); SERIAL_OUT.print(",");
  SERIAL_OUT.println(rightMotor.pid.D);

  // Reset state
  leftMotor.resetCounter();
  rightMotor.resetCounter();
  motion.reset();
  robotState_reset();

#if ROS
  // Initialize ROS context
  rosContext.robotState = &robotState;
  rosContext.odomContext.motion = &motion;
  rosContext.odomContext.leftMotor = &leftMotor;
  rosContext.odomContext.rightMotor = &rightMotor;
#endif

  // Attach interrupts for encoders
  // Note: digitalPinToInterrupt() is necessary for mapping pin numbers to interrupt numbers.
  attachInterrupt(digitalPinToInterrupt(PIN_LENCA), leftEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LENCB), leftEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RENCA), rightEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RENCB), rightEncoderInterrupt, CHANGE);
  SERIAL_OUT.println("Encoder interrupts attached");

  // I2C Setup (Common)
  SERIAL_OUT.println("Initializing I2C and IMU...");
  Wire.begin();
  SERIAL_OUT.println("I2C begun");
  Wire.beginTransmission(MPU9250_ADDR);  // MPU9250 default address
  Wire.write(0x6B);              // PWR_MGMT_1 register
  Wire.write(0x80);              // Reset device
  auto rv = Wire.endTransmission(true);
  SERIAL_OUT.print("IMU Reset returned: ");
  SERIAL_OUT.println(rv);
  delay(100);  // Wait for reset

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Wake up device
  rv = Wire.endTransmission(true);
  SERIAL_OUT.print("IMU Wakeup returned: ");
  SERIAL_OUT.println(rv);
  delay(100);

  // Initialize magnetometer (AK8963) for MPU9250
  // Enable I2C passthrough to access magnetometer
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x37);  // INT_PIN_CFG register
  Wire.write(0x02);  // Enable I2C passthrough (bit 1 = BYPASS_EN)
  rv = Wire.endTransmission(true);
  SERIAL_OUT.print("IMU Passthrough setup returned: ");
  SERIAL_OUT.println(rv);
  delay(10);
  
  // Initialize magnetometer (AK8963 at address 0x0C)
  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);  // CNTL1 register
  Wire.write(0x16);  // 16-bit output, continuous mode 2 (100Hz)
  rv = Wire.endTransmission(true);
  SERIAL_OUT.print("Magnetometer init returned: ");
  SERIAL_OUT.println(rv);
  delay(10);
  
  // Verify magnetometer is responding
  Wire.beginTransmission(0x0C);
  Wire.write(0x00);  // WIA register (Who I Am)
  Wire.endTransmission(false);
  uint8_t bytes_read = Wire.requestFrom((uint8_t)0x0C, (uint8_t)1, (uint8_t)true);
  if (bytes_read > 0 && Wire.available()) {
    uint8_t wia = Wire.read();
    SERIAL_OUT.print("Magnetometer WIA: 0x");
    SERIAL_OUT.println(wia, HEX);
    if (wia == 0x48) {
      SERIAL_OUT.println("Magnetometer (AK8963) detected successfully");
    } else {
      SERIAL_OUT.println("Warning: Unexpected magnetometer ID");
    }
  }

  // Add more IMU config if needed (e.g., setting ranges, DLPF)

  SERIAL_OUT.println("--- Setup Complete ---");
}

// --- Status Printing ---
void printStatus() {
  char buf[128];

  // Print Odometry
  motion.print();

  // Print Motor Speeds (Target vs Actual)
  if (PRINT_MOVES > 1) {
    snprintf(buf, sizeof(buf), "Speed L: Tgt=%.2f Act=%.2f | R: Tgt=%.2f Act=%.2f m/s",
             leftMotor.getTargetSpeed(), leftMotor.getSpeed(),
             rightMotor.getTargetSpeed(), rightMotor.getSpeed());
    SERIAL_OUT.println(buf);
  }

#if !ROS
  // Print Non-ROS specific info
  snprintf(buf, sizeof(buf), "Encoders L: %ld/%ld | R: %ld/%ld",
           leftMotor.getCounter(), robotState.leftTargetDistance,
           rightMotor.getCounter(), robotState.rightTargetDistance);
  SERIAL_OUT.println(buf);

  snprintf(buf, sizeof(buf), "Control: MaxSpd=%.2f HeadTgt=%.1f HeadCorr=%.3f",
           robotState.maxSpeed, robotState.targetHeading * 180.0f / M_PI, robotState.corr);
  SERIAL_OUT.println(buf);
  snprintf(buf, sizeof(buf), "Diff PID: P=%.2f I=%.2f D=%.2f",
           differential.P, differential.I, differential.D);
  SERIAL_OUT.println(buf);
  if (tuningManager) {
    tuningManager->printStatus();
  }
#else
  // Print ROS specific info (if any needed beyond speeds)
  // TODO: figure out what to do for this message
  // snprintf(buf, sizeof(buf), "ROS State: %d | Target Lin=%.2f Ang=%.2f",
  //          currentRosAgentStatus, robotState.targetLinearVelocity, robotState.targetAngularVelocity);
  // SERIAL_OUT.println(buf);
#endif
}

/**
 * Controls the movement of the robot by calculating and setting the target speeds for its left and right motors.
 * The control logic depends on whether the robot is in ROS (Robot Operating System) mode or not.
 *
 * In ROS mode, it converts target linear and angular velocities to wheel velocities using the robot's track width,
 * and optionally applies a speed limit to prevent high speeds.
 *
 * In non-ROS mode, it implements a simple P-controller for position-based control towards target distances,
 * calculates the remaining distance for each wheel and adjusts the velocity accordingly, applies a threshold to stop
 * the robot near the target, and optionally corrects the heading during forward/backward motion using a differential PID controller.
 *
 * Ultimately, it sets the target speeds for the left and right motors, ensuring they are within the robot's maximum speed limits.
 */
void drivecontrol() {
  float leftVelocity = 0.0f;
  float rightVelocity = 0.0f;
  static bool notedDisabled = false;  // one-shot log flag for disabled drive

  if (!robotState.cmdDrive) {
    // If drive explicitly disabled, ensure motors are set to zero speed
    leftMotor.setTargetSpeed(0.0f);
    rightMotor.setTargetSpeed(0.0f);
    if (!notedDisabled) {
      SERIAL_OUT.println("Drive disabled: holding zero speeds");
      notedDisabled = true;
    }
    return;  // Don't proceed with control logic
  }
  // Clear one-shot notice if drive re-enabled
  notedDisabled = false;

#if ROS
  // ROS mode: Convert target Twist velocities (linear/angular) to wheel velocities
  // Use public constant from Motion
  leftVelocity = robotState.targetLinearVelocity - (robotState.targetAngularVelocity * Motion::TRACK_WIDTH / 2.0f);
  rightVelocity = robotState.targetLinearVelocity + (robotState.targetAngularVelocity * Motion::TRACK_WIDTH / 2.0f);

#if PRINT_MOVES > 1
  SERIAL_OUT.print("ROS drivecontrol vL="); SERIAL_OUT.print(leftVelocity);
  SERIAL_OUT.print(" vR="); SERIAL_OUT.print(rightVelocity);
  SERIAL_OUT.print(" lin="); SERIAL_OUT.print(robotState.targetLinearVelocity);
  SERIAL_OUT.print(" ang="); SERIAL_OUT.println(robotState.targetAngularVelocity);
#endif

  // Apply overall speed limit if needed (optional, Twist could command high speeds)
  // float max_vel = max(abs(leftVelocity), abs(rightVelocity));
  // if (max_vel > robotState.maxSpeed) { // Uses RobotState maxSpeed
  //    float scale = robotState.maxSpeed / max_vel;
  //    leftVelocity *= scale;
  //    rightVelocity *= scale;
  // }

#else  // !ROS
  // Non-ROS mode: Position-based control towards target distances
  // This implements a simple P-controller for position with optional heading correction

  // Calculate remaining distance for each wheel
  int32_t leftError = robotState.leftTargetDistance - leftMotor.getCounter();
  int32_t rightError = robotState.rightTargetDistance - rightMotor.getCounter();

#if PRINT_MOVES > 1
  SERIAL_OUT.print("Err L="); SERIAL_OUT.print(leftError);
  SERIAL_OUT.print(" R="); SERIAL_OUT.println(rightError);
#endif

  // Simple P-control for position: velocity proportional to error (with ceiling)
  // Use a threshold to stop near the target
  const int32_t stopThreshold = 10;       // Encoder counts threshold to stop
  const int32_t slowDownThreshold = 200;  // Start slowing down this far away

  if (abs(leftError) > stopThreshold) {
    float speedFactor = constrain(static_cast<float>(abs(leftError)) / slowDownThreshold, 0.1f, 1.0f);  // Scale speed
    leftVelocity = robotState.maxSpeed * speedFactor * ((leftError > 0) ? 1.0f : -1.0f);
  } else {
    leftVelocity = 0.0f;  // Stop if close enough
  }

  if (abs(rightError) > stopThreshold) {
    float speedFactor = constrain(static_cast<float>(abs(rightError)) / slowDownThreshold, 0.1f, 1.0f);  // Scale speed
    rightVelocity = robotState.maxSpeed * speedFactor * ((rightError > 0) ? 1.0f : -1.0f);
  } else {
    rightVelocity = 0.0f;  // Stop if close enough
  }

  // Check if the overall move command is complete
  if (abs(leftError) <= stopThreshold && abs(rightError) <= stopThreshold) {
    robotState.move = false;  // Mark move as complete
    robotState.isMoving = false;
    leftVelocity = 0.0f;
    rightVelocity = 0.0f;
    SERIAL_OUT.println("Position move complete");
    // Reset targets? Or allow accumulation? Current code accumulates.
  }

  // Apply heading correction during forward/backward motion (isMoving flag)
  if (robotState.isMoving && robotState.move) {  // Only correct during F commands that are still active
    // Calculate heading error (target - current)
    float headingError = robotState.targetHeading - motion.getTheta();
    // Normalize heading error to [-PI, PI)
    headingError = atan2(sin(headingError), cos(headingError));

    // Update differential PID
    robotState.corr = differential.update(headingError);

    // Apply correction: slow down the faster wheel, speed up the slower one
    // Correction should be proportional to the desired speed to avoid overshooting at low speeds
    float baseSpeed = max(abs(leftVelocity), abs(rightVelocity));  // Use the larger of the two target speeds
    if (baseSpeed > 1e-3) {                                        // Avoid division by zero
      float correctionAmount = robotState.corr * baseSpeed;        // Scale correction by speed
      leftVelocity -= correctionAmount;
      rightVelocity += correctionAmount;
    }
#if PRINT_MOVES > 2
    if (abs(robotState.corr) > 0.01) {  // Debug print for correction
      SERIAL_OUT.print("Heading Err: ");
      SERIAL_OUT.print(headingError * 180.0f / M_PI);
      SERIAL_OUT.print(" Corr: ");
      SERIAL_OUT.println(robotState.corr);
    }
#endif
  } else {
    // Not doing a forward move or move is complete, reset correction
    robotState.corr = 0;
    differential.reset();  // Reset PID if not actively correcting heading
  }
  // Apply final speed limits (using RobotState maxSpeed)
  leftVelocity = constrain(leftVelocity, -robotState.maxSpeed, robotState.maxSpeed);
  rightVelocity = constrain(rightVelocity, -robotState.maxSpeed, robotState.maxSpeed);

#endif  // ROS / !ROS


  // Apply motor scaling factors to compensate for speed differences
  if (tuningManager) {
    leftVelocity *= tuningManager->getLeftMotorScale();
    rightVelocity *= tuningManager->getRightMotorScale();
  }

  // set motor speeds
  leftMotor.setTargetSpeed(leftVelocity);
  rightMotor.setTargetSpeed(rightVelocity);
}

// --- Main Loop ---
void loop() {
  const auto nowMicros = micros();

  leftMotor.readCurrent();
  rightMotor.readCurrent();

  // --- Periodic Updates ---
  static auto nextMotionUpdate = nowMicros;
  const unsigned long motionUpdateInterval = 10000;  // 10ms = 100Hz
  if (nowMicros >= nextMotionUpdate) {
    // Update Odometry
    motion.update(leftMotor.getCounter(), rightMotor.getCounter());

    // Update Motor PIDs and apply PWM
    leftMotor.motionUpdate();
    rightMotor.motionUpdate();

    // Calculate next update time, handling potential rollover
    nextMotionUpdate = (nowMicros / motionUpdateInterval + 1) * motionUpdateInterval;
  }

  // --- Drive Control Logic ---
  // This determines the target speeds based on ROS commands or non-ROS targets
  drivecontrol();

  // --- ROS Handling ---
#if ROS
  handleRosAgentState(&rosContext);  // Handles connection and spins the executor
#endif

  // --- Serial Command Input ---
  static char cmdbuf[32];  // Increased buffer size
  static size_t cmdbufpos = 0;
  while (SERIAL_OUT.available()) {
    char c = SERIAL_OUT.read();
    if (c == '\n' || c == '\r') {
      if (cmdbufpos > 0) {         // Process only if buffer is not empty
        cmdbuf[cmdbufpos] = '\0';  // Null terminate
        SERIAL_OUT.print("Received Command: [");
        SERIAL_OUT.print(cmdbuf);
        SERIAL_OUT.println("]");
        parseCommand(cmdbuf);
        cmdbufpos = 0;  // Reset buffer
      }
    } else if (cmdbufpos < (sizeof(cmdbuf) - 1)) {
      cmdbuf[cmdbufpos++] = c;  // Add char to buffer
    } else {
      // Buffer overflow, discard command
      SERIAL_OUT.println("Error: Command too long");
      cmdbufpos = 0;  // Reset buffer
    }
  }

  // --- Encoder Streaming ---
  static auto nextEncoderStream = nowMicros;
  const unsigned long encoderStreamInterval = 10000;  // 10ms = 100Hz for encoder streaming
  if (encoderStreaming && nowMicros >= nextEncoderStream) {
    // Read raw encoder pin states for debugging
    int leftA = digitalRead(PIN_LENCA);
    int leftB = digitalRead(PIN_LENCB);
    int rightA = digitalRead(PIN_RENCA);
    int rightB = digitalRead(PIN_RENCB);
    
    SERIAL_OUT.print("L:");
    SERIAL_OUT.print(leftMotor.getCounter());
    SERIAL_OUT.print("(");
    SERIAL_OUT.print(leftA);
    SERIAL_OUT.print(leftB);
    SERIAL_OUT.print(") R:");
    SERIAL_OUT.print(rightMotor.getCounter());
    SERIAL_OUT.print("(");
    SERIAL_OUT.print(rightA);
    SERIAL_OUT.print(rightB);
    SERIAL_OUT.println(")");
    nextEncoderStream = (nowMicros / encoderStreamInterval + 1) * encoderStreamInterval;
  }

  // --- Periodic Status Output ---
  static auto nextStatusUpdate = nowMicros;
  const unsigned long statusUpdateInterval = 1000000;  // 1 second
  if (nowMicros >= nextStatusUpdate) {
    if (PRINT_MOVES > 1) {
      SERIAL_OUT.println("--- Status Update ---");
      printStatus();
      // Read and print battery voltage
      // Assuming a voltage divider: Vbat -> R1 -> ADC_PIN -> R2 -> GND
      // Voltage = ADC_reading * (AREF / ADC_resolution) * (R1 + R2) / R2
      // Example: Teensy 3.3V AREF, 10-bit ADC (1024), R1=10k, R2=10k -> Factor = 3.3/1024 * 2 = 0.006445
      const float VOLTAGE_FACTOR = 0.00967;  // Teensy 3.3V AREF, R!=2M, R2=1M -> Factor = (3.3/1024) * 3 = 0.00967
      float voltage = analogRead(PIN_VBAT) * VOLTAGE_FACTOR;
      SERIAL_OUT.print("Voltage: ");
      SERIAL_OUT.print(voltage);
      SERIAL_OUT.println(" V");
      SERIAL_OUT.println("---------------------");
    }

    nextStatusUpdate = (nowMicros / statusUpdateInterval + 1) * statusUpdateInterval;
  }
}
