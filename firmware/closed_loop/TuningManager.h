#ifndef TUNING_MANAGER_H
#define TUNING_MANAGER_H

#include <Arduino.h>
#include <EEPROM.h>

// Forward declarations
class MotorControl;
class PidControl;

// EEPROM structure for persistent tuning parameters
struct TuningParams {
  float leftMotorScale;
  float rightMotorScale;
  float leftMotorP;
  float leftMotorI;
  float leftMotorD;
  float rightMotorP;
  float rightMotorI;
  float rightMotorD;
  float differentialP;
  float differentialI;
  float differentialD;
  uint32_t magic; // Magic number to verify data integrity
};

/**
 * @class TuningManager
 * @brief Manages persistent storage of motor tuning parameters in EEPROM
 * 
 * This class handles saving and loading of motor scaling factors and PID parameters
 * to/from EEPROM, allowing tuning to persist across power cycles.
 */
class TuningManager {
private:
  static const uint32_t TUNING_MAGIC = 0x12345678;
  static const int TUNING_EEPROM_ADDR = 0;
  
  // Reference to motor objects for accessing PID parameters
  MotorControl* leftMotor;
  MotorControl* rightMotor;
  PidControl* differential;
  
  // Current tuning parameters
  float leftMotorScale;
  float rightMotorScale;
  
public:
  /**
   * @brief Constructor
   * @param leftMotor Pointer to left motor control object
   * @param rightMotor Pointer to right motor control object
   * @param differential Pointer to differential PID control object (can be nullptr in ROS mode)
   */
  TuningManager(MotorControl* leftMotor, MotorControl* rightMotor, PidControl* differential = nullptr);
  
  /**
   * @brief Load tuning parameters from EEPROM on startup
   */
  void loadFromEEPROM();
  
  /**
   * @brief Save current tuning parameters to EEPROM
   */
  void saveToEEPROM();
  
  /**
   * @brief Reset all tuning parameters to defaults
   */
  void resetToDefaults();
  
  /**
   * @brief Get left motor scale factor
   * @return Current left motor scale factor
   */
  float getLeftMotorScale() const { return leftMotorScale; }
  
  /**
   * @brief Get right motor scale factor
   * @return Current right motor scale factor
   */
  float getRightMotorScale() const { return rightMotorScale; }
  
  /**
   * @brief Set left motor scale factor
   * @param scale New scale factor (0.5 to 2.0)
   * @return true if valid range, false otherwise
   */
  bool setLeftMotorScale(float scale);
  
  /**
   * @brief Set right motor scale factor
   * @param scale New scale factor (0.5 to 2.0)
   * @return true if valid range, false otherwise
   */
  bool setRightMotorScale(float scale);
  
  /**
   * @brief Print current tuning parameters to serial
   */
  void printStatus() const;
};

#endif // TUNING_MANAGER_H
