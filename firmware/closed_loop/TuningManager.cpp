#include "TuningManager.h"
#include "MotorControl.h"
#include "PidControl.h"
#include "setup.h"  // For SERIAL_OUT

TuningManager::TuningManager(MotorControl* leftMotor, MotorControl* rightMotor, PidControl* differential)
  : leftMotor(leftMotor), rightMotor(rightMotor), differential(differential),
    leftMotorScale(1.0f), rightMotorScale(1.0f) {
}

void TuningManager::loadFromEEPROM() {
  TuningParams params;
  EEPROM.get(TUNING_EEPROM_ADDR, params);
  
  // Check if data is valid
  if (params.magic == TUNING_MAGIC) {
    leftMotorScale = params.leftMotorScale;
    rightMotorScale = params.rightMotorScale;
    
    // Update motor PID parameters
    if (leftMotor) {
      leftMotor->pid.P = params.leftMotorP;
      leftMotor->pid.I = params.leftMotorI;
      leftMotor->pid.D = params.leftMotorD;
    }
    
    if (rightMotor) {
      rightMotor->pid.P = params.rightMotorP;
      rightMotor->pid.I = params.rightMotorI;
      rightMotor->pid.D = params.rightMotorD;
    }
    
    // Update differential PID parameters (if available)
    if (differential) {
      differential->P = params.differentialP;
      differential->I = params.differentialI;
      differential->D = params.differentialD;
    }
    
    SERIAL_OUT.println("Tuning parameters loaded from EEPROM");
    SERIAL_OUT.print("Motor Scales: L="); SERIAL_OUT.print(leftMotorScale);
    SERIAL_OUT.print(" R="); SERIAL_OUT.println(rightMotorScale);
  } else {
    SERIAL_OUT.println("No valid tuning data in EEPROM, using defaults");
  }
}

void TuningManager::saveToEEPROM() {
  TuningParams params;
  params.leftMotorScale = leftMotorScale;
  params.rightMotorScale = rightMotorScale;
  
  // Get current PID parameters from motor objects
  if (leftMotor) {
    params.leftMotorP = leftMotor->pid.P;
    params.leftMotorI = leftMotor->pid.I;
    params.leftMotorD = leftMotor->pid.D;
  } else {
    params.leftMotorP = 8.0f;
    params.leftMotorI = 0.8f;
    params.leftMotorD = 0.0f;
  }
  
  if (rightMotor) {
    params.rightMotorP = rightMotor->pid.P;
    params.rightMotorI = rightMotor->pid.I;
    params.rightMotorD = rightMotor->pid.D;
  } else {
    params.rightMotorP = 8.0f;
    params.rightMotorI = 0.8f;
    params.rightMotorD = 0.0f;
  }
  
  // Get differential PID parameters (if available)
  if (differential) {
    params.differentialP = differential->P;
    params.differentialI = differential->I;
    params.differentialD = differential->D;
  } else {
    params.differentialP = 0.0f;
    params.differentialI = 0.0f;
    params.differentialD = 0.0f;
  }
  
  params.magic = TUNING_MAGIC;
  
  EEPROM.put(TUNING_EEPROM_ADDR, params);
  SERIAL_OUT.println("Tuning parameters saved to EEPROM");
}

void TuningManager::resetToDefaults() {
  leftMotorScale = 1.0f;
  rightMotorScale = 1.0f;
  
  // Reset motor PID parameters
  if (leftMotor) {
    leftMotor->pid.P = 8.0f;
    leftMotor->pid.I = 0.8f;
    leftMotor->pid.D = 0.0f;
  }
  
  if (rightMotor) {
    rightMotor->pid.P = 8.0f;
    rightMotor->pid.I = 0.8f;
    rightMotor->pid.D = 0.0f;
  }
  
  // Reset differential PID parameters (if available)
  if (differential) {
    differential->P = 0.8f;
    differential->I = 0.01f;
    differential->D = 0.05f;
  }
  
  SERIAL_OUT.println("Tuning parameters reset to defaults");
}

bool TuningManager::setLeftMotorScale(float scale) {
  if (scale >= 0.2f && scale <= 5.0f) {
    leftMotorScale = scale;
    SERIAL_OUT.print("Left motor scale set to: ");
    SERIAL_OUT.println(scale);
    saveToEEPROM();  // Auto-save to EEPROM
    return true;
  } else {
    SERIAL_OUT.println("Error: Scale must be between 0.2 and 5.0");
    return false;
  }
}

bool TuningManager::setRightMotorScale(float scale) {
  if (scale >= 0.2f && scale <= 5.0f) {
    rightMotorScale = scale;
    SERIAL_OUT.print("Right motor scale set to: ");
    SERIAL_OUT.println(scale);
    saveToEEPROM();  // Auto-save to EEPROM
    return true;
  } else {
    SERIAL_OUT.println("Error: Scale must be between 0.2 and 5.0");
    return false;
  }
}

void TuningManager::printStatus() const {
  char buf[128];
  snprintf(buf, sizeof(buf), "Motor Scales: L=%.3f R=%.3f",
           leftMotorScale, rightMotorScale);
  SERIAL_OUT.println(buf);
}
