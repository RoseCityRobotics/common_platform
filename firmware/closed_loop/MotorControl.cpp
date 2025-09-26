#include "MotorControl.h"
#include "setup.h"

void MotorControl::applyPWM(int pwmValue) {
  pwmValue = constrain(pwmValue, -MotorControl::MAX_PWM, MotorControl::MAX_PWM);
  static int lastPwm = 0;
  static int lastDir = 0;
  int dir = (pwmValue > 0) - (pwmValue < 0);
  if (dir != lastDir && (abs(pwmValue) > 0 || abs(lastPwm) > 0)) {
#if PRINT_MOVES
    SERIAL_OUT.print(label);
    SERIAL_OUT.print(" dir change ");
    SERIAL_OUT.print((lastDir >= 0) ? (lastDir == 0 ? "0" : "+") : "-");
    SERIAL_OUT.print(" -> ");
    SERIAL_OUT.println((dir >= 0) ? (dir == 0 ? "0" : "+") : "-");
#endif
  }
  if (abs(pwmValue) == MotorControl::MAX_PWM && abs(lastPwm) != MotorControl::MAX_PWM) {
    SERIAL_OUT.print(label);
    SERIAL_OUT.print(" PWM saturated at ");
    SERIAL_OUT.println(MotorControl::MAX_PWM);
  }
  if (pwmValue >= 0) {
    analogWrite(ciPwmA, pwmValue);
    analogWrite(ciPwmB, 0);
  } else {
    analogWrite(ciPwmA, 0);
    analogWrite(ciPwmB, -pwmValue);
  }
  lastPwm = pwmValue;
  lastDir = dir;
}

MotorControl::MotorControl(int pinCurrent, int pinEncA, int pinEncB,
                           int pinPwmA, int pinPwmB, const char *label)
    : ciPinCurrent(pinCurrent), ciPinEncA(pinEncA), ciPinEncB(pinEncB),
      ciPwmA(pinPwmA), ciPwmB(pinPwmB), label(label) {
  resetCounter(); // Initialize state in constructor via reset
  msgcnt = 0;
  prevCounterForSpeed =
      0; // Initialized by resetCounter if called after member init
  prevTimeForSpeed = micros(); // Initialized by resetCounter
}

int32_t MotorControl::getCounter() const {
  int32_t tempCounter;
  noInterrupts();
  tempCounter = counter;
  interrupts();
  return tempCounter;
}

float MotorControl::getSpeed() const { return lastSpeed; }

float MotorControl::getTargetSpeed() const { return targetSpeed; }

void MotorControl::setTargetSpeed(float speed) {
  speed = constrain(speed, -MotorControl::MAX_SPEED, MotorControl::MAX_SPEED);

  if (targetSpeed != speed) {
    targetSpeed = speed;
    // Consider PID reset strategy carefully (e.g., only on sign change or zero
    // crossing) pid.reset(); // Resetting integrator on any change can be jerky

#if PRINT_MOVES
    SERIAL_OUT.print(label);
    SERIAL_OUT.print(" New speed target: ");
    SERIAL_OUT.println(speed);
#endif
  }
}

void MotorControl::resetCounter() {
  noInterrupts();
  counter = 0;
  lastEncTime = micros();
  // Initialize lastEnc based on current pin readings.
  lastEnc = (digitalRead(ciPinEncA) ? 2 : 0) | (digitalRead(ciPinEncB) ? 1 : 0);
  // Reset speed calculation variables
  prevCounterForSpeed = counter; // After counter itself is reset
  interrupts();

  // These are not shared with ISRs, so can be set outside critical section
  lastSpeed = 0;
  targetSpeed = 0;
  mpwm = 0;
  pid.reset();
  prevTimeForSpeed =
      lastEncTime; // Align with lastEncTime from critical section
  SERIAL_OUT.print(label);
  SERIAL_OUT.println(" counters reset");
}

void MotorControl::readCurrent() {
  float lastCurrent = 0.0;
  int ocm = analogRead(ciPinCurrent);

  float current = ocm * CURRENT_SCALE; // convert to amps.

  if (current > 0.1 || lastCurrent > 0.1) { // Threshold to reduce noise print
#if PRINT_MOVES > 1 // Make current printing higher verbosity
    SERIAL_OUT.print(label);
    SERIAL_OUT.print(" motor current: ");
    SERIAL_OUT.print(current * 1000);
    SERIAL_OUT.println(" mA");
#endif
    if (current > MAX_CURRENT) { // Uses private constant
      SERIAL_OUT.print("!!! OVER CURRENT on ");
      SERIAL_OUT.print(label);
      SERIAL_OUT.print(": ");
      SERIAL_OUT.print(current);
      SERIAL_OUT.println(" A !!!");
      // Consider a more robust safety stop, maybe disable motors via enable
      // pin?
      setTargetSpeed(0);
      mpwm = 0;    // Force PID output to zero
      applyPWM(0); // Force PWM off immediately
    }
  }
  lastCurrent = current;
}

void MotorControl::motionUpdate() {
  uint32_t currentTime = micros();

  int32_t currentCounter;
  uint32_t currentLastEncTime;

  noInterrupts(); // Protect reads of volatile variables shared with ISRs
  currentCounter = counter;         // Copy volatile counter
  currentLastEncTime = lastEncTime; // Copy volatile time
  interrupts();
  // Calculate speed based on counts over the motionUpdate interval
  float dtSpeedCalc = (currentTime - prevTimeForSpeed) * 1e-6f; // in seconds
  int32_t deltaCount = currentCounter - prevCounterForSpeed;

  if (dtSpeedCalc > 1e-6f) { // Avoid division by zero or tiny dt
    lastSpeed =
        (static_cast<float>(deltaCount) * METERS_PER_COUNT) / dtSpeedCalc;
  } else if (currentCounter == prevCounterForSpeed) {
    // If no time passed for robust calculation AND no counts,
    // rely on the timeout below to force zero if truly stopped.
  }

  // Robust zero-speed detection:
  // If counter hasn't changed since last motionUpdate AND >50ms since *any*
  // edge.
  if ((currentCounter == prevCounterForSpeed) &&
      (currentTime - currentLastEncTime > 50000)) {
    lastSpeed = 0.0f;
  }

  // Update history for the next speed calculation cycle
  prevCounterForSpeed = currentCounter;
  prevTimeForSpeed = currentTime;

  float err = targetSpeed - lastSpeed;
  mpwm = pid.update(err); // Get PID adjustment
  mpwm = constrain(mpwm, -1.0, 1.0);

  // Convert PID output (-1.0 to 1.0) to PWM value (-MAX_PWM to MAX_PWM)
  int setspeed = static_cast<int>(mpwm * MotorControl::MAX_PWM);

  // Stop motor if target speed is effectively zero
  if (abs(targetSpeed) < 1e-3) {
    mpwm = 0;
    setspeed = 0;
    pid.reset(); // Reset PID when stopped
    static uint32_t lastStopLog = 0;
    uint32_t now = micros();
    if (now - lastStopLog > 500000) { // Log at most twice per second
      if (PRINT_MOVES > 1) {
        SERIAL_OUT.print(label);
        SERIAL_OUT.println(" target ~0: stopping, PID reset");
      }
      lastStopLog = now;
    }
  }

  applyPWM(setspeed);

  // Debug output
  if (msgcnt++ > 100 &&
      abs(setspeed) > 0) { // Only print if moving and periodically
#if PRINT_MOVES
    char message[100]; // Smaller buffer
    snprintf(message, sizeof(message),
             "%c: Tgt=%.2f Cur=%.2f Err=%.2f PIDOut=%.2f PWM=%d", label,
             targetSpeed, lastSpeed, err, mpwm, setspeed);
    SERIAL_OUT.println(message);
#endif
    msgcnt = 0;
  }
}
