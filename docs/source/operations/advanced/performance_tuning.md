# Performance Tuning Guide

This guide covers advanced performance tuning techniques for the RCR Common Robotics Platform, focusing on motor calibration for straight driving and PID tuning for optimal dynamic response.

## Table of Contents

1. [Motor Calibration for Straight Driving](#motor-calibration-for-straight-driving)
2. [PID Tuning for Dynamic Response](#pid-tuning-for-dynamic-response)
3. [Serial Commands for Real-time Tuning](#serial-commands-for-real-time-tuning)
4. [Troubleshooting Common Issues](#troubleshooting-common-issues)

## Motor Calibration for Straight Driving

### Overview

Motor calibration ensures that both wheels rotate at the same speed when given identical commands, enabling the robot to drive straight. This is critical for accurate navigation and odometry.

### Prerequisites

- Robot hardware assembled and functional
- `closed_loop.ino` firmware from the add_tuning_and_odom branch uploaded to the microcontroller (see [Firmware Updates](firmware_updates.md) for upload procedure)
- Serial monitor access (115200 baud) (see [Common Issues](common_issues.md) for debug output setup)
- Level surface for testing
- Measuring tape or ruler

### Calibration Process

#### 1. Initial Setup

See [Keyboard Teleoperation Setup](KeyboardTeleop.md).

#### 2. Basic Motor Test

```bash
# Enable motor drive (You may need to send the M character multiple times to enable the motors)
M

# Test individual motors
A0.5    # Left motor at 0.5 m/s
Z0.5    # Right motor at 0.5 m/s

# Stop motors
M
```

#### 3. Straight Line Test

1. **Mark Starting Position**
   - Place robot on level surface
   - Mark starting position with tape
   - Ensure robot is facing straight ahead

2. **Run Straight Test**
   ```bash
   # Reset odometry
   X

   # Drive forward at moderate speed
   ↑ (Up Arrow)
   ```

3. **Measure Deviation**
   - Measure how far the robot deviated from straight line
   - Record the deviation distance and direction

#### 4. Coarse-tuning Parameters

**Motor Scale Adjustment (Recommended First)**
```bash
# If robot drifts left, increase right motor scale
N1.1     # Increase right motor scale to 1.1

# If robot drifts right, increase left motor scale
B1.1     # Increase left motor scale to 1.1
```


## PID Tuning for Dynamic Response

### Overview

PID (Proportional-Integral-Derivative) controllers regulate motor speed to match target velocities. Proper tuning ensures:
- Fast response to speed changes
- Minimal overshoot
- Steady-state accuracy
- Stability under load

### PID Parameters

#### Proportional (P) Gain
- **Effect**: Immediate response to error
- **Too High**: Oscillations, instability
- **Too Low**: Slow response, steady-state error
- **Default**: 8.0

#### Integral (I) Gain
- **Effect**: Eliminates steady-state error
- **Too High**: Oscillations, windup
- **Too Low**: Persistent error
- **Default**: 0.8

#### Derivative (D) Gain
- **Effect**: Damping, reduces overshoot
- **Too High**: Noise sensitivity, instability
- **Too Low**: Overshoot, oscillations
- **Default**: 0.0 (disabled)

### Tuning Methodology

#### 1. Start with P-Only Control

```bash
# Reset PID parameters
X

# Set P gain only
Q5.0     # Left motor P=5.0
U5.0     # Right motor P=5.0
W0.0     # Disable left integral
O0.0     # Disable right integral
E0.0     # Disable left derivative
T0.0     # Disable right derivative
```

#### 2. Test Response

```bash
# Test step response
S0.5     # Test both motors at 0.5 m/s
```

**Observe:**
- Rise time (time to reach target)
- Overshoot (maximum error)
- Steady-state error
- Oscillations

#### 3. Adjust P Gain

**If too slow/steady-state error:**
```bash
Q6.0     # Increase left P gain
U6.0     # Increase right P gain
```

**If oscillating:**
```bash
Q4.0     # Decrease left P gain
U4.0     # Decrease right P gain
```

#### 4. Add Integral Control

Once P gain is stable:

```bash
W0.5     # Add small left I gain
O0.5     # Add small right I gain
```

**Test for:**
- Elimination of steady-state error
- No oscillations
- Smooth response

#### 5. Add Derivative Control (Optional)

For systems with overshoot:

```bash
E0.1     # Small left D gain
T0.1     # Small right D gain
```

**Monitor for:**
- Reduced overshoot
- No noise amplification
- Stable response

### Advanced Tuning Techniques

#### 1. Ziegler-Nichols Method

1. **Find Ultimate Gain (Ku)**
   - Start with P=1.0, I=0, D=0
   - Increase P until sustained oscillations
   - Record Ku and oscillation period Tu

2. **Calculate Parameters**
   - P = 0.6 × Ku
   - I = 2 × P / Tu
   - D = P × Tu / 8

#### 2. Load-Dependent Tuning

Test under different load conditions:

```bash
# Light load (no payload)
F1.0

# Heavy load (with payload)
F1.0
```

Adjust gains for consistent performance across load ranges.

#### 3. Speed-Dependent Tuning

Test at different speeds:

```bash
# Low speed
F0.2

# Medium speed
F0.5

# High speed
F1.0
```

### Performance Metrics

#### 1. Response Time
- **Target**: < 0.5 seconds to reach 95% of target speed
- **Measurement**: Monitor speed feedback via serial output

#### 2. Steady-State Error
- **Target**: < 2% of target speed
- **Measurement**: Compare target vs actual speed

#### 3. Overshoot
- **Target**: < 10% of target speed
- **Measurement**: Peak speed during step response

#### 4. Settling Time
- **Target**: < 1 second to settle within 5% of target
- **Measurement**: Time to reach and stay within error band

## Serial Commands for Real-time Tuning

The `closed_loop.ino` firmware provides serial commands for real-time parameter adjustment:

### Basic Commands

| Command | Description | Example |
|---------|-------------|---------|
| `M` | Toggle motor drive on/off | `M` |
| `X` | Reset all state | `X` |
| `H` | Show help with all commands | `H` |

### Motor Testing Commands

| Command | Description | Example |
|---------|-------------|---------|
| `S<speed>` | Test both motors at same speed | `S0.5` |
| `A<speed>` | Test left motor only | `A0.5` |
| `Z<speed>` | Test right motor only | `Z0.5` |

### Motor Scale Commands

| Command | Description | Example |
|---------|-------------|---------|
| `B<scale>` | Set left motor scale (0.5-2.0) | `B1.1` |
| `N<scale>` | Set right motor scale (0.5-2.0) | `N0.9` |

### PID Tuning Commands

| Command | Description | Example |
|---------|-------------|---------|
| `Q<value>` | Set left motor PID P | `Q8.0` |
| `W<value>` | Set left motor PID I | `W0.8` |
| `E<value>` | Set left motor PID D | `E0.1` |
| `U<value>` | Set right motor PID P | `U8.0` |
| `O<value>` | Set right motor PID I | `O0.8` |
| `T<value>` | Set right motor PID D | `T0.1` |

### Parameter Management Commands

| Command | Description | Example |
|---------|-------------|---------|
| `Y` | Save tuning to EEPROM | `Y` |
| `G` | Load tuning from EEPROM | `G` |
| `C` | Reset tuning to defaults | `C` |

### Monitoring Commands

Enable debug output to monitor performance:

```cpp
// In the setup.h header for closed_loop.ino, set debug levels:
#define PRINT_MOVES 2    // Enable status updates
#define PRINT_PID 1      // Enable PID debug output
```

### Adjust the maximum allowed speed

```cpp
// In the motor control header MotorControl.h, set the maximum PWM allowed:
static constexpr int MAX_PWM = 120; // Limit PWM magnitude (0-255) on or around line 46
```

Recompile the firmware and upload after changing PRINT_MOVES or MAX_PWM (see [Firmware Updates](firmware_updates.md) for procedure).

###

## Troubleshooting Common Issues

### 1. Robot Won't Drive Straight

**Symptoms:**
- Consistent drift in one direction
- Uneven wheel speeds

**Solutions:**
- Verify encoder connections
- Calibrate individual motor PWM scaling factors
- Check for mechanical binding

### 2. Oscillations/Instability

**Symptoms:**
- Motor speed oscillates around target
- Robot shakes or vibrates

**Solutions:**
- Reduce P gain
- Add derivative control
- Check for mechanical issues
- Verify encoder noise

### 3. Slow Response

**Symptoms:**
- Takes too long to reach target speed
- Sluggish acceleration

**Solutions:**
- Increase P gain
- Add integral control
- Check for mechanical friction
- Verify power supply

### 4. Steady-State Error

**Symptoms:**
- Robot doesn't reach target speed
- Persistent speed error

**Solutions:**
- Increase I gain
- Check for load variations
- Verify target speed calculations
- Check for mechanical binding

### 5. Noise Sensitivity

**Symptoms:**
- Erratic behavior with small changes
- Sensitive to disturbances

**Solutions:**
- Reduce D gain
- Add filtering
- Check encoder connections
- Improve mechanical isolation

### 6. Motor Stall

**Symptoms:**
- High-pitched whining or squealing sound from motors
- Motors not responding to commands
- Can cause motors to heat up due to current not producing useful work

**Solutions:**
- P gain too high causing oscillations, reduce P gain
- Mechanical binding or obstruction, inspect wheels
- Insufficient power supply, check battery voltage, charge if necessary
- Inadequate PWM scaling, adjust calibration
