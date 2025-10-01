# Hardware Setup Guide

This guide walks you through the complete hardware assembly and connection process for the PARTS Common Robotics Platform.

## Prerequisites

- All hardware components from the BOM
- Basic tools (screwdrivers, wire strippers, etc.)
- Multimeter for testing connections
- Clean, well-lit workspace

## Assembly Steps

### 1. Chassis Assembly

1. **Base Platform**
   - Attach the main chassis components
   - Install wheels and motors
   - Secure all mounting points

2. **Power System**
   - Install battery compartment
   - Connect power distribution board
   - Verify voltage levels

3. **Controller Installation**
   - Mount the main controller board
   - Connect power and ground connections
   - Install any expansion boards

### 2. Sensor Installation

1. **LiDAR Sensor**
   - Mount LiDAR on designated mounting point
   - Connect power and communication cables
   - Ensure clear 360° rotation

2. **Camera System**
   - Install camera mount
   - Connect camera to controller
   - Adjust camera angle and focus

3. **IMU Installation**
   - Mount IMU in stable location
   - Connect to I2C bus
   - Ensure minimal vibration

### 3. Wiring and Connections

1. **Power Connections**
   ```
   Battery → Power Board → Controller
   Battery → Motor Drivers
   Battery → Sensors (via regulators)
   ```

2. **Communication Connections**
   - USB connections for programming
   - Serial connections for sensors
   - I2C bus for IMU and other sensors

3. **Motor Connections**
   - Connect left motor to motor driver
   - Connect right motor to motor driver
   - Verify encoder connections

## Verification Steps

### Power System Check
1. Measure battery voltage (should be 12V nominal)
2. Check all power rails with multimeter
3. Verify no short circuits

### Communication Check
1. Test USB connection to computer
2. Verify serial communication with sensors
3. Check I2C bus functionality

### Mechanical Check
1. Ensure all screws are tight
2. Verify wheels rotate freely
3. Check sensor mounting stability

## Safety Considerations

⚠️ **Important Safety Notes:**

- Always disconnect power before making connections
- Double-check polarity on all connections
- Use appropriate wire gauges for current requirements
- Secure all cables to prevent damage during operation

## Troubleshooting

### Common Issues

**No Power**
- Check battery connections
- Verify power switch operation
- Test with multimeter

**Communication Errors**
- Check cable connections
- Verify baud rates match
- Test with known good cables

**Motor Issues**
- Verify motor driver connections
- Check encoder wiring
- Test motor operation individually

## Next Steps

After completing hardware setup:

1. Proceed to [Software Setup](software_setup.md)
2. Perform [Calibration](calibration.md) procedures
3. Run initial system tests

---

*For additional help, see [Troubleshooting](../troubleshooting/index.md) or contact the PARTS team.*
