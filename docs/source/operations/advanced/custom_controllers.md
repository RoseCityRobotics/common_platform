# Custom Controllers

This guide covers advanced controller configuration and customization for the PARTS Common Robotics Platform.

## Overview

Custom controllers allow you to implement advanced control algorithms, optimize performance, and add new functionality to your robot.

## Prerequisites

- Understanding of control theory
- Familiarity with ROS2 control
- Knowledge of robot kinematics
- Experience with C++ or Python

## Controller Types

### 1. Motion Controllers

**Differential Drive Controller:**
- Controls left and right wheels
- Implements velocity commands
- Handles odometry feedback

**Omni-Drive Controller:**
- Controls multiple wheels
- Implements holonomic motion
- Advanced kinematics

### 2. Sensor Controllers

**LiDAR Controller:**
- Manages LiDAR data
- Implements filtering
- Handles calibration

**Camera Controller:**
- Manages camera data
- Implements image processing
- Handles calibration

## Controller Development

### 1. Basic Controller Structure

**C++ Controller:**
```cpp
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

class CustomController : public controller_interface::ControllerInterface
{
public:
  CustomController();
  ~CustomController();

  controller_interface::return_type init(
    const std::string & controller_name) override;

  controller_interface::return_type update() override;

private:
  // Controller implementation
};
```

**Python Controller:**
```python
import rclpy
from controller_interface import ControllerInterface

class CustomController(ControllerInterface):
    def __init__(self):
        super().__init__()
    
    def init(self, controller_name):
        # Initialize controller
        pass
    
    def update(self):
        # Update controller
        pass
```

### 2. Controller Configuration

**Controller Parameters:**
```yaml
custom_controller:
  ros__parameters:
    # Controller parameters
    update_rate: 100.0
    control_mode: velocity
    
    # PID parameters
    p_gain: 1.0
    i_gain: 0.0
    d_gain: 0.0
    
    # Safety parameters
    max_velocity: 1.0
    max_acceleration: 2.0
```

### 3. Controller Integration

**Hardware Interface:**
```yaml
hardware:
  ros__parameters:
    joints:
      - left_wheel
      - right_wheel
    
    interfaces:
      - position
      - velocity
      - effort
```

**Controller Manager:**
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    
    controllers:
      custom_controller:
        type: custom_controller/CustomController
```

## Advanced Control Algorithms

### 1. PID Control

**PID Implementation:**
```cpp
class PIDController
{
public:
  PIDController(double p, double i, double d);
  double update(double error, double dt);
  
private:
  double p_gain_, i_gain_, d_gain_;
  double integral_, previous_error_;
};
```

**PID Tuning:**
- Start with P gain only
- Add I gain for steady-state error
- Add D gain for stability
- Tune for performance

### 2. Model Predictive Control

**MPC Implementation:**
- Predict future behavior
- Optimize control inputs
- Handle constraints
- Real-time optimization

### 3. Adaptive Control

**Adaptive Algorithms:**
- Self-tuning controllers
- Parameter estimation
- Robust control
- Learning control

## Performance Optimization

### 1. Real-time Performance

**Timing Requirements:**
- Control loop frequency
- Latency constraints
- Jitter minimization
- Priority scheduling

**Optimization Techniques:**
- Efficient algorithms
- Memory management
- CPU optimization
- Real-time scheduling

### 2. Control Performance

**Performance Metrics:**
- Tracking accuracy
- Response time
- Stability margins
- Robustness

**Optimization Methods:**
- Parameter tuning
- Algorithm selection
- Hardware optimization
- System integration

## Testing and Validation

### 1. Simulation Testing

**Gazebo Simulation:**
```bash
# Launch simulation
ros2 launch common_platform launch_sim.launch.py

# Test controller
ros2 run custom_controller test_controller
```

**Controller Testing:**
- Unit tests
- Integration tests
- Performance tests
- Safety tests

### 2. Hardware Testing

**Safe Testing:**
- Start with low gains
- Test in safe environment
- Monitor system behavior
- Have emergency stop ready

**Validation Procedures:**
- Step response tests
- Frequency response tests
- Stability tests
- Performance tests

## Troubleshooting

### Common Issues

**Controller Not Starting:**
- Check configuration
- Verify hardware interface
- Check for errors
- Review logs

**Poor Performance:**
- Tune parameters
- Check sensor data
- Verify timing
- Review algorithms

**Instability:**
- Reduce gains
- Check for delays
- Verify system model
- Review stability

### Diagnostic Tools

**Controller Monitoring:**
```bash
# Monitor controller status
ros2 control list_controllers

# Check controller parameters
ros2 param list /controller_manager

# Monitor performance
ros2 run rqt_plot rqt_plot
```

## Best Practices

### Development

**Code Quality:**
- Follow coding standards
- Use version control
- Document code
- Test thoroughly

**Safety:**
- Implement safety checks
- Use appropriate limits
- Handle failures gracefully
- Test extensively

### Deployment

**Configuration Management:**
- Version control configurations
- Document changes
- Test in simulation first
- Deploy gradually

**Monitoring:**
- Monitor performance
- Check for errors
- Log important events
- Maintain backups

---

*For firmware updates, see [Firmware Updates](firmware_updates.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
