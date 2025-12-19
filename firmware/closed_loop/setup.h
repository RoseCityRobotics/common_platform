#ifndef SETUP_H
#define SETUP_H

#define ROS 1 // 0 for Teensy stand alone, 1 for ROS based firmware
#define PRINT_MOVES 1

// ROS namespace - can be overridden at compile time via -DROS_NAME="namespace"
#ifndef ROS_NAME
#define ROS_NAME "rcr019"
#endif

#if ROS
#define SERIAL_OUT SerialUSB1
#else                     // !ROS
#define SERIAL_OUT Serial // Use default Serial for non-ROS
#endif

#endif
