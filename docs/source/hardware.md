# Hardware Guide

#### 1. Solder Your PARTS CRP Board
Soldering the PARTS CRP board is the first crucial step. Follow the instructions carefully to ensure a successful setup.

- **Interactive BOM**: Utilize the [Interactive BOM](https://htmlpreview.github.io/?https://github.com/rosecityrobotics/common_platform/blob/master/hardware/romi_board/bom/ibom.html) for an easier soldering process.
- **Headers for Connectivity**: We recommend using headers for connections between boards. This makes disassembly possible if needed.
  - Use female (socket) headers on the bottom board.
  - Use male (pin) headers on the top board. These are typically included with all breakout boards.
- **MOSFET Orientation**: Ensure the correct orientation of the MOSFETs during soldering.
- **Resistor for Battery Voltage Divider**: Choose the resistor based on your battery choice:
  - Use a 2M ohm resistor (R1) for 6 AA batteries.
  - Opt for a 3M ohm resistor if using higher voltage batteries like 2 LiPo batteries.
  - Install a >100 pf capacitor across R2 to address voltage drop issues.
- **Teensy Board Modification**: For battery power, cut the trace on the Teensy board. Refer to the note "Cut to separate VIN from VUSB" on the [Teensy 4.0 Back Side](https://github.com/rosecityrobotics/common_platform/blob/master/hardware/main_schematic.pdf) image.
- **Resistor Values**: The values of R4, R5, R8, R9 depend on your specific usage. For I2C, 4.7k is appropriate.
- **Schematic**: Refer to the [Main Schematic](https://github.com/rosecityrobotics/common_platform/blob/master/hardware/main_schematic.pdf) for detailed understanding.

#### 2. Solder Accessories
Solder accessories like the Inertial Measurement Unit (IMU), ensuring proper orientation and connection.

#### 3. Assemble the Pololu Romi Chassis
The Pololu Romi Chassis forms the physical structure of your robot.

- Follow the [Romi Chassis Assembly Guide](https://www.pololu.com/docs/0J68/4) for detailed instructions on assembling the chassis.
