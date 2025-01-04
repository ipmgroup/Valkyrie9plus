# Valkyrie9plus
reworked library MotorWare 18

# Project Overview

## Key Updates

### 1. Enhanced Calculations
- Certain calculations have been converted to `float` to support devices with high current capabilities. For example, the **EPC9186** board can handle peak currents up to **330A**, which exceeds the limitations of the `iq24` format.

### 2. Added CAN Bus Support
- Implemented communication over **CAN bus** for robust and flexible system interaction.

### 3. DroneCAN Protocol Integration
- Support for the **DroneCAN** protocol has been added, enabling seamless communication with compatible devices in distributed systems.

### 4. Simplified Board Configuration
- All board settings have been centralized into a single location, making configuration and maintenance more straightforward.

### 5. Virtual Driver for DRV83xx
- A unified virtual driver has been implemented for all **DRV83xx** drivers, simplifying their integration and management.

### 6. Persistent User Settings
- Settings are now stored in **FLASH memory**, allowing:
  - Users to modify parameters via an interface.
  - New configurations to be saved for future use.

### 7. Easy Addition of New Boards
- New boards can be supported by creating an additional header file that describes their characteristics, **without modifying the core project**. This modular approach makes it easy to expand functionality and adapt to new hardware requirements.

### 8. Examples of Sensor and Controller Integration
- **Temperature Sensor:**
  - Example of connecting and processing data from a temperature sensor. The readings can be used for system monitoring or overheat protection.
- **Speed Controller:**
  - Example implementation of a motor speed controller using CAN bus or other interfaces. Supports both manual adjustments and dynamic management via the software interface.

### 9. Debug Information via CAN Bus
- Debug and controller state information is transmitted over **CAN bus** and can be viewed using **DroneCAN GUI**.
  - This provides convenient monitoring of system parameters such as sensor states, controller configurations, diagnostic messages, and more.

### 10. Added Sound Playback Capability
- Added the ability to play sound.
- To generate sound headers with notes, use a Python script for efficient and flexible sound integration.