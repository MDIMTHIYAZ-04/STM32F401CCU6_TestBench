# STM32F401CCU6 Embedded Test Platform

## Overview

This repository contains firmware, testing modules, communication protocols, and validation files developed for the **STM32F401CCU6 Blackpill board**.

The project is focused on embedded system testing, hardware validation, RS485 communication, Modbus protocol integration, sensor interfacing, and firmware debugging.

---

# Project Purpose

This repository is used to:

* Develop and test STM32F401CCU6 firmware
* Validate hardware modules and external sensors
* Perform RS485 and Modbus communication testing
* Build reusable embedded testing frameworks
* Store firmware versions and validation builds
* Organize industrial embedded experiments and board testing

---

# Hardware Used

* STM32F401CCU6 Blackpill Board
* ST-Link Programmer
* RS485 Communication Module
* Sensors (Air Sensor / Analog / Digital)
* Power Supply Module
* Connector Test Board
* UART Communication Interface

---

# Software & Tools

* Arduino IDE
* STM32duino Core
* Embedded C / C++
* GitHub Version Control
* ST-Link Programmer
* Serial Monitor / UART Debugging
* STM32 Board Package for Arduino IDE

---

# Repository Structure

```text
STM32F401CCU6_Embedded_TestPlatform/
│
├── AIR_SENSOR_WORKING_TESTING/
├── CODE_UPLOAD_TEST_WITH_STLINK/
├── FULLSETUP_TESTING_V1/
├── FULLSETUP_TESTING_V3/
├── FULLSETUP_TESTING_V4/
├── FULL_SETUP_TESTING_V2/
├── FULL_SETUP_TESTING_V5/
├── RS485_TESTING/
├── ERROR_HANDLING_OPTIMIZED_FINAL.ino
├── MODBUS_POLL.mbp
├── SENSOR_OUTPUT_SIMILATION_DEMO.html
└── README.md
```

---

# Features

* STM32 firmware development
* Sensor reading and validation
* RS485 communication testing
* Modbus RTU protocol implementation
* Embedded debugging support
* Hardware validation workflows
* Multi-version firmware testing

---

# Communication Protocols

### UART

Used for debugging and serial communication.

### RS485

Used for industrial communication testing.

### Modbus RTU

Used for device polling and data transfer.

---

# Development Workflow

1. Create firmware project
2. Configure peripherals
3. Upload code using ST-Link
4. Test hardware response
5. Validate communication
6. Optimize firmware
7. Push updates to GitHub

---

# Future Improvements

* Add FreeRTOS integration
* Add industrial dashboard monitoring
* Add IoT cloud connectivity
* Add data logging support
* Improve error handling system

---

# Author

**Imthiyaz**
Embedded Engineer | STM32 Developer | Industrial Automation

---

# License

This repository is for educational, development, and industrial testing purposes.

---

# Notes

This project is continuously updated with new firmware versions, testing methods, and embedded communication experiments.
