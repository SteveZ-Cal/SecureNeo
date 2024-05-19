# Secure NEORV32 SoC Multitasking

This repository contains the implementation of the Secure NEORV32 System-on-Chip (SoC) Multitasking project, aimed at augmenting security in multitask execution scenarios on the NEORV32 SoC platform. It achieves this by customizing the Custom Functions Subsystem (CFS) module and leveraging Zephyr OS scheduling mechanisms.

## Overview

The Secure NEORV32 SoC Multitasking project provides a comprehensive solution for improving security and reliability in embedded systems. It addresses potential security concerns through the integration of both software and hardware components. The project consists of three main components:

- **Trusted Hardware**: Implemented within the NEORV32 RTL/core files, Trusted Hardware serves as the firmware customization for the NEORV32 processor. It ensures the integrity of the hardware and facilitates secure booting processes. Additionally, it collaborates with Trusted Software and Untrusted Main for context switching, contributing to system resilience.
- **Trusted Software**: Functioning as the central controller within the NEORV32 software environment, Trusted Software orchestrates secure booting for the Zephyr RTOS and handles interrupt events. It undergoes initialization, interrupt handling, and error management phases, collaborating closely with Trusted Hardware for context switching and system integrity verification.
- **Untrusted Main**: Operating within a multi-threaded environment, Untrusted Main hosts the Zephyr RTOS and user-defined tasks. It interacts with Trusted Software for system initialization and handles interruptions by engaging Trusted Software's interrupt handling mechanism. Additionally, Untrusted Main interfaces with Trusted Hardware for context switching, ensuring efficient task management and system stability.

## Features

- Secure multitasking execution on the NEORV32 SoC platform
- Customized CFS module for enhanced access controls, data encryption, and integrity checks
- Zephyr OS integration for efficient task scheduling and management
- Secure booting and interrupt handling mechanisms
- System integrity verification through hybrid attestation

## Requirements

- **NEORV32 RISC-V Processor (version 1.8.6)**:
  The NEORV32 RISC-V Processor is a customizable and open-source RISC-V processor core designed for FPGA implementations. Version 1.8.6 (or compatible) is required for compatibility with the Secure NEORV32 SoC Multitasking project. This processor core serves as the foundation for the project's hardware components, facilitating secure booting processes and system integrity verification.

- **Zephyr RTOS (version 3.4.99)**:
  Zephyr is a scalable real-time operating system (RTOS) designed for resource-constrained embedded devices. Version 3.4.99 (or compatible) is required for integration with the Secure NEORV32 SoC Multitasking project. Zephyr provides efficient task scheduling and management capabilities, enabling secure multitasking execution on the NEORV32 SoC platform.

- **Intel Quartus Prime Lite Edition Design Software (version 22.1 for Windows)**:
  Intel Quartus Prime Lite Edition is a development software suite used for designing, simulating, and programming FPGA-based systems. Version 22.1 (or compatible) is required for compiling and synthesizing the Secure NEORV32 SoC Multitasking project's hardware components for deployment on the Cyclone V FPGA. This software suite provides the necessary tools for FPGA configuration and debugging.

- **Cyclone V 5CEBA4F23C7 FPGA**:
  The Cyclone V 5CEBA4F23C7 FPGA is a field-programmable gate array (FPGA) manufactured by Intel (formerly Altera). It serves as the target hardware platform for deploying the Secure NEORV32 SoC Multitasking project. The FPGA's reconfigurable nature allows for flexible customization and integration of hardware components, enabling secure multitasking execution in embedded systems.

- **TeraTerm terminal program**:
  TeraTerm is a free and open-source terminal emulator program used for serial communication with embedded systems. It provides a user-friendly interface for interacting with the Secure NEORV32 SoC Multitasking project during development and debugging. TeraTerm facilitates communication with the NEORV32 SoC platform, allowing users to monitor system behavior and output diagnostic information.


# Getting Started with Secure NEORV32 SoC Multitasking Project

## 1. Set up FPGA Development Environment

1. **Install Intel Quartus Prime Lite Edition Design Software**
   - Download and install Intel Quartus Prime Lite Edition Design Software (version 22.1 for Windows).
   
2. **Connect the FPGA Board**
   - Connect the Cyclone V 5CEBA4F23C7 FPGA board to your computer.
   
3. **Configure FPGA**
   - Refer to the NEORV32 official user guide (section 2, "General Hardware Setup") to configure the FPGA top file and set up the required parameters (e.g., clock frequency, memory sizes).
   
4. **Install and Configure TeraTerm**
   - Install and configure TeraTerm terminal program for communicating with the FPGA via UART.

## 2. Set up Zephyr RTOS

1. **Install Zephyr RTOS**
   - Install Zephyr RTOS (version 3.4.99) and the zephyr-sdk-0.16.1 toolchain.

2. **Development Environment Setup**
   - Follow the Zephyr documentation to set up the development environment and configure the necessary tools.

## 3. Clone the Secure NEORV32 SoC Multitasking Repository

Clone the repository:

```bash
git clone https://github.com/your-username/secure-neorv32-soc-multitasking.git
```

This repository includes the NEORV32 RISC-V Processor (version 1.8.6), compatible with the Secure NEORV32 SoC Multitasking project.

Follow the instructions in the repository's documentation to build and integrate the Trusted Hardware, Trusted Software, and Untrusted Main components with the FPGA and Zephyr RTOS.

## License

This project is licensed under the [MIT License](LICENSE).

## Acknowledgments

- **NEORV32 RISC-V Processor**: The NEORV32 RISC-V Processor serves as the foundation for the Secure NEORV32 SoC Multitasking project, providing a customizable and extensible architecture.
- **Zephyr Project RTOS**: The Zephyr Project RTOS plays a crucial role in the project by providing a robust and scalable real-time operating system environment.