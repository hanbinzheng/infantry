# Infantry Robot

> Electrical control code for RMUL 2026

## Architecture

- **Application Layer**: High-level task management and logic for robot subsystems.
- **Controller/Interface Layer**: Abstraction of control laws logic and device-specific command interfaces
- **Algorithm Layer**: Mathematical foundations including kinematics and state estimation.
- **Device & BSP Layer**: Hardware abstraction and low-level peripheral drivers.

---

## Project Structure

```text
├── Application/      # Logic-level task execution (1000Hz)
│   ├── body            # Chassis control logic (body_task)
│   ├── head            # Pitch, friction wheels, and trigger logic (head_task)
│   ├── neck            # Yaw-axis gimbal control (neck_task)
│   └── controller      # Abstracted set_target/velocity functions
├── Algorithm/        # Core mathematical implementations
│   ├── kinematics      # Omni-directional chassis kinematics
│   ├── mahony          # Mahony filter for sensor fusion
│   ├── quaternion      # Quaternion-based calculation
│   └── pid             # PID control algorithms
├── Device/           # Hardware component drivers
│   ├── motor           # motor can communication & encapsulation
│   ├── imu             # IMU data acquisition
│   └── dbus            # Remote control receiver (DBUS) protocol
└── BSP/              # Low-level hardware abstraction
    ├── bsp_fdcan       # FDCAN configurations
    ├── bsp_spi         # SPI for IMU communication
    ├── bsp_tim         # Timers for high-frequency control loops
    ├── bsp_usart       # Serial communication
    └── bsp_gpio        # GPIO configurations
```

---

## Control Loops

The system utilizes hardware timer interrupts to ensure strict execution frequency for control stability:

| Task             | Frequency | Timer    | Functionality                                                |
| :--------------- | :-------- | :------- | :----------------------------------------------------------- |
| **imu_update()** | 1000 Hz   | `htim4`  | attitude sampling & Mahony filtering, in **Device/Src/imu.c** |
| **neck_task()**  | 1000 Hz   | `htim5`  | yaw gimbal control, in **Application/Src/neck.c**            |
| **head_task()**  | 1000 Hz   | `htim12` | pitch gimbal, trigger, and friction wheel, in **Application/Src/head.c** |
| **body_task()**  | 125 Hz    | `htim15` | chassis control, in **Application/Src/body.c**               |

---

## Build & Requirements

- **Toolchain**: Arm GNU Toolchain (GCC)

- **Build System**: Makefile

- **Hardware**: STM32H723VGT6 (damiao-mc02)

- **Framework**: STM32Cube HAL Package

