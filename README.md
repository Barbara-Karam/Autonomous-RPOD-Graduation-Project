# RPOD — Autonomous Rendezvous, Proximity Operations & Docking

An end-to-end, ground-based demonstrator of autonomous spacecraft **Rendezvous, Proximity Operations, and Docking (RPOD)**, built around a two-vehicle testbed: an autonomous **Chaser** and a controllable **Target**. The system combines GPU-accelerated computer vision, Model Predictive Control, and deterministic embedded firmware to detect, track, and physically dock with a target that may be either stationary or tumbling.

This repository contains the full avionics stack for the project: ROS 2 perception/GNC software, STM32 embedded firmware, and the ground-segment telemetry dashboard.

## Motivation

Real orbital RPOD missions rely on distributed, expensive, power-rich avionics. This project asks a harder question: how much of that capability can be reproduced on a **resource-constrained, single-board embedded platform**, while still meeting real-time perception and control deadlines? The result is a tightly integrated architecture that separates AI-heavy perception from hard-real-time actuation, connected by a custom fault-tolerant serial protocol — a pattern directly transferable to small-satellite and CubeSat avionics design.

## Key Features

- **Dual-mode pose estimation** — a YOLO-Pose deep learning model and a classical AprilTag fiducial detector both publish into the same pose topic, providing redundant, cross-checked target tracking.
- **Dual MPC control strategy** — a static-target rendezvous controller and an adaptive controller for a *tumbling* target, arbitrated at runtime by a supervisor node.
- **Sub-25 ms perception-to-actuation latency**, achieved through hardware-accelerated CSI camera capture and INT8 Tensor Core inference on the Jetson.
- **Deterministic motor control** — 100 Hz closed-loop PID velocity control on dedicated STM32 microcontrollers, fully decoupled from the non-real-time Linux/ROS 2 layer.
- **Custom fault-tolerant serial protocol** — DMA-backed, CRC-8-checked binary framing between the Jetson and STM32 controllers, with automatic error recovery.
- **Hardware-level safety interlocks** — limit-switch debouncing with immediate motor cutoff, latching E-stop, and voltage brownout protection, all independent of software/OS state.
- **Live PID tuning** over the same serial link, with firmware-side safety gates preventing gain changes while the robot is moving.
- **Real-time ground telemetry dashboard** for the Target spacecraft, built on a WebSocket relay and live charting.

## [System Architecture](https://drive.google.com/file/d/1i5ng6CL1H7LBbY_pDOo0vnm8feHESqVQ/view?usp=sharing))

The platform is split into two independent vehicles with very different computational demands.

```
                        ┌───────────────────────────────────────────┐
                        │              CHASER SPACECRAFT             │
                        │                                             │
  IMX708 Camera ───────►│  camera_node                                │
                        │      │                                      │
                        │      ├──► ai_subscriber (YOLO-Pose)          │
                        │      └──► cube_apriltag_node (AprilTag)      │
                        │                 │                            │
                        │                 ▼  cv/relative_pose          │
                        │      ┌──────────────────────────┐            │
                        │      │ modelv2_mpc_controller     │           │
                        │      │ (static target)            │──┐       │
                        │      └──────────────────────────┘  │        │
                        │      ┌──────────────────────────┐  ├──►     │
                        │      │ statespace_chaser_mpc      │  │ mpc_supervisor_node
                        │      │ (tumbling target)          │──┘       │
                        │      └──────────────────────────┘            │
                        │                 │  /wheel_angular_velocities │
                        │                 ▼                            │
                        │        speed_processing_node                 │
                        │                 │  /adjusted_rpms             │
                        │                 ▼                            │
                        │            uart_node ───────UART 921600───┐  │
                        │                                            │  │
                        │        arm_controller_node ───UART/USB────┤  │
                        │                                            │  │
                        │        communication_node ◄── all topics  │  │
                        │                 │ HTTP POST (1 Hz)         │  │
                        └─────────────────┼───────────────────────────┘
                                          │                            │
                                          ▼                       ┌────▼─────┐
                                Ground dashboard              STM32 Base +
                                                               Arms Controllers
```

```
                        ┌───────────────────────────────────────────┐
                        │              TARGET SPACECRAFT             │
                        │                                             │
                        │   ESP32-S3  ──► MPU6050 IMU                 │
                        │             ──► Quadrature Encoder          │
                        │             ──► DC Motor + Driver           │
                        │             ◄──► WebSocket (/esp32) ────────┼──► Node.js relay ──► Web dashboard (/ws)
                        └───────────────────────────────────────────┘
```

**Compute stratification:**

| Layer | Hardware | Responsibility |
|---|---|---|
| High-level autonomy | NVIDIA Jetson Orin Nano Super | Perception (YOLO-Pose, AprilTag), MPC guidance & control, telemetry aggregation |
| Deterministic actuation | STM32F401/F411 "Black Pill" ×2 | 100 Hz PID motor control, encoder decoding, limit-switch safety, IMU polling |
| Target vehicle | ESP32-S3 | Motor actuation, IMU/encoder telemetry, WebSocket command/telemetry link |

## Embedded Firmware — STM32 (Bare-Metal C / HAL)

This is the core hard-real-time layer of the project: two independent STM32F4 "Black Pill" microcontrollers (Base Controller on an STM32F401, Arms Controller migrated to an STM32F411CEU6) that isolate deterministic physical control from the non-real-time Jetson/ROS 2 layer. Everything here runs bare-metal, with no OS, and was built and debugged from the ground up — timer configuration, ISR-level control loops, and the entire inter-processor communication protocol.

**Base Controller — mobility & odometry**
- **Hardware quadrature decoding on 4 independent timers** (TIM2 and TIM5 as 32-bit, TIM3 and TIM4 as 16-bit), with explicit overflow casting to maintain continuous, zero-pulse-loss absolute encoder counts even at high wheel speed. Software-defined polarity matrices decouple physical wiring from the control math.
- **100 Hz closed-loop PID velocity control**, driven directly by a hardware timer interrupt (TIM9) rather than the main loop, for deterministic timing.
- **Conditional-integration anti-windup**: instead of a hard-clamped integrator, the controller checks on every tick whether the output is already saturated (±4199 PWM duty) *and* whether the pending integral term would push it further into saturation — only then is accumulation skipped. This preserves full control authority across the whole PWM range while still preventing windup, with a hard integrator ceiling kept only as a safety backstop.
- **Braking transient clamp + zero-target stall cutout**: a temporary, highly restrictive integrator ceiling is applied the instant a zero-setpoint is commanded, preventing the motor from winding up and reversing during deceleration; once measured velocity settles within a noise threshold for 50 consecutive ms, the firmware forcefully zeroes PWM output and resets PID state, eliminating the encoder-quantization limit cycle that would otherwise occur.
- **Live, in-situ PID tuning** over the same serial link (no reflashing required), gated by two safety rules enforced entirely in firmware: gains are only ever applied while the robot is provably stationary (loop disabled or all setpoints exactly zero), and every incoming gain is hard-clamped to a safe ceiling to protect against corrupted payloads or bad operator input.

**Arms Controller — docking manipulation & spatial sensing**
- **Slew-rate-limited PWM actuation** on two end-effector motors (TIM1), ramping duty cycle over time to protect the mechanical linkage from current spikes and jerk during docking, with a deadband filter to remove jitter near zero-actuation.
- **Hardware-debounced limit switches** via External Interrupts (EXTI) with a 20 ms software debounce, triggering an immediate `Motor_CoastAll()` hardware override on activation — physical safety that holds regardless of what the main loop or higher-level software is doing.
- **6-DoF IMU (MPU6050) polling with automatic bus recovery**: on an I²C read failure (sensor disconnect, EMI-induced bus hang), the bus is de-initialized and a 500 ms cooldown observed before re-init is attempted, so a single unresponsive sensor can never starve the real-time motor loop of CPU time.

**Shared design work across both controllers**
- **Custom fault-tolerant binary protocol**, hand-built and DMA-backed end-to-end: a 3-byte sync header (`0xAA 0x55 0xA5`) reduces false-frame-sync probability to roughly 1 in 16.7 million, every frame is CRC-8 checked (polynomial `0x07`) and any corrupted or oversized payload is discarded outright. Non-blocking reception (`HAL_UARTEx_ReceiveToIdle_DMA`) and transmission (`HAL_UART_Transmit_DMA`, mutex-protected against buffer corruption) keep the link fully asynchronous from the control loops.
- **UART fault recovery**: the HAL error callback was overridden to catch overrun/framing errors from electrically noisy wiring, immediately abort the corrupted reception, and reset the DMA idle-listening stream — so one bad byte can never permanently sever the link to the Jetson.
- **ISR-safe telemetry**: because the 100 Hz control ISR updates multi-byte floats (RPM, PWM duty) asynchronously from the transmit path, a dedicated critical-section routine (`__disable_irq()` / PRIMASK save, snapshot copy, re-enable) prevents the classic ARM float-tearing bug from corrupting telemetry.
- **Latching software E-stop**: an `MSG_ESTOP` command triggers an immediate `Motor_CoastAll()`, zeroes every active setpoint, and latches a fault flag in software — the system stays safe even after communication is restored, and clearing the stale setpoints guarantees it won't lurch back to a pre-halt velocity target on re-enable.
- A full rewrite/debug pass across `comm.c`, `control_loop.c`, `motor_control.c`, `pid.c`, and the ISR files, including fixing a sync-byte mismatch that was silently discarding every frame, resolving BTS7960 logic-voltage/TX-RX wiring faults, and correcting `TIM1` OSSR/OSSI settings that were causing partial simultaneous motor drive.

## ROS 2 / Jetson Software (Higher-Level Perception & GNC)

The Jetson side hosts the perception and guidance layer that feeds commands *into* the firmware above. It's built as an eight-package ROS 2 Humble workspace.

**Perception**
- `camera_node` — captures IMX708 frames via a GStreamer/`nvarguscamerasrc` hardware pipeline at 30 fps, republishing calibration data alongside every frame.
- `ai_subscriber` — YOLO-Pose keypoint detection → PnP-RANSAC pose recovery → Kalman filter + EMA smoothing, published to `cv/relative_pose`.
- `cube_apriltag_node` — deterministic, redundant pose source using AprilTag fiducials on the target cube, publishing to the same topic.

**Guidance & Control**
- `modelv2_mpc_controller` — Simulink-generated MPC for a static target.
- `statespace_chaser_mpc_controller` — Simulink-generated adaptive MPC tracking a live tumble angle for a rotating target.
- `mpc_supervisor_node` — arbitrates between the two controllers at a fixed 10 Hz, smoothing transitions on switch.
- `speed_processing_node` — converts arbitrated wheel commands from rad/s to RPM.

**Hardware Interface & Telemetry**
- `uart_node` — serial bridge to the Base Controller firmware above; also carries live PID tuning commands.
- `arm_controller_node` — serial bridge to the Arms Controller firmware; debounces limit-switch contact at the ROS 2 level and issues an arm E-stop on confirmed docking contact.
- `communication_node` — aggregates state from all subsystems (including a TF lookup for target pose, so it's agnostic to which perception node is active) and POSTs a JSON snapshot to the ground dashboard once per second, on a separate thread so a slow link can't stall the control graph.

## Target Spacecraft & Ground Segment
- **ESP32-S3 firmware** (Arduino framework) — streams IMU/encoder telemetry over WebSocket at 20 Hz, executes JSON telecommands, and fails safe (motor halt) if the link drops.
- **Node.js relay + web dashboard** — dual WebSocket paths for hardware (`/esp32`) and operators (`/ws`), state caching for instant reconnects, and a live Chart.js visualization of inertial data.

## Technologies & Tools

`STM32 HAL` (bare-metal C) · `STM32CubeIDE` · `STM32F401`/`STM32F411` · DMA UART / custom binary protocol · `ROS 2 Humble` · `Ubuntu 22.04` · `NVIDIA Jetson Orin Nano Super` (JetPack/L4T) · `TensorRT` · `YOLO-Pose` · `AprilTag` · `OpenCV` · `Simulink Embedded Coder` (generated MPC) · `ESP32-S3` / Arduino framework · `Node.js` WebSocket relay · `Chart.js`

## Repository Structure

```
├── firmware/
│   ├── base_controller/           # STM32F401 — mobility, PID, odometry (bare-metal C/HAL)
│   └── arms_controller/           # STM32F411 — docking arm, limit switches, IMU (bare-metal C/HAL)
├── ros2_ws/
│   ├── cam_publisher/             # camera_node
│   ├── modelv2_mpc_limited_ros2/  # static-target MPC
│   ├── statespace_chaser_mpc_ros2/ # tumbling-target adaptive MPC
│   ├── mpc_super_visor/           # controller arbitration
│   ├── motor_uart_pkg/            # uart_node (Base Controller bridge)
│   ├── rpod_arm_pkg/              # arm_controller_node
│   ├── speed_processing/          # rad/s -> RPM conversion
│   └── system_launch/             # combined launch files, remapping
├── target_firmware/               # ESP32-S3 Arduino sketch
├── ground_segment/                 # Node.js relay + web dashboard
└── rpod_start.sh                   # workspace bring-up / systemd launch script
```

## Design Decisions Worth Noting

- **Jetson Orin Nano Super over Jetson Nano / Raspberry Pi 5 / FPGA** — chosen specifically for its Tensor Cores, needed to hit sub-15 ms inference latency; alternatives lacked either AI acceleration or real-time feasibility within the project timeline.
- **STM32F4 over Arduino Mega, STM32F103, or Jetson-native GPIO** — the Cortex-M4F's hardware FPU and native 32-bit hardware quadrature timers give deterministic, zero-CPU-overhead encoder decoding and PID execution, immune to Linux scheduling jitter.
- **Two independent MPC controllers instead of one generalized controller** — a static-rendezvous case and a tumbling-target case have different reference dynamics; keeping them separate (with a lightweight supervisor) was simpler and safer than a single controller handling both regimes.
- **CP2102 USB bridge for the Arms Controller** — the Jetson's 40-pin header only exposes one hardware UART, which is reserved for the Base Controller; USB-serial was the pragmatic choice over reworking the interface to SPI/I²C.

## Future Work

- Additional sensor fusion (extra IMUs or LiDAR) for redundant state estimation.
- Redundant communication paths for improved fault tolerance.
- Extended-duration autonomous runs to validate long-term reliability.

## Author

**Barbara Karam Mansour** — Embedded Systems / Avionics
[GitHub](https://github.com/Barbara-Karam) · [LinkedIn](https://linkedin.com/in/barbara-ghaly)
