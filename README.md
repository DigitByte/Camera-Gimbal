# Camera Gimbal (Multi-Axis Actuator Reference)

Application repository for a multi-axis gimbal built on actuator control and IMU feedback. It acts as a practical reference for stabilizing mechanisms within the actuator component library ecosystem.

## Purpose

This project provides reusable examples for:
- Multi-axis actuator coordination.
- IMU-assisted control loops for stabilization.
- CAD components for gimbal structural assemblies.

## Repository Structure

```text
camera-gimbal/
├── docs/
│   └── repository-map.md
├── mechanical/
│   └── cad/
│       └── model/
├── electronics/
│   ├── firmware/
│   └── hardware/
└── software/
    └── python/
        └── code/
            ├── iphone-imu.py
            └── camera-gimbal-code/
```

## Component Coverage

### Mechanical
- `mechanical/cad/model/`: STL files for gimbal links, camera mount, motor mounts, and board base.

### Software
- `software/python/code/camera-gimbal-code/`: control scripts, IMU utility code, and calibration data.
- `software/python/code/iphone-imu.py`: additional IMU data utility.

## Role in Actuator Library

This repo is an **application testbed** for actuator behavior under orientation control and sensor feedback.
