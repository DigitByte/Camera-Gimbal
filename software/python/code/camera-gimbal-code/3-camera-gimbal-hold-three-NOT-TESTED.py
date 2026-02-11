import myactuator_rmd_py as rmd
import time
import numpy as np

driver = rmd.CanDriver("can1")
actuators = [rmd.ActuatorInterface(driver, i) for i in [1, 2, 3]]
initial_angles = []

try:
    # Set acceleration/deceleration for all actuators
    for actuator in actuators:
        actuator.setAcceleration(0, rmd.actuator_state.AccelerationType.POSITION_PLANNING_ACCELERATION)
        actuator.setAcceleration(0, rmd.actuator_state.AccelerationType.POSITION_PLANNING_DECELERATION)

    # Record initial angles
    initial_angles = [a.getMultiTurnAngle() for a in actuators]

    print('Setting Initial Position')
    time.sleep(3)

    traj_angle = 0
    while True:
        for actuator, init_angle in zip(actuators, initial_angles):
            target_angle = traj_angle + init_angle
            actuator.sendPositionAbsoluteSetpoint(target_angle, 15.0)

        for i, (actuator, init_angle) in enumerate(zip(actuators, initial_angles)):
            current_angle = actuator.getMultiTurnAngle() - init_angle
            print(f"Motor {i+1} angle: {current_angle:.2f} degrees")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Ctrl+C detected")
    for actuator in actuators:
        actuator.shutdownMotor()
