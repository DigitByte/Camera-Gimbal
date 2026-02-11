import myactuator_rmd_py as rmd
import time
import numpy as np

driver = rmd.CanDriver("can1")
node_id = 3
actuator = rmd.ActuatorInterface(driver, node_id)

try:
    start_time = time.time()  # Record the start timea



    actuator.setAcceleration(0, rmd.actuator_state.AccelerationType.POSITION_PLANNING_ACCELERATION)
    actuator.setAcceleration(0, rmd.actuator_state.AccelerationType.POSITION_PLANNING_DECELERATION)

  
    initial_angle = actuator.getMultiTurnAngle() 

    print('Setting Initial Position')
    time.sleep(3)

    traj_angle = 45
    while True:
        
        angle = traj_angle + initial_angle

        # Send the calculated sine wave angle as an absolute setpoint
        actuator.sendPositionAbsoluteSetpoint(angle, 15.0)
        
        
        # actuator.sendVelocitySetpoint(angle)
        # output = actuator.sendCurrentSetpoint(angle)
        # Print the angle sent to the actuator
        print(f"Sent angle: {traj_angle:.2f} degrees")
        
     
        current_angle = actuator.getMultiTurnAngle() - initial_angle
        print(f"Current actuator angle: {current_angle}")
        time.sleep(0.1)

    
except KeyboardInterrupt:
    print("Ctrl+C detected")
    # Shutdown the motor
    actuator.shutdownMotor()


