import os
import sys
import time
import smbus
import numpy as np
from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman 
import myactuator_rmd_py as rmd

def angle_error(setpoint, measurement):
    return (setpoint - measurement + 180) % 360 - 180

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

address = 0x68
bus = smbus.SMBus(5)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
dirName = os.path.dirname(__file__)
calib_file_path = os.path.join(dirName,'calib','calib.json')
imu.loadCalibDataFromFile(calib_file_path)
sensorfusion = kalman.Kalman()
imu.readSensor()
imu.computeOrientation()
sensorfusion.roll = imu.roll
sensorfusion.pitch = imu.pitch
sensorfusion.yaw = imu.yaw
currTime = time.time()

pid_3_roll = PID(Kp=5, Ki=0.0, Kd=0)
pid_2_pitch = PID(Kp=2, Ki=0.0, Kd=0)
pid_1_yaw = PID(Kp=2, Ki=0.0, Kd=0)

# These are the desired IMU angles 
setpoint_3_roll = 180.0 # rotation about y axis - short tilt 
setpoint_2_pitch = 0.0  # rotation about about x axis - long tilt 
setpoint_1_yaw = 96.0 # rotation about axis in/out of imu board z axis

# Setup actuators 
use_3_roll = True
use_1_yaw = True
use_2_pitch = True

# Hold Angles Degrees
hold_angle_1_yaw = 0
hold_angle_2_pitch = 90
hold_angle_3_roll = 45

# Max Speed 
max_speed = 15.0


print('Setting Initial Position - Hold Motor In Position')
time.sleep(3)
driver = rmd.CanDriver("can1")

if use_1_yaw: 
    actuator_1_yaw = rmd.ActuatorInterface(driver, 1)
    actuator_1_yaw.setAcceleration(0, rmd.actuator_state.AccelerationType.POSITION_PLANNING_ACCELERATION)
    actuator_1_yaw.setAcceleration(0, rmd.actuator_state.AccelerationType.POSITION_PLANNING_DECELERATION)
    initial_angle_1_yaw = actuator_1_yaw.getMultiTurnAngle() 

if use_2_pitch: 
    actuator_2_pitch = rmd.ActuatorInterface(driver, 2)
    actuator_2_pitch.setAcceleration(0, rmd.actuator_state.AccelerationType.POSITION_PLANNING_ACCELERATION)
    actuator_2_pitch.setAcceleration(0, rmd.actuator_state.AccelerationType.POSITION_PLANNING_DECELERATION)
    initial_angle_2_pitch = actuator_2_pitch.getMultiTurnAngle() 

if use_3_roll: 
    actuator_3_roll = rmd.ActuatorInterface(driver, 3)
    actuator_3_roll.setAcceleration(0, rmd.actuator_state.AccelerationType.POSITION_PLANNING_ACCELERATION)
    actuator_3_roll.setAcceleration(0, rmd.actuator_state.AccelerationType.POSITION_PLANNING_DECELERATION)
    initial_angle_3_roll = actuator_3_roll.getMultiTurnAngle() 

print('Okay Let Go')
time.sleep(5)

start_home = time.time()

try:
    while True:
        # Read IMU Data
        imu.readSensor()
        imu.computeOrientation()
        newTime = time.time()
        dt = newTime - currTime
        currTime = newTime
        sensorfusion.computeAndUpdateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2],\
                                                    imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)

        print("Kalmanroll:{0} KalmanPitch:{1} KalmanYaw:{2} ".format(sensorfusion.roll, sensorfusion.pitch, sensorfusion.yaw))
        error_3_roll = (setpoint_3_roll - sensorfusion.roll + 180) % 360 - 180
        error_2_pitch = (setpoint_2_pitch - sensorfusion.pitch + 180) % 360 - 180
        error_1_yaw = (setpoint_1_yaw - sensorfusion.yaw + 180) % 360 - 180
        print('+++error_3_roll', error_3_roll) 
        # print('error_2_pitch', error_2_pitch) 
        print('---error_1_yaw', error_1_yaw) 

        output_3_roll = pid_3_roll.update(error_3_roll, dt)
        output_2_pitch = pid_2_pitch.update(error_2_pitch, dt)
        output_1_yaw = pid_1_yaw.update(error_1_yaw, dt)
        print('+++output_3_roll', output_3_roll)
        # print('output_2_pitch', output_2_pitch)
        print('---output_1_yaw', output_1_yaw)

        # Correct Current Angles (with Offset from Initial Angles)
        current_angle_1_yaw = actuator_1_yaw.getMultiTurnAngle() - initial_angle_1_yaw
        current_angle_2_pitch = actuator_2_pitch.getMultiTurnAngle() - initial_angle_2_pitch
        current_angle_3_roll = actuator_3_roll.getMultiTurnAngle() - initial_angle_3_roll
        
        # Software Limits 
        if abs(current_angle_1_yaw) > 90 or abs(current_angle_2_pitch) > 135 or abs(current_angle_3_roll) > 130: 
            print('exceed angle limits')
            actuator_1_yaw.shutdownMotor()
            actuator_2_pitch.shutdownMotor()        
            actuator_3_roll.shutdownMotor()
            break
                    
        if time.time() > start_home + 10:
            print('Start Tracking')
            imu_angle_1_yaw = -output_1_yaw*0
            imu_angle_2_pitch = -output_2_pitch*0
            imu_angle_3_roll = output_3_roll*0
            max_speed = 100
        else: 
            imu_angle_1_yaw = 0
            imu_angle_2_pitch = 0
            imu_angle_3_roll = 0
        
        send_angle_1_yaw = hold_angle_1_yaw + initial_angle_1_yaw + imu_angle_1_yaw
        send_angle_2_pitch = hold_angle_2_pitch + initial_angle_2_pitch + imu_angle_2_pitch
        send_angle_3_roll = hold_angle_3_roll + initial_angle_3_roll + imu_angle_3_roll
        
        print(f"--current yaw angle (measured angle): {current_angle_1_yaw}")
        print('---imu_angle_1_yaw (desired angle): ', imu_angle_1_yaw)
        # print(f"Current pitch angle: {current_angle_2_pitch}")        
        
        print(f"+++current roll angle: {current_angle_3_roll}")
        print('+++imu_angle_3_roll (desired angle): ', imu_angle_3_roll)
        
        # print('---hold_angle_1_yaw: ', hold_angle_1_yaw)
        # print('---initial_angle_1_yaw: ', initial_angle_1_yaw)
        # print('---send_angle_1_yaw: ', send_angle_1_yaw)
        actuator_1_yaw.sendPositionAbsoluteSetpoint(send_angle_1_yaw, max_speed)
        actuator_2_pitch.sendPositionAbsoluteSetpoint(send_angle_2_pitch, max_speed)
        actuator_3_roll.sendPositionAbsoluteSetpoint(send_angle_3_roll, max_speed)
        
        time.sleep(.01)
        

    
    
except KeyboardInterrupt:
    print("Ctrl+C detected")
    # Shutdown the motor
    if use_3_roll: 
        actuator_3_roll.shutdownMotor()
    if use_2_pitch: 
        actuator_2_pitch.shutdownMotor()
    if use_1_yaw:
        actuator_1_yaw.shutdownMotor()