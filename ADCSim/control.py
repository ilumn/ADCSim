import numpy as np
from physics import getVirtualSensorAngle

# Aggressive PID gains for fast response.
kp = 0.5    # Proportional gain
ki = 0.01   # Integral gain
kd = 0.05   # Derivative gain

# target angle input degrees (will be converted to radians)
targetAngle = np.deg2rad(45)

# PID state
pidIntegral = 0.0
pidPrevError = 0.0

# error between target and measured angles (wrapped to [-pi, pi]).
def angleError(target, measured):
    error = target - measured
    error = (error + np.pi) % (2 * np.pi) - np.pi
    return -error

# PID controller: uses virtual sensor reading to compute and return a motor command
def controlSystemPID(cubeSat, dt, sensorNoiseStd=0.005):
    global pidIntegral, pidPrevError

    measuredAngle = getVirtualSensorAngle(cubeSat, noiseStd=sensorNoiseStd)
    error = angleError(targetAngle, measuredAngle)
    derivative = (error - pidPrevError) / dt
    pidIntegral += error * dt
    pidIntegral = max(min(pidIntegral, 1.0), -1.0)

    pTerm = kp * error
    iTerm = ki * pidIntegral
    dTerm = kd * derivative
    pidPrevError = error

    motorCommand = pTerm + iTerm + dTerm
    maxCommand = 1.0
    motorCommand = max(min(motorCommand, maxCommand), -maxCommand)
    motorCommand += np.random.normal(0, 0.00001)
    
    return motorCommand, error, pTerm, iTerm, dTerm