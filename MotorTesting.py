""" post_motor_position_control_step.py

Contains the example code to run a DC motor that has an integrated shaft
encoder as a closed-loop position system with a PID controller.
A step input is applied and can be used to tune the PID gains.
https://thingsdaq.org/2022/05/15/motor-position-control-with-raspberry-pi/

Run this in a terminal instead of an interactive window.

Author: Eduardo Nigro
    rev 0.0.1
    2022-05-15

"""
# Importing modules and classes
import time
import numpy as np
import pigpio
from utils import plot_line
from gpiozero_extended import Motor, PID

# Setting general parameters
tstop = 3  # Execution duration (s)
tsample = 0.01  # Sampling period (s)
targetMotorAngle = 360  # Motor position set point (deg)
tau = 0.1  # Speed low-pass filter response time (s)

# Creating PID controller object
kp = 0.036
ki = 0.379
kd = 0.0009
taupid = 0.01
pid = PID(tsample, kp, ki, kd, tau=taupid)

pi = pigpio.pi()
mymotor = Motor(pi, direction=27, pwm1=12, encoder1=17, encoder2=18, encoderppr=2660)

mymotor.reset_angle()

# Pre-allocating output arrays
t = []  # Time (s)
theta = []  # Measured shaft position (deg)
u = []  # Controler output

# Initializing variables and starting clock
thetaprev = 0
tprev = 0
tcurr = 0
tstart = time.perf_counter()



try:
    # Running execution loop
    print('Running code for', tstop, 'seconds ...')
    while tcurr <= tstop:
        # Pausing for `tsample` to give CPU time to process encoder signal
        time.sleep(tsample)
        # Getting current time (s)
        tcurr = time.perf_counter() - tstart
        # Getting motor shaft angular position: I/O (data in)
        currentMotorAngle = mymotor.get_angle()    
        currentEncoderSteps = mymotor.pos

        #print("encoder count = ", currentEncoderSteps, "Angle = ", currentMotorAngle)
        # Calculating closed-loop output
        ucurr = pid.control(targetMotorAngle, currentMotorAngle)
        # Assigning motor output: I/O (data out)
        mymotor.set_output(ucurr)
        #mymotor.set_output(0.1)
        # Updating output arrays
        t.append(tcurr)
        theta.append(currentMotorAngle)
        u.append(ucurr)
        # Updating previous values
        thetaprev = currentMotorAngle
        tprev = tcurr

    print('Done. Final Angle = ' ,mymotor.get_angle() )
    # Stopping motor and releasing GPIO pins
except:
    print("Something went wrong")

mymotor.set_output(0)
del mymotor
pi.stop()

# Plotting results
# plot_line(
#     [t]*2, [theta, u], marker=True, axes='multi',
#     yname=['Shaft Position (deg.)', 'Control Output (-)'])
# plot_line(t[1::], 1000*np.diff(t), marker=True, yname='Sampling Period (ms)')