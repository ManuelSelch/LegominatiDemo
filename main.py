#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the color sensor.
farbsensor = ColorSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Calculate the light threshold. Choose values based on your measurements.
schwarz = 9
weiss = 85
mittelwert = (schwarz + weiss) / 2

# Set the drive speed at 100 millimeters per second.
speedFaktor = 8
geschwindigkeit = -10 * speedFaktor

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
faktorP = 0.05 * speedFaktor
faktorI = 0.01 * speedFaktor
faktorD = -0.5 * speedFaktor

global i, old1, old2, old3, last
i = 0
old1 = farbsensor.reflection() - mittelwert
old2 = farbsensor.reflection() - mittelwert
old3 = farbsensor.reflection() - mittelwert

last = farbsensor.reflection() - mittelwert

#comment laurin 02

ev3 = EV3Brick()

def pid_regler():
    #p = proportional
    #i = integral
    #d = derivative: future error based on past error
    last_error = 0
    integral = 0
    while True:
        light_sensor_value = farbsensor.reflection()
        error = light_sensor_value - mittelwert

        integral = integral + error
        derivative = error - last_error

        turn_rate = faktorP * error + faktorI * integral + faktorD * derivative
        robot.drive(geschwindigkeit, turn_rate)

        last_error = error
        wait(10)

def run1():
    global i, old1, old2, old3, last
    print("run 1")
    while True:
        i = i+1
        if(i%3 == 0):
            old1 = farbsensor.reflection() - mittelwert
        if(i%6 == 0):
            old2 = farbsensor.reflection() - mittelwert
        if(i%9 == 0):
            old3 = farbsensor.reflection() - mittelwert
        

        # Calculate the deviation from the threshold.
        p_regler = faktorP * (farbsensor.reflection() - mittelwert)
        i_regler =  faktorI * (old1 + old2 + old3)
        d_regler =  faktorD * (farbsensor.reflection() - last)
        last = farbsensor.reflection()
        abweichung =  p_regler + d_regler # + i_regler 

        # Set the drive base speed and turn rate.
        robot.drive(geschwindigkeit, abweichung)

        # You can wait for a short time or do other things in this loop.
        #wait(10)


def run2():
    print("run 2")

    robot.straight(300)
    robot.stop()
    
def run3():
    pid_regler()
    print("run 3")

def run4():
    print("run 4")

# Start following the line endlessly.
while True:
    if Button.UP in ev3.buttons.pressed():
        run1()
    if Button.RIGHT in ev3.buttons.pressed():
        run2()
    if Button.DOWN in ev3.buttons.pressed():
        run3()
    if Button.LEFT in ev3.buttons.pressed():
        run4()

    