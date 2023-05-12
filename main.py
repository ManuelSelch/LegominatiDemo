#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase

import time

# Initialize the EV3 Brick.
ev3 = EV3Brick()

#test comment

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=62.4, axle_track=104)
robot.settings(500,200,500,200)

# Go forward and backwards for one meter.
robot.straight(1000)
robot.stop()
ev3.speaker.beep()
while(True):    
    colorSensor = ColorSensor()
    left_motor.hold()

