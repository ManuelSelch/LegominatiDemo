#!/usr/bin/env pybricks-micropython

from pybricks.tools import wait
from robot import Robot

class Main():
    def __init__(self):
        robot = Robot()

        while True:
            robot.pid_regler()
            wait(10)