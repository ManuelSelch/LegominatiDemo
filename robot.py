#!/usr/bin/env pybricks-micropython


from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Button
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick

class Robot():
    #region parameters
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

    ev3 = EV3Brick()
    #endregion

    #region pid regler
    def pid_regler(self):
        #p = proportional
        #i = integral
        #d = derivative: future error based on past error
        last_error = 0
        integral = 0
        while True:
            light_sensor_value = self.farbsensor.reflection()
            error = light_sensor_value - self.mittelwert

            integral = integral + error
            derivative = error - last_error

            turn_rate = self.faktorP * error + self.faktorI * integral + self.faktorD * derivative
            self.robot.drive(self.geschwindigkeit, turn_rate)

            last_error = error
    #endregion
        