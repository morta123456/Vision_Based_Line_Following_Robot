import numpy as np
import time
import mathutils
from math import *

import pypot.dynamixel
import RobotController 

class Odometer():

    def __init__(self, motor_ids, half_span=12.86 / 2):
        self.robot = RobotController()
        self.motor_ids = motor_ids
        self.half_span = half_span
        pass

    def inverse_kinematics(self, linear_velocity, angular_velocity):
        left_vellocity = linear_velocity - self.half_span * angular_velocity
        right_velocity = linear_velocity + self.half_span * angular_velocity
        return (left_vellocity, right_velocity)
    
    def go_to(self, x, y, theta):
        distance = np.sqrt(x**2 + y**2)
        alpha = np.arccos(x / distance)

        # Tourner pendant un temps
        lv,rv  = self.inverse_kinematics(0, 45) #45 deg/s
        temps_manoeuvre = alpha / 45
        previous_time = time.monotonic() # in seconds
        while(True):
            delta = time.monotonic() - previous_time
            if(delta > temps_manoeuvre):
                break
            self.set_speed(lv, rv)

        # Avancer pendant un temps
        lv,rv  = self.inverse_kinematics(20, 0) #20 cm/s
        temps_manoeuvre = distance / 20
        previous_time = time.monotonic() # in seconds
        while(True):
            delta = time.monotonic() - previous_time
            if(delta > temps_manoeuvre):
                break
            self.set_speed(lv, rv)

        # Tourner pendant un temps
        lv,rv  = self.inverse_kinematics(0, 45) #45 deg/s
        temps_manoeuvre = (theta-alpha) / 45
        previous_time = time.monotonic() # in seconds
        while(True):
            delta = time.monotonic() - previous_time
            if(delta > temps_manoeuvre):
                break
            self.set_speed(lv, rv)


if __name__ == "__main__":
    left = 2
    right = 5
    robot = Odometer
    length = 45
    angle = 90
    time = 9
    linear_vel = length / time
    angular_vel = np.deg2rad(angle) / time

    # left_vel, right_vel = robot.inverse_kinematics(linear_vel, angular_vel)
    
    # print(robot.motor_controller.speed_to_radial(left_vel))
    # print(robot.motor_controller.speed_to_radial(right_vel))
    # robot.motor_controller.set_speed(left, -left_vel)
    # robot.motor_controller.set_speed(right, right_vel)
    # sleep(time)
    # robot.motor_controller.motor_stop(left)
    # robot.motor_controller.motor_stop(right)

    robot.go_to(100, 0, 0)
        