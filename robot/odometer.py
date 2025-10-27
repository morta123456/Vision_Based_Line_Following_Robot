import numpy as np
import mathutils
from math import *

import pypot.dynamixel

class Odometer():

    def __init__(self, motor_ids, half_span=12.86 / 2):
        self.motor_ids = motor_ids
        self.half_span = half_span
        pass

    def direct_kinematics(self, left_velocity, right_velocity):
        robot_velocity = (left_velocity + right_velocity) / 2
        teta = (right_velocity - left_velocity) / (2 * self.half_span)
        return (robot_velocity, teta)
    
    def odom(self, left_velocity, right_velocity, time_duration):
        (linear_velocity, angular_velocity) = self.direct_kinematics(left_velocity, right_velocity)
        angular_variation = (angular_velocity * time_duration)
        rayon  = 0
        if angular_velocity != 0 :
            rayon = linear_velocity / angular_velocity
        x_variation = rayon * sin(angular_variation)
        y_variation = rayon * (1 - cos(angular_variation))
        return(x_variation, y_variation, angular_variation)


if __name__ == "__main__":
    left = 2
    right = 5
    robot = RobotController(debug=False)
