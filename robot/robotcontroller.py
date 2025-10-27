from linefollower import LineFollower
from motor_controller import MotorController
from math import *
import time
import numpy as np
from odometer import Odometer

HALF_PI = np.pi / 2
LEFT = 1
RIGHT = 2

class RobotController():

    def __init__(self, linear_velocity = 5, angular_velocity=np.deg2rad(20), half_span=12.86 / 2, debug=False):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.line_follower = LineFollower(debug=debug)
        self.motor_controller = MotorController()
        self.half_span = half_span
        self.max_linear = linear_velocity
        self.max_angular = angular_velocity
        self.perimeter = 2 * np.pi * half_span
        self.odom = Odometer(self.motor_controller.ids)

    def tick_line_follow(self, frame, _delta):
        _, angle, done = self.line_follower.update(frame)
        if angle is None:
            return
        if done:
            return True
        angular_ratio = -(HALF_PI - angle) / HALF_PI

        print(f"Angular ratio is {angular_ratio}")

        angular_vel = self.max_angular * angular_ratio
        linear_vel = self.max_linear * (1 - abs(angular_ratio))
        
        left_vel, right_vel = self.inverse_kinematics(linear_vel, angular_vel)
        
        print(f"Computed left speed {left_vel}, computed right speed {right_vel}")

        self.motor_controller.set_speed(LEFT, left_vel)
        self.motor_controller.set_speed(RIGHT, -right_vel)

    def tick_odom(self, dt):
        left_velocity = self.motor_controller.get_speed(LEFT)
        right_velocity = -self.motor_controller.get_speed(RIGHT)
        (dx, dy, dtheta) = self.odom.odom(left_velocity, right_velocity, dt)
        # print(f"x= {dx}; y= {dy}; theta= {dtheta}")
        dx, dy = np.dot(
                np.array(
                    [[cos(-self.theta), -sin(-self.theta)],
                    [sin(-self.theta), cos(-self.theta)]]
                ),
                np.array([dx, dy])
        )
        self.x += dx
        self.y += dy
        self.theta += dtheta


    def panic(self):
        self.motor_controller.stop_all()

    def inverse_kinematics(self, linear_velocity, angular_velocity):
        left_velocity = linear_velocity - self.half_span * angular_velocity
        right_velocity = linear_velocity + self.half_span * angular_velocity

        return left_velocity, right_velocity
    
    # def go_to(self, x_goal, y_goal, teta_goal):
    #     dist = np.sqrt(x_goal*x_goal + y_goal*y_goal)
    #     alpha = np.arccos(x_goal / dist) 
    #     rotate = alpha * self.half_span
    #     self.motor_controller.set_position(2,rotate)     #une roue va à rotate, et l'autre inverse, mais elles sont inversees de base, donc si on met les deux à rotate, il va tourner 
    #     self.motor_controller.set_position(5,rotate)     #une roue va à rotate, et l'autre inverse, mais elles sont inversees de base, donc si on met les deux à rotate, il va tourner 
    

    def go_to(self, x, y, theta):
        distance = np.sqrt(x**2 + y**2)
        alpha = np.arccos(x / distance)
        # print(f"distance {distance}, alpha {alpha}")

        angular = np.deg2rad(45)
        # Tourner pendant un temps
        lv,rv  = self.inverse_kinematics(0, angular) #45 deg/s
        temps_manoeuvre = alpha / angular
        previous_time = time.monotonic() # in seconds
        print(f"left speed {lv}, right speed {rv}, temps manoeuvre {temps_manoeuvre}")
        while(time.monotonic() - previous_time < temps_manoeuvre):
            self.motor_controller.set_speed(LEFT, lv)
            self.motor_controller.set_speed(RIGHT, -rv)

        self.motor_controller.stop_all()
        # return
        # Avancer pendant un temps
        lv,rv  = self.inverse_kinematics(20, 0) #20 cm/s
        temps_manoeuvre = distance / 20
        previous_time = time.monotonic() # in seconds
        while(time.monotonic() - previous_time < temps_manoeuvre):
            self.motor_controller.set_speed(LEFT, lv)
            self.motor_controller.set_speed(RIGHT, -rv)
        self.motor_controller.stop_all()

        # Tourner pendant un temps
        lv,rv  = self.inverse_kinematics(0, angular) #45 deg/s
        temps_manoeuvre = (np.deg2rad(theta)-alpha) / angular
        previous_time = time.monotonic() # in seconds
        while(time.monotonic() - previous_time < temps_manoeuvre):
            self.motor_controller.set_speed(LEFT, lv)
            self.motor_controller.set_speed(RIGHT, -rv)
        
        self.motor_controller.stop_all()

if __name__ == "__main__":
    left = 2
    right = 5
    robot = RobotController(debug=False)
    length = 45
    angle = 90
    # time = 9
    # linear_vel = length / time
    # angular_vel = np.deg2rad(angle) / time

    # left_vel, right_vel = robot.inverse_kinematics(linear_vel, angular_vel)
    
    # print(robot.motor_controller.speed_to_radial(left_vel))
    # print(robot.motor_controller.speed_to_radial(right_vel))
    # robot.motor_controller.set_speed(left, -left_vel)
    # robot.motor_controller.set_speed(right, right_vel)
    # sleep(time)
    # robot.motor_controller.motor_stop(left)
    # robot.motor_controller.motor_stop(right)

    # robot.go_to(200, 100, 270)

    previous_time = time.monotonic()
    robot.motor_controller.dxl_io.disable_torque(robot.motor_controller.ids)
    # robot.motor_controller.set_speed(1,0)
    # robot.motor_controller.set_speed(2,0)
    while 1:
        current_time = time.monotonic()
        robot.tick_odom(current_time - previous_time)
        print(f"x= {robot.x}; y= {robot.y}; theta= {robot.theta}")
        previous_time = current_time



