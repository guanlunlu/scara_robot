#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import math
import time
import word_trajectory as word

ev3 = EV3Brick()

class Scara():
    def __init__(self):
        # self.ev3 = EV3Brick()
        self.motor_a = Motor(Port.A, Direction.COUNTERCLOCKWISE, gears=[1, 5])
        self.motor_b = Motor(Port.B, Direction.COUNTERCLOCKWISE, gears=[1, 5])
        self.motor_c = Motor(Port.C, Direction.COUNTERCLOCKWISE)
        self.motor_a_init_angle = 0
        self.motor_b_init_angle = 90
        self.joint_track_tol = 0.5
        
        # timer config
        self.watch = StopWatch()
        self.prev_time = 0
        self.iter_time = 0
        # hz
        self.control_freq = 100
        self.control_period = 1/self.control_freq
        self.timeout = 100

        # trajectory
        self.spring = word.word_spring([-20,80], 3)
        self.day = word.word_day([-20, 80], 3)
        # self.task = self.day
        self.task = self.spring

        # ik config
        self.L1 = 80
        self.L2 = 119
        pass

    def timer(self, theta_a, theta_b):
        prev_iter = 0
        self.iter_time = 0
        self.prev_time = 0
        J1_finished = 0
        J2_finished = 0

        while self.iter_time < self.timeout:
            d_t = self.watch.time()/1000 - prev_iter
            if (d_t >= 1 / self.control_freq):
                err_a = theta_a - self.motor_a.angle()
                err_b = theta_b - self.motor_b.angle()
                print("err ", err_a, err_b)
                print("motor ", self.motor_a.angle(), self.motor_b.angle())

                if(abs(err_a) > self.joint_track_tol):
                    if err_a > 0:
                        vel_a = 8
                    else:
                        vel_a = -8
                    # vel_a = err_a/self.control_period
                else:
                    vel_a = 0
                    J1_finished = 1

                if(abs(err_b) > self.joint_track_tol):
                    if err_b > 0:
                        vel_b = 8
                    else:
                        vel_b = -8
                    # vel_b = err_b/self.control_period
                else:
                    vel_b = 0
                    J2_finished = 1
                
                self.motor_a.run(vel_a)
                self.motor_b.run(vel_b)
                prev_iter = self.watch.time()/1000

            if(J1_finished and J2_finished):
                break 
                # print(self.iter_time)

            self.iter_time += self.watch.time()/1000 - self.prev_time
            self.prev_time = self.watch.time()/1000
    
    def controller(self):
        self.motor_set_zero()
        self.motor_c.run_target(500, 60)

        for seg in self.task.seg_list:
            pen_down = 0
            for pt in seg.trajectory:
                print("x, y ",pt)
                theta_a, theta_b = self.inverse_kinematics(pt)
                print("theta_goal ",(theta_a, theta_b))
                self.timer(theta_a, theta_b)
                if pen_down == 0:
                    self.motor_c.run_target(500, 0)
                    pen_down = 1
            self.motor_c.run_target(500, 60)
            print("---")


    def motor_set_zero(self):
        self.motor_a.reset_angle(self.motor_a_init_angle)
        self.motor_b.reset_angle(self.motor_b_init_angle)

    def inverse_kinematics(self, cart_pose):
        x, y = cart_pose
        L3 = math.sqrt(pow(x, 2)+pow(y, 2))
        phi = math.atan2(y, x)
        alpha_1 = math.acos((pow(self.L1, 2) + pow(L3, 2) - pow(self.L2, 2))/(2*self.L1*L3))
        alpha_2 = math.acos((pow(self.L1, 2) + pow(self.L2, 2) - pow(L3, 2))/(2*self.L1*self.L2))
        theta_1 = math.degrees(phi - alpha_1)
        theta_2 = math.degrees(phi - alpha_1 + (math.pi - alpha_2))
        return [theta_1, theta_2]
        

if __name__ == '__main__':
    scara = Scara()
    scara.controller()
    pass