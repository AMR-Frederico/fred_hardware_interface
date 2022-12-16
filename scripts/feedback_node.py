#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from math import pi

dict_pos = {}
dict_vel = {}
sim_switch = True

# Simulation feedback


# Real robot feedback
# Recebe em rpm e converte para rad/s


def callback_right(msg):
    # print("right")

    if True:
        pub_right_vel.publish(msg.data*0.10471975511966)


def callback_left(msg):
    # print("left")

    if True:
        pub_left_vel.publish(msg.data*0.10471975511966)

# recebe em graus e converte para radianos


def callback_right_angle(msg):
    # print("right angle")

    if True:
        pub_right_pos.publish(msg.data*pi/180)


def callback_left_angle(msg):
    # print("left angle")

    if True:
        pub_left_pos.publish(msg.data*pi/180)


if __name__ == '__main__':

    rospy.init_node('Joint_Position_Feedback')

    pub_right_pos = rospy.Publisher(
        "/feedback/right_position", Float64, queue_size=1)
    pub_left_pos = rospy.Publisher(
        "/feedback/left_position", Float64, queue_size=1)
    pub_right_vel = rospy.Publisher(
        "/feedback/right_velocity", Float64, queue_size=1)
    pub_left_vel = rospy.Publisher(
        "/feedback/left_velocity", Float64, queue_size=1)

    rospy.Subscriber("power/status/pwm/right", Float64, callback_right)
    rospy.Subscriber("power/status/pwm/left", Float64, callback_left)

    rospy.Subscriber("power/status/speed/angular/right",
                     Float64, callback_right_angle)
    rospy.Subscriber("power/status/speed/angular/left",
                     Float64, callback_left_angle)

    rospy.spin()
