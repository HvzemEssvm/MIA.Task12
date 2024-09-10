#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
from dynamic_reconfigure.server import Server
from turtlebot_pid.cfg import PIDConfig

# Global variables for the current position and orientation
current_x = 0
current_y = 0
current_theta = 0

# Target values
target_x = 2.0
target_y = 2.0
target_theta = 0.0

# PID parameters (initialized)
kp_linear = 1.0
ki_linear = 0.01
kd_linear = 0.1
kp_angular = 1.0
ki_angular = 0.01
kd_angular = 0.1

prev_distance_error = 0
prev_angular_error = 0
distance_integral = 0
angular_integral = 0

def odom_callback(data):
    global current_x, current_y, current_theta
    position = data.pose.pose.position
    current_x = position.x
    current_y = position.y

    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, current_theta = euler_from_quaternion(orientation_list)

def dynamic_reconfigure_callback(config, level):
    global kp_linear, ki_linear, kd_linear, kp_angular, ki_angular, kd_angular
    kp_linear = config.kp_linear
    ki_linear = config.ki_linear
    kd_linear = config.kd_linear
    kp_angular = config.kp_angular
    ki_angular = config.ki_angular
    kd_angular = config.kd_angular
    return config

def control_loop():
    global prev_distance_error, prev_angular_error, distance_integral, angular_integral
    rospy.init_node('turtlebot_pid')

    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Initialize dynamic reconfigure server
    srv = Server(PIDConfig, dynamic_reconfigure_callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Compute errors
        distance_error = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
        angle_to_goal = math.atan2(target_y - current_y, target_x - current_x)
        angular_error = angle_to_goal - current_theta
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))  # Normalize angle

        # PID for linear velocity
        distance_integral += distance_error
        linear_velocity = (kp_linear * distance_error +
                           kd_linear * (distance_error - prev_distance_error) +
                           ki_linear * distance_integral)

        # PID for angular velocity
        angular_integral += angular_error
        angular_velocity = (kp_angular * angular_error +
                            kd_angular * (angular_error - prev_angular_error) +
                            ki_angular * angular_integral)

        # Publish velocity command
        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity
        vel_msg.angular.z = angular_velocity
        vel_pub.publish(vel_msg)

        # Update previous errors
        prev_distance_error = distance_error
        prev_angular_error = angular_error

        rate.sleep()

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
