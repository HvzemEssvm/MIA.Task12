#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt

# K values for linear and angular
Kp_lin = 0.5
Ki_lin = 0.0
Kd_lin = 0.1

Kp_ang = 1.0
Ki_ang = 0.0
Kd_ang = 0.1

# Velocity Limits
MAX_LINEAR_VELOCITY = 0.5
MAX_ANGULAR_VELOCITY = 1.0

# Velocity and Position Thresholds
POSITION_THRESHOLD = 0.1
ORIENTATION_THRESHOLD = 0.1

# Global Variables for Current and Target Position
current_x = 0.0
current_y = 0.0
current_theta = 0.0

target_x = 0.0  # Initialize target position
target_y = 0.0
target_theta = 0.0

# At Starting Point
prev_error_lin = 0.0
prev_error_ang = 0.0
integral_lin = 0.0
integral_ang = 0.0

# Function to Update the Current Position
def odom_callback(msg):
    global current_x, current_y, current_theta
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, current_theta) = euler_from_quaternion(orientation_list)

# Function to Apply PID Controlling on Linear and Angular
def apply_pid():
    global prev_error_lin, prev_error_ang, integral_lin, integral_ang

    error_lin = sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
    desired_theta = atan2(target_y - current_y, target_x - current_x)
    error_ang = desired_theta - current_theta

    # Applying the PID Formula on linear
    control_signal_lin = Kp_lin * error_lin + Ki_lin * integral_lin + Kd_lin * (error_lin - prev_error_lin)
    integral_lin += error_lin
    prev_error_lin = error_lin

    # Applying the PID Formula on angular
    control_signal_ang = Kp_ang * error_ang + Ki_ang * integral_ang + Kd_ang * (error_ang - prev_error_ang)
    integral_ang += error_ang
    prev_error_ang = error_ang

    # Limit velocities
    control_signal_lin = min(max(control_signal_lin, -MAX_LINEAR_VELOCITY), MAX_LINEAR_VELOCITY)
    control_signal_ang = min(max(control_signal_ang, -MAX_ANGULAR_VELOCITY), MAX_ANGULAR_VELOCITY)

    return control_signal_lin, control_signal_ang

def main():
    global target_x, target_y, target_theta

    rospy.init_node('pid_controller', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    stop_indicator = True

    while not rospy.is_shutdown():
        # Get Target Position from User Input
        if stop_indicator :
            print("\n")
            target_x = float(input("Enter target x position: "))
            target_y = float(input("Enter target y position: "))
            target_theta_degree = float(input("Enter target orientation (theta in degrees): "))
            target_theta = target_theta_degree*3.141592653589793238462643383279502884197/180
            stop_indicator = False
        
        linear_velocity, angular_velocity = apply_pid()

        # Condition to stop the robot if it reached the target
        if abs(linear_velocity) < POSITION_THRESHOLD and abs(angular_velocity) < ORIENTATION_THRESHOLD:
            linear_velocity = 0.0
            angular_velocity = 0.0
            stop_indicator = True

        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity
        vel_msg.angular.z = angular_velocity

        cmd_vel_pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
