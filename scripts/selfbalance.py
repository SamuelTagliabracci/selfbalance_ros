#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import os
import time
import math
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_matrix
import tf.transformations as tf

#Default Kp = 0.5 Ki = 0.1 Kd = 0.01

#centerpoint = 0.6625, Kp =4, Ki = 0.1, Kd = 0.3 Forward Slow Crawl

CenterPoint = 0.67
ActivationError = 0.08

Kp = 12
Ki = 0.3
Kd = 0.1


class BalancingRobot:
  def init(self):

    # Initialize ROS node
    rospy.init_node("selfbalancing_node")

    self.imu_topic="/gx5/imu/data"
    self.vel_topic="/cmd_vel"

    # Subscribe to IMU topic
    self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)

    # Advertise Twist topic for controlling motors
    self.cmd_vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=1)

    # Get parameter values from ROS parameter server
    self.Kp = rospy.get_param("selfbalance_Kp", Kp)
    self.Ki = rospy.get_param("selfbalance_Ki", Ki)
    self.Kd = rospy.get_param("selfbalance_Kd", Kd)

    self.error_integral = 0
    self.last_error = 0
    self.last_time = rospy.Time.now()

  def imu_callback(self, msg):

    # Calculate error (difference between current pitch and desired pitch of 0)
    quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

    # Convert quaternion to pitch angle
    #error = math.atan2(2 * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0]), 1 - 2 * (quaternion[1] ** 2 + quaternion[2] ** 2))

    roll, pitch, yaw = tf.euler_from_quaternion(quaternion)

    #pitch_degrees = pitch * 180.0 / 3.14159265358979323846
    print("roll: " + str(roll))
    print("pitch: " + str(pitch))
    print("yaw: " + str(yaw))
    
     error = pitch

    #q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    #R = quaternion_matrix(q)[:3,:3] # Extract the rotation matrix from the quaternion

    #gravity = [0.0, 0.0, -9.81] # Define the gravity vector in the body frame
    #expected_gravity = R.dot(gravity) # Rotate the gravity vector to the global frame

    #measured_gravity = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

    #error = math.sqrt(sum([(a - b)**2 for a, b in zip(measured_gravity, expected_gravity)]))

    #q_x = msg.orientation.x
    #q_y = msg.orientation.y
    #q_z = msg.orientation.z
    #q_w = msg.orientation.w

    #error = math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x ** 2 + q_y ** 2))

    #print(error)

    #print("Roll angle: " + str(roll))
    #print("Pitch angle: " + str(pitch))
    #print("Yaw angle: " + str(yaw))
    #error = CenterPoint - msg.orientation.y

    # Calculate derivative of error
    error_dot = (error - self.last_error) / (rospy.Time.now() - self.last_time).to_sec()

    # Calculate integral of error
    self.error_integral += error * (rospy.Time.now() - self.last_time).to_sec()

    if abs(error) > ActivationError:

      #Set 0 if error angle is beyond safe compensation aka abort self balance effort
      control_output = 0

    else:

      # Calculate control output using PID formula
      control_output = self.Kp * error + self.Ki * self.error_integral + self.Kd * error_dot

    # Publish Twist message to control motors
    twist_msg = Twist()
    twist_msg.linear.x = control_output * -1 # forward/backward motion
    twist_msg.linear.y = 0  # left/right motion
    twist_msg.linear.z = 0  # up/down motion
    twist_msg.angular.x = 0  # roll
    twist_msg.angular.y = 0  # pitch
    twist_msg.angular.z = 0  # yaw
    self.cmd_vel_pub.publish(twist_msg)

    # Save current values for next iteration
    self.last_error = error
    self.last_time = rospy.Time.now()

if __name__ == '__main__':

    robot = BalancingRobot().init()

    # Spin ROS node
    rospy.spin()
