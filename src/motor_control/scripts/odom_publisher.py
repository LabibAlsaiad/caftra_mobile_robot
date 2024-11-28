#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import time
from math import pi, sin, cos, fmod
import tf
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

# GPIO Pin Definitions
ENCODER_LEFT_A = 23
ENCODER_LEFT_B = 24
ENCODER_RIGHT_A = 22
ENCODER_RIGHT_B = 27

# Calibrated Robot Specifications
WHEEL_RADIUS = 0.04426  # meters
WHEEL_BASE = 0.21  # meters
TICKS_PER_REV_LEFT = 660.6
TICKS_PER_REV_RIGHT = 660.0
DISTANCE_PER_TICK_LEFT = 0.00042097151331481754  # meters per tick
DISTANCE_PER_TICK_RIGHT = 0.00042135421469055833  # meters per tick

class DifferentialDriveOdom:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('differential_drive_odom', anonymous=True)

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENCODER_LEFT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_LEFT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_RIGHT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_RIGHT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Encoder state variables
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.last_left_encoder_count = 0
        self.last_right_encoder_count = 0

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Publishers
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Encoder interrupt setup
        GPIO.add_event_detect(ENCODER_LEFT_A, GPIO.RISING, callback=self.left_encoder_callback)
        GPIO.add_event_detect(ENCODER_RIGHT_A, GPIO.RISING, callback=self.right_encoder_callback)

        # Rate and time tracking
        self.rate = rospy.Rate(50)  # 50 Hz update rate
        self.last_time = rospy.Time.now()

    def left_encoder_callback(self, channel):
        # Increment left encoder count based on direction
        if GPIO.input(ENCODER_LEFT_B) == GPIO.HIGH:
            self.left_encoder_count += 1
        else:
            self.left_encoder_count -= 1

    def right_encoder_callback(self, channel):
        # Increment right encoder count based on direction
        if GPIO.input(ENCODER_RIGHT_B) == GPIO.HIGH:
            self.right_encoder_count -= 1
        else:
            self.right_encoder_count += 1

    def calculate_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Calculate distance traveled by each wheel
        left_distance = (self.left_encoder_count - self.last_left_encoder_count) * DISTANCE_PER_TICK_LEFT
        right_distance = (self.right_encoder_count - self.last_right_encoder_count) * DISTANCE_PER_TICK_RIGHT

        # Calculate linear and angular velocities
        linear_velocity = (left_distance + right_distance) / (2 * dt)
        angular_velocity = (right_distance - left_distance) / (WHEEL_BASE * dt)

        # Update pose
        self.theta += angular_velocity * dt
        self.theta = fmod(self.theta, 2 * pi)  # Normalize angle

        # Calculate average distance and heading
        avg_distance = (left_distance + right_distance) / 2
        cos_theta = cos(self.theta)
        sin_theta = sin(self.theta)

        # Update x and y positions
        self.x += avg_distance * cos_theta
        self.y += avg_distance * sin_theta

        # Create quaternion from yaw
        quaternion = quaternion_from_euler(0, 0, -self.theta)

        # Publish transform
        transform = TransformStamped()
        transform.header.stamp = current_time
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(
            (transform.transform.translation.x, 
             transform.transform.translation.y, 
             transform.transform.translation.z),
            (transform.transform.rotation.x, 
             transform.transform.rotation.y, 
             transform.transform.rotation.z, 
             transform.transform.rotation.w),
            current_time,
            transform.child_frame_id,
            transform.header.frame_id
        )

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom)

        # Update last state
        self.last_time = current_time
        self.last_left_encoder_count = self.left_encoder_count
        self.last_right_encoder_count = self.right_encoder_count

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.calculate_odometry()
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            GPIO.cleanup()

if __name__ == '__main__':
    odom_node = DifferentialDriveOdom()
    odom_node.run()