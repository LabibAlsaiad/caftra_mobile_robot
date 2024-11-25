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
ENCODER_LEFT_A = 27
ENCODER_LEFT_B = 22
ENCODER_RIGHT_A = 23
ENCODER_RIGHT_B = 24

# Calibrated Robot Specifications
WHEEL_RADIUS = 0.04426  # meters
WHEEL_BASE = 0.21       # meters
TICKS_PER_REV_LEFT = 660.6
TICKS_PER_REV_RIGHT = 660.0
DISTANCE_PER_TICK_LEFT = 0.00042097151331481754   # meters per tick
DISTANCE_PER_TICK_RIGHT = 0.00042135421469055833  # meters per tick

class DifferentialDriveOdometry:
    def __init__(self):
        # Initialize odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Encoder counts
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        
        # Timing
        self.last_time = rospy.Time.now()
        
        # Moving average filters for velocity
        self.velocity_filter_size = 5
        self.linear_velocities = []
        self.angular_velocities = []
        
        # Covariance matrices (to be tuned based on your robot)
        self.pose_covariance = [0.001, 0, 0, 0, 0, 0,
                               0, 0.001, 0, 0, 0, 0,
                               0, 0, 0.001, 0, 0, 0,
                               0, 0, 0, 0.001, 0, 0,
                               0, 0, 0, 0, 0.001, 0,
                               0, 0, 0, 0, 0, 0.003]
        
        self.twist_covariance = [0.001, 0, 0, 0, 0, 0,
                                0, 0.001, 0, 0, 0, 0,
                                0, 0, 0.001, 0, 0, 0,
                                0, 0, 0, 0.001, 0, 0,
                                0, 0, 0, 0, 0.001, 0,
                                0, 0, 0, 0, 0, 0.003]

    def encoder_callback(self, pin, is_left):
        """Improved quadrature encoder reading"""
        if is_left:
            A_pin = ENCODER_LEFT_A
            B_pin = ENCODER_LEFT_B
        else:
            A_pin = ENCODER_RIGHT_A
            B_pin = ENCODER_RIGHT_B
            
        A_current = GPIO.input(A_pin)
        B_current = GPIO.input(B_pin)
        
        if is_left:
            if A_current != B_current:
                self.left_ticks -= 1  # Reversed direction for left wheel
            else:
                self.left_ticks += 1  # Reversed direction for left wheel
        else:
            if A_current != B_current:
                self.right_ticks -= 1  # Reversed direction for right wheel
            else:
                self.right_ticks += 1  # Reversed direction for right wheel

    def normalize_angle(self, angle):
        """Normalize angle to be between -pi and pi"""
        return fmod(angle + pi, 2.0 * pi) - pi

    def update_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        if dt < 1e-6:
            return 0.0, 0.0
            
        # Calculate wheel movements using calibrated values
        delta_left = (self.left_ticks - self.last_left_ticks) * DISTANCE_PER_TICK_LEFT
        delta_right = (self.right_ticks - self.last_right_ticks) * DISTANCE_PER_TICK_RIGHT
        
        # Update last values
        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks
        self.last_time = current_time
        
        # Calculate distance traveled by each wheel
        delta_distance = (delta_right + delta_left) / 2.0
        delta_theta = (delta_right - delta_left) / WHEEL_BASE  # This remains the same as it's just the angular difference
        
        # Calculate velocities
        linear_velocity = delta_distance / dt
        angular_velocity = delta_theta / dt
        
        # Apply moving average filter
        self.linear_velocities.append(linear_velocity)
        self.angular_velocities.append(angular_velocity)
        if len(self.linear_velocities) > self.velocity_filter_size:
            self.linear_velocities.pop(0)
            self.angular_velocities.pop(0)
            
        filtered_linear = sum(self.linear_velocities) / len(self.linear_velocities)
        filtered_angular = sum(self.angular_velocities) / len(self.angular_velocities)
        
        # Update pose - Modified to match ROS coordinate convention
        if abs(delta_theta) < 1e-6:
            # Straight line approximation for very small angular changes
            self.x += delta_distance * cos(self.theta)  # Forward motion is +x
            self.y += delta_distance * sin(self.theta)  # Leftward motion is +y
        else:
            # Exact equations for curved path
            radius = delta_distance / delta_theta
            self.x += radius * (sin(self.theta + delta_theta) - sin(self.theta))  # Changed signs to match ROS convention
            self.y -= radius * (cos(self.theta + delta_theta) - cos(self.theta))  # Changed signs to match ROS convention
        
        self.theta = self.normalize_angle(self.theta + delta_theta)
        
        return filtered_linear, filtered_angular

    def publish_odometry(self, linear_velocity, angular_velocity):
        # Create quaternion from yaw
        quaternion = quaternion_from_euler(0, 0, self.theta)
        
        # First, publish the transform over tf
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.last_time
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_footprint"
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation.x = quaternion[0]
        odom_trans.transform.rotation.y = quaternion[1]
        odom_trans.transform.rotation.z = quaternion[2]
        odom_trans.transform.rotation.w = quaternion[3]
        
        # Next, publish the odometry message
        odom = Odometry()
        odom.header.stamp = self.last_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        
        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_trans.transform.rotation
        
        # Set the velocity
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        
        # Set the covariance
        odom.pose.covariance = self.pose_covariance
        odom.twist.covariance = self.twist_covariance
        
        return odom_trans, odom

def main():
    rospy.init_node('differential_drive_odometry')
    
    # Create odometry object
    odom = DifferentialDriveOdometry()
    
    # Setup publishers
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
    tf_broadcaster = tf.TransformBroadcaster()
    
    # Setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ENCODER_LEFT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_LEFT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_RIGHT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_RIGHT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    # Setup encoder callbacks
    GPIO.add_event_detect(ENCODER_LEFT_A, GPIO.BOTH, 
                         callback=lambda x: odom.encoder_callback(x, True))
    GPIO.add_event_detect(ENCODER_RIGHT_A, GPIO.BOTH, 
                         callback=lambda x: odom.encoder_callback(x, False))
    
    rate = rospy.Rate(50)  # 50Hz
    
    try:
        while not rospy.is_shutdown():
            # Update odometry
            linear_vel, angular_vel = odom.update_odometry()
            
            # Publish odometry data
            transform, odom_msg = odom.publish_odometry(linear_vel, angular_vel)
            tf_broadcaster.sendTransformMessage(transform)
            odom_pub.publish(odom_msg)
            
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()