#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist

# GPIO pin definitions
MOTOR_A_PWM_1 = 13 # left
MOTOR_A_PWM_2 = 19
MOTOR_B_PWM_1 = 12
MOTOR_B_PWM_2 = 18

# Frequency for PWM
PWM_FREQUENCY = 1000

# Initialize GPIO and PWM
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(MOTOR_A_PWM_1, GPIO.OUT)
GPIO.setup(MOTOR_A_PWM_2, GPIO.OUT)
GPIO.setup(MOTOR_B_PWM_1, GPIO.OUT)
GPIO.setup(MOTOR_B_PWM_2, GPIO.OUT)

pwm_a1 = GPIO.PWM(MOTOR_A_PWM_1, PWM_FREQUENCY)
pwm_a2 = GPIO.PWM(MOTOR_A_PWM_2, PWM_FREQUENCY)
pwm_b1 = GPIO.PWM(MOTOR_B_PWM_1, PWM_FREQUENCY)
pwm_b2 = GPIO.PWM(MOTOR_B_PWM_2, PWM_FREQUENCY)

pwm_a1.start(0)
pwm_a2.start(0)
pwm_b1.start(0)
pwm_b2.start(0)

def control_motors(linear_x, angular_z):
    """
    Control motors based on linear and angular velocity.
    """
    # Determine speed for each motor
    motor_a_speed = linear_x - angular_z
    motor_b_speed = linear_x + angular_z

    # Ensure speeds are within 0 to 100
    motor_a_speed = max(min(motor_a_speed, 1), -1)
    motor_b_speed = max(min(motor_b_speed, 1), -1)

    # Set motor A
    if motor_a_speed > 0:
        pwm_a1.ChangeDutyCycle(motor_a_speed * 100 * 1.15)
        pwm_a2.ChangeDutyCycle(0)
    elif motor_a_speed < 0:
        pwm_a1.ChangeDutyCycle(0)
        pwm_a2.ChangeDutyCycle(motor_a_speed * 100 * 1.15)
    else:
        pwm_a1.ChangeDutyCycle(0)
        pwm_a2.ChangeDutyCycle(0)

    # Set motor B
    if motor_b_speed > 0:
        pwm_b1.ChangeDutyCycle(motor_b_speed * 100)
        pwm_b2.ChangeDutyCycle(0)
    elif motor_b_speed < 0:
        pwm_b1.ChangeDutyCycle(0)
        pwm_b2.ChangeDutyCycle(motor_b_speed * 100)
    else:
        pwm_b1.ChangeDutyCycle(0)
        pwm_b2.ChangeDutyCycle(0)

def velocity_callback(msg):
    """
    Callback function for /cmd_vel topic.
    """
    linear_x = msg.linear.x * 2
    angular_z = msg.angular.z / 10

    control_motors(linear_x, angular_z)

def main():
    """
    Main function to initialize the ROS node and subscribe to /cmd_vel.
    """
    rospy.init_node('motor_control_node')
    rospy.Subscriber('/cmd_vel', Twist, velocity_callback)

    rospy.loginfo("Motor control node started. Listening to /cmd_vel topic.")
    rospy.spin()

    # Cleanup GPIO on shutdown
    pwm_a1.stop()
    pwm_a2.stop()
    pwm_b1.stop()
    pwm_b2.stop()
    GPIO.cleanup()

if __name__ == "__main__":
    main()
