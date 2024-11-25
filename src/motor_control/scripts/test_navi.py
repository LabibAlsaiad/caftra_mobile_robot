#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

# GPIO Pin Definitions
MOTOR_A_PWM1 = 13   # Motor A PWM pin
MOTOR_A_PWM2 = 19   # Motor A direction pin
ENCODER_A_A = 27   # Encoder A Channel A
ENCODER_A_B = 22   # Encoder A Channel B

MOTOR_B_PWM1 = 12   # Motor B PWM pin
MOTOR_B_PWM2 = 18   # Motor B direction pin
ENCODER_B_A = 23   # Encoder B Channel A
ENCODER_B_B = 24   # Encoder B Channel B

# Global variables for encoder counts
encoder_af_count = 0
encoder_ab_count = 0
encoder_bf_count = 0
encoder_bb_count = 0

def encoder_a_callback(channel):
    """Callback for Encoder A."""
    global encoder_af_count
    global encoder_ab_count
    state_a = GPIO.input(ENCODER_A_A)
    state_b = GPIO.input(ENCODER_A_B)
    if state_a == state_b:
        encoder_af_count += 1  # Forward
    else:
        encoder_ab_count -= 1  # Reverse

def encoder_b_callback(channel):
    """Callback for Encoder B."""
    global encoder_bf_count
    global encoder_bb_count
    state_a = GPIO.input(ENCODER_B_A)
    state_b = GPIO.input(ENCODER_B_B)
    if state_a == state_b:
        encoder_bf_count += 1  # Forward
    else:
        encoder_bb_count -= 1  # Reverse

def motor_test():
    """Test motor orientation and encoder feedback."""
    try:
        GPIO.setmode(GPIO.BCM)

        # Setup Motor GPIOs
        GPIO.setup(MOTOR_A_PWM1, GPIO.OUT)
        GPIO.setup(MOTOR_A_PWM2, GPIO.OUT)
        GPIO.setup(MOTOR_B_PWM1, GPIO.OUT)
        GPIO.setup(MOTOR_B_PWM2, GPIO.OUT)

        # Setup Encoder GPIOs
        GPIO.setup(ENCODER_A_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_A_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_B_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_B_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Setup PWM for motors
        pwm_a1 = GPIO.PWM(MOTOR_A_PWM1, 100)  # 100 Hz frequency
        pwm_a2 = GPIO.PWM(MOTOR_A_PWM2, 100)
        pwm_b1 = GPIO.PWM(MOTOR_B_PWM1, 100)  # 100 Hz frequency
        pwm_b2 = GPIO.PWM(MOTOR_B_PWM2, 100)
        pwm_a1.start(0)  # Start with 0% duty cycle
        pwm_a2.start(0)
        pwm_b1.start(0)  
        pwm_b2.start(0)

        # Setup Encoder Callbacks
        GPIO.add_event_detect(ENCODER_A_A, GPIO.BOTH, callback=encoder_a_callback)
        GPIO.add_event_detect(ENCODER_B_A, GPIO.BOTH, callback=encoder_b_callback)

        print("Starting Motor and Encoder Test...")
        input("Press Enter to start testing Motor A...")
        # Test Motor A Forward
        pwm_a1.ChangeDutyCycle(50)
        pwm_a2.ChangeDutyCycle(0)
        time.sleep(2)
        pwm_a1.ChangeDutyCycle(0)
        pwm_a2.ChangeDutyCycle(0)
        print(f"Motor A Forward Count: {encoder_af_count}")
        print(f"Motor A Backward Count: {encoder_ab_count}")
        input("Press Enter to test Motor A Reverse...")

        # Test Motor A Reverse
        pwm_a1.ChangeDutyCycle(0)
        pwm_a2.ChangeDutyCycle(50)
        time.sleep(2)
        pwm_a1.ChangeDutyCycle(0)
        pwm_a2.ChangeDutyCycle(0)
        print(f"Motor A Forward Count: {encoder_af_count}")
        print(f"Motor A Backward Count: {encoder_ab_count}")

        input("Press Enter to start testing Motor B...")

        # Test Motor B Forward
        pwm_b1.ChangeDutyCycle(50)
        pwm_b2.ChangeDutyCycle(0)
        time.sleep(2)
        pwm_b1.ChangeDutyCycle(0)
        pwm_b2.ChangeDutyCycle(0)
        print(f"Motor B Forward Count: {encoder_bf_count}")
        print(f"Motor B Backward Count: {encoder_bb_count}")
        input("Press Enter to test Motor B Reverse...")

        # Test Motor B Reverse
        pwm_b1.ChangeDutyCycle(0)
        pwm_b2.ChangeDutyCycle(50)
        time.sleep(2)
        pwm_b1.ChangeDutyCycle(0)
        pwm_b2.ChangeDutyCycle(0)
        print(f"Motor B Forward Count: {encoder_bf_count}")
        print(f"Motor B Backward Count: {encoder_bb_count}")

        print("Motor and Encoder Test Complete.")

    except KeyboardInterrupt:
        print("Test interrupted by user.")
    finally:
        pwm_a1.stop()
        pwm_a2.stop()
        pwm_b1.stop()
        pwm_b2.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    motor_test()
