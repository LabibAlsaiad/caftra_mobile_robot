#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import time
import numpy as np
from math import pi
import json
import threading

class EncoderCalibration:
    def __init__(self):
        # Known robot parameters (in meters)
        self.WHEEL_RADIUS = 0.04426    # 44.26mm
        self.WHEEL_BASE = 0.210        # 210mm
        
        # GPIO Setup
        self.ENCODER_LEFT_A = 23
        self.ENCODER_LEFT_B = 24
        self.ENCODER_RIGHT_A = 22
        self.ENCODER_RIGHT_B = 27
        
        # Initialize counts and calibration data
        self.left_ticks = 0
        self.right_ticks = 0
        self.calibration_data = {
            'ticks_per_rev': {'left': 0, 'right': 0},
            'wheel_radius': self.WHEEL_RADIUS,
            'wheel_base': self.WHEEL_BASE,
            'distance_per_tick': {'left': 0, 'right': 0}
        }
        
        # Threading lock
        self.lock = threading.Lock()
        
        # Setup GPIO
        self._setup_gpio()

    def _setup_gpio(self):
        """Setup GPIO pins and callbacks"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ENCODER_LEFT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENCODER_LEFT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENCODER_RIGHT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENCODER_RIGHT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        GPIO.add_event_detect(self.ENCODER_LEFT_A, GPIO.BOTH, 
                            callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.ENCODER_RIGHT_A, GPIO.BOTH, 
                            callback=self.right_encoder_callback)

    def left_encoder_callback(self, channel):
        with self.lock:
            current_A = GPIO.input(self.ENCODER_LEFT_A)
            current_B = GPIO.input(self.ENCODER_LEFT_B)
            if current_A != current_B:
                self.left_ticks += 1
            else:
                self.left_ticks -= 1

    def right_encoder_callback(self, channel):
        with self.lock:
            current_A = GPIO.input(self.ENCODER_RIGHT_A)
            current_B = GPIO.input(self.ENCODER_RIGHT_B)
            if current_A != current_B:
                self.right_ticks += 1
            else:
                self.right_ticks -= 1

    def reset_ticks(self):
        with self.lock:
            self.left_ticks = 0
            self.right_ticks = 0

    def calibrate_ticks_per_revolution(self):
        """
        Calibrate ticks per revolution by manually rotating wheels for 5 revolutions
        """
        print("\nCalibrating ticks per revolution...")
        print("We will calibrate each wheel separately for 5 complete revolutions.")
        
        for wheel in ['left', 'right']:
            print(f"\nCalibrating {wheel} wheel:")
            print("1. Mark a starting position on the wheel")
            print("2. Manually rotate the wheel EXACTLY 5 complete revolutions")
            input(f"Press Enter when ready to start {wheel} wheel calibration...")
            
            self.reset_ticks()
            input("Rotate the wheel 5 times now, then press Enter when done...")
            
            # Get tick count
            with self.lock:
                total_ticks = abs(self.left_ticks if wheel == 'left' else self.right_ticks)
            
            # Calculate ticks per revolution
            ticks_per_rev = total_ticks / 5
            self.calibration_data['ticks_per_rev'][wheel] = round(ticks_per_rev, 2)
            
            # Calculate distance per tick
            wheel_circumference = 2 * pi * self.WHEEL_RADIUS
            self.calibration_data['distance_per_tick'][wheel] = wheel_circumference / ticks_per_rev
            
            print(f"\n{wheel.capitalize()} wheel results:")
            print(f"Total ticks for 5 revolutions: {total_ticks}")
            print(f"Ticks per revolution: {ticks_per_rev:.2f}")
            print(f"Distance per tick: {self.calibration_data['distance_per_tick'][wheel]*1000:.4f} mm")

    def verify_calibration(self, test_distance=0.5):
        """
        Verify calibration by moving robot a known distance
        """
        print(f"\nVerification Test - {test_distance*1000}mm movement")
        print("Place the robot on the floor and mark start and end points")
        input("Press Enter when ready to start verification...")
        
        self.reset_ticks()
        input(f"Move the robot forward exactly {test_distance*1000}mm, then press Enter...")
        
        # Get tick counts
        with self.lock:
            left_ticks = abs(self.left_ticks)
            right_ticks = abs(self.right_ticks)
        
        # Calculate distances using calibration
        left_distance = left_ticks * self.calibration_data['distance_per_tick']['left']
        right_distance = right_ticks * self.calibration_data['distance_per_tick']['right']
        
        print("\nVerification Results:")
        print(f"Target distance: {test_distance*1000:.1f}mm")
        print(f"Left wheel - Ticks: {left_ticks}, Calculated distance: {left_distance*1000:.1f}mm")
        print(f"Right wheel - Ticks: {right_ticks}, Calculated distance: {right_distance*1000:.1f}mm")
        
        # Calculate error percentage
        left_error = abs(left_distance - test_distance) / test_distance * 100
        right_error = abs(right_distance - test_distance) / test_distance * 100
        
        print(f"\nError percentage:")
        print(f"Left wheel: {left_error:.1f}%")
        print(f"Right wheel: {right_error:.1f}%")

    def save_calibration(self, filename='robot_calibration.json'):
        """Save calibration data to file"""
        # Add some additional calculated values
        self.calibration_data['theoretical_distance_per_rev'] = 2 * pi * self.WHEEL_RADIUS * 1000  # in mm
        
        with open(filename, 'w') as f:
            json.dump(self.calibration_data, f, indent=4)
        print(f"\nCalibration data saved to {filename}")

    def run_calibration(self):
        """Run the complete calibration procedure"""
        try:
            print("Starting encoder calibration procedure...")
            print(f"Using known measurements:")
            print(f"- Wheel radius: {self.WHEEL_RADIUS*1000:.2f}mm")
            print(f"- Wheel base: {self.WHEEL_BASE*1000:.2f}mm")
            print(f"- Theoretical distance per revolution: {2*pi*self.WHEEL_RADIUS*1000:.2f}mm")
            
            # Step 1: Calibrate ticks per revolution
            self.calibrate_ticks_per_revolution()
            
            # Step 2: Verify with known distance
            self.verify_calibration()
            
            # Save calibration data
            self.save_calibration()
            
            print("\nCalibration complete!")
            
        except KeyboardInterrupt:
            print("\nCalibration interrupted by user")
        finally:
            GPIO.cleanup()

if __name__ == '__main__':
    try:
        calibrator = EncoderCalibration()
        calibrator.run_calibration()
    except Exception as e:
        print(f"Error during calibration: {e}")
        GPIO.cleanup()