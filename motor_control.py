#!/usr/bin/env python3
"""
Raspberry Pi 4B Motor Control with YOLOv5 Object Detection
Controls a motor continuously until an object is detected, then stops the motor.
"""

import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import threading
from ultralytics import YOLO
import os

class MotorController:
    def __init__(self, motor_pin1=17, motor_pin2=18, enable_pin=27):
        """
        Initialize motor controller with GPIO pins
        
        Args:
            motor_pin1 (int): GPIO pin for motor direction 1
            motor_pin2 (int): GPIO pin for motor direction 2  
            enable_pin (int): GPIO pin for motor enable/PWM
        """
        self.motor_pin1 = motor_pin1
        self.motor_pin2 = motor_pin2
        self.enable_pin = enable_pin
        self.is_running = False
        self.pwm = None
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.motor_pin1, GPIO.OUT)
        GPIO.setup(self.motor_pin2, GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        
        # Initialize PWM for speed control
        self.pwm = GPIO.PWM(self.enable_pin, 1000)  # 1kHz frequency
        self.pwm.start(0)
        
        print("Motor controller initialized")
    
    def start_motor(self, speed=50):
        """
        Start the motor spinning
        
        Args:
            speed (int): Motor speed (0-100)
        """
        if not self.is_running:
            GPIO.output(self.motor_pin1, GPIO.HIGH)
            GPIO.output(self.motor_pin2, GPIO.LOW)
            self.pwm.ChangeDutyCycle(speed)
            self.is_running = True
            print(f"Motor started at {speed}% speed")
    
    def stop_motor(self):
        """Stop the motor"""
        if self.is_running:
            GPIO.output(self.motor_pin1, GPIO.LOW)
            GPIO.output(self.motor_pin2, GPIO.LOW)
            self.pwm.ChangeDutyCycle(0)
            self.is_running = False
            print("Motor stopped")
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop_motor()
        if self.pwm:
            self.pwm.stop()
        GPIO.cleanup()
        print("GPIO cleanup completed")

class ObjectDetector:
    def __init__(self, model_path=None, confidence_threshold=0.5):
        """
        Initialize YOLOv5 object detector
        
        Args:
            model_path (str): Path to YOLOv5 model file
            confidence_threshold (float): Minimum confidence for detection
        """
        self.confidence_threshold = confidence_threshold
        
        # Load YOLOv5 model
        if model_path and os.path.exists(model_path):
            self.model = YOLO(model_path)
        else:
            # Use default YOLOv5n model
            self.model = YOLO('yolov5n.pt')
            print("Using default YOLOv5n model")
        
        # Initialize camera
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            print("Error: Could not open camera")
            raise RuntimeError("Camera not available")
        
        print("Object detector initialized")
    
    def detect_objects(self):
        """
        Detect objects in the current camera frame
        
        Returns:
            bool: True if objects detected, False otherwise
        """
        ret, frame = self.camera.read()
        if not ret:
            print("Error: Could not read camera frame")
            return False
        
        # Run YOLOv5 detection
        results = self.model(frame, verbose=False)
        
        # Check for detections above confidence threshold
        for result in results:
            if result.boxes is not None:
                for box in result.boxes:
                    confidence = box.conf.item()
                    if confidence >= self.confidence_threshold:
                        class_id = int(box.cls.item())
                        class_name = self.model.names[class_id]
                        print(f"Object detected: {class_name} (confidence: {confidence:.2f})")
                        return True
        
        return False
    
    def release(self):
        """Release camera resources"""
        if self.camera:
            self.camera.release()

def main():
    """Main function to run motor control with object detection"""
    print("Starting Raspberry Pi Motor Control with YOLOv5 Object Detection")
    print("Press Ctrl+C to stop")
    
    # Initialize motor controller and object detector
    motor = MotorController()
    detector = ObjectDetector()
    
    try:
        # Start motor
        motor.start_motor(speed=50)
        
        # Main detection loop
        while True:
            # Check for objects
            if detector.detect_objects():
                print("Object detected! Stopping motor...")
                motor.stop_motor()
                break
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopping program...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        motor.cleanup()
        detector.release()
        print("Program terminated")

if __name__ == "__main__":
    main()
