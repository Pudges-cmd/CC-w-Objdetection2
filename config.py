"""
Configuration file for Raspberry Pi Motor Control with YOLOv5
Adjust these settings according to your hardware setup and requirements.
"""

# Motor Configuration
MOTOR_PIN1 = 17      # GPIO pin for motor direction 1
MOTOR_PIN2 = 18      # GPIO pin for motor direction 2
ENABLE_PIN = 27      # GPIO pin for motor enable/PWM
MOTOR_SPEED = 50     # Motor speed (0-100)

# Object Detection Configuration
CONFIDENCE_THRESHOLD = 0.5    # Minimum confidence for object detection (0.0-1.0)
MODEL_PATH = None             # Path to custom YOLOv5 model (None for default)
DETECTION_INTERVAL = 0.1      # Time between detections in seconds

# Camera Configuration
CAMERA_INDEX = 0              # Camera device index (usually 0 for default camera)
CAMERA_WIDTH = 640            # Camera resolution width
CAMERA_HEIGHT = 480           # Camera resolution height

# GPIO Configuration
GPIO_MODE = "BCM"             # GPIO mode: "BCM" or "BOARD"

# Detection Classes (optional - for specific object detection)
# Leave empty to detect all objects, or specify class names to detect only certain objects
TARGET_CLASSES = []           # e.g., ["person", "car", "bottle"]

# Logging Configuration
ENABLE_LOGGING = True         # Enable/disable logging
LOG_LEVEL = "INFO"           # Log level: DEBUG, INFO, WARNING, ERROR
