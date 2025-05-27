# Obstacle-avoiding-robot
This Arduino-based project utilizes an ultrasonic sensor, servo motor,
buzzer, and two DC motors to create an autonomous obstacle-avoiding
robot. The robot scans the surroundings using a servo-mounted
ultrasonic sensor, detects obstacles in front, and decides to turn left
or right based on the obstacle distance. It uses a buzzer to alert
before changing directions. Motor control is achieved via the Adafruit
Motor Shield using the AFMotor library.

Pin Connections
- Ultrasonic Sensor (HC-SR04):
 - TRIGGER: A0
 - ECHO: A1
- Servo Motor:
 - Signal Pin: D9
- Buzzer:
 - Connected to: D4
- Indicator LED:
 - Connected to: D1
- Motor Shield:
 - Motor 1: M1 port (Left motor)
 - Motor 2: M4 port (Right motor)
