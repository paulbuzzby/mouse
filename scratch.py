import RPi.GPIO as GPIO
import time
import math

AIN1 = 17
AIN2 = 27
BIN1 = 22
BIN2 = 23
PWM_A = 18
PWM_B = 12

GPIO.setmode(GPIO.BCM)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)
GPIO.setup(PWM_A, GPIO.OUT)
GPIO.setup(PWM_B, GPIO.OUT)

pwm_a = GPIO.PWM(PWM_A, 1000)
pwm_b = GPIO.PWM(PWM_B, 1000)

kp = 0.5
kd = 0.2
target_position = 1000
encoder_pin = 18
ultrasonic_left_pin = 4  # GPIO pin for left ultrasonic sensor
ultrasonic_right_pin = 5  # GPIO pin for right ultrasonic sensor
log_data_buffer = []

GPIO.setup(encoder_pin, GPIO.IN)
GPIO.setup(ultrasonic_left_pin, GPIO.IN)
GPIO.setup(ultrasonic_right_pin, GPIO.IN)

def encoder_callback(channel):
    global encoder_pulse_count, motor_position
    encoder_pulse_count += 1
    motor_position = encoder_pulse_count

def calculate_pid_signal():
    global motor_position

    error = target_position - motor_position
    derivative = error - calculate_pid_signal.previous_error
    calculate_pid_signal.previous_error = error

    pid_signal = kp * error + kd * derivative

    return pid_signal

calculate_pid_signal.previous_error = 0

def read_ultrasonic_distance(pin):
    # Logic to read ultrasonic sensor distance
    # Implement your ultrasonic sensor reading code here
    return distance

def drive_motor():
    global log_data_buffer
    while motor_position < target_position:
        pid_signal = calculate_pid_signal()

        # Read ultrasonic distances
        distance_left = read_ultrasonic_distance(ultrasonic_left_pin)
        distance_right = read_ultrasonic_distance(ultrasonic_right_pin)

        # Adjust motor control based on ultrasonic sensor data
        if distance_left < safe_distance:
            # Object detected on the left, turn right
            GPIO.output(AIN1, GPIO.HIGH)
            GPIO.output(AIN2, GPIO.LOW)
            GPIO.output(BIN1, GPIO.LOW)
            GPIO.output(BIN2, GPIO.HIGH)
            pwm_a.start(pid_signal)
            pwm_b.start(pid_signal)
        elif distance_right < safe_distance:
            # Object detected on the right, turn left
            GPIO.output(AIN1, GPIO.LOW)
            GPIO.output(AIN2, GPIO.HIGH)
            GPIO.output(BIN1, GPIO.HIGH)
            GPIO.output(BIN2, GPIO.LOW)
            pwm_a.start(pid_signal)
            pwm_b.start(pid_signal)
        else:
            # No obstacles, drive straight
            GPIO.output(AIN1, GPIO.HIGH)
            GPIO.output(AIN2, GPIO.LOW)
            GPIO.output(BIN1, GPIO.HIGH)
            GPIO.output(BIN2, GPIO.LOW)
            pwm_a.start(pid_signal)
            pwm_b.start(pid_signal)

        log_data = f"Motor Position: {motor_position}, PID Signal: {pid_signal}, Distance Left: {distance_left}, Distance Right: {distance_right}\n"
        log_data_buffer.append(log_data)

        time.sleep(0.01)

    pwm_a.stop()
    pwm_b.stop()

def rotate_robot(degrees, direction):
    global motor_position, target_position

    # Convert degrees to motor position
    rotation_distance = (math.pi * degrees * wheelbase) / (360 * wheel_radius)
    target_position = motor_position + rotation_distance

    # Set the motor rotation direction
    if direction == "clockwise":
        GPIO.output(AIN1, GPIO.LOW)
        GPIO.output(AIN2, GPIO.HIGH)
        GPIO.output(BIN1, GPIO.LOW)
        GPIO.output(BIN2, GPIO.HIGH)
    elif direction == "counterclockwise":
        GPIO.output(AIN1, GPIO.HIGH)
        GPIO.output(AIN2, GPIO.LOW)
        GPIO.output(BIN1, GPIO.HIGH)
        GPIO.output(BIN2, GPIO.LOW)

    pwm_a.start(50)
    pwm_b.start(50)

def write_log_file():
    with open(log_file_path, "w") as log_file:
        for log_data in log_data_buffer:
            log_file.write(log_data)

# Example usage: Drive the motor and buffer the data
encoder_pulse_count = 0
motor_position = 0
log_file_path = "/path/to/logfile.txt"
safe_distance = 20  # Adjust the safe distance as per your requirements
wheelbase = 100  # Distance between the two wheels in mm
wheel_radius = 25  # Radius of the wheels in mm

drive_motor()

# Rotate the robot 90 degrees clockwise
rotate_robot(90, "clockwise")

# Write the buffered data to the log file
write_log_file()

# Cleanup GPIO on program exit
GPIO.cleanup()
