# imports
import RPi.GPIO as GPIO 
import time
import RPistepper as stp
import threading
from picamera.array import PiRGBArray
from picamera import PiCamera
from time import sleep
import cv2
import matplotlib

## GPIO Pins
# Limit switches
limit_front1 = 20 
limit_front2 = 20
limit_front3 = 21
limit_right = 24

# input button
start_pin = 27
estop_pin = 10
stop_flag = threading.Event()

# Servos 
servo_front_left_pin = 11
servo_front_right_pin = 15
servo_rear_left_pin = 22   
servo_rear_right_pin = 8 
servo_left_pin = 14       
servo_right_pin = 9   

# Stepper motors
front_timing_belt_stepper_step = 0
front_timing_belt_stepper_direction = 5
rear_timing_belt_stepper_step = 19
rear_timing_belt_stepper_direction = 26
major_pusher_stepper_step = 1
major_pusher_stepper_direction = 12
minor_pusher_stepper_step = 6
minor_pusher_stepper_direction = 13
bed_stepper_step = 17 
bed_stepper_direction = 4 
# big stepper -> black = direction, red = 5v, green = ground, red/brown = pulse
big_bed_stepper_step = 23 
big_bed_stepper_direction = 25
big_bed_stepper_enable = 7

# Servo objects
servo_front_left = None
servo_front_right = None
servo_left = None
servo_right = None
servo_rear_right = None
servo_rear_left = None

# Step counters
bed_steps = 0
big_bed_steps = 0
minor_pusher_steps = 0
major_pusher_steps = 0
front_timing_belt_steps = 0
rear_timing_belt_steps = 0

# Camera
camera = PiCamera()
camera.start_preview()

def stepper_forward():
    # Stepper motor directions
    # high = counterclockwise (away from motor on lead screw), low = clcokwise, (towards motor on lead screw)
    GPIO.output(front_timing_belt_stepper_direction, GPIO.LOW)
    GPIO.output(rear_timing_belt_stepper_direction, GPIO.HIGH)
    GPIO.output(major_pusher_stepper_direction, GPIO.HIGH)
    GPIO.output(minor_pusher_stepper_direction, GPIO.HIGH)
    GPIO.output(bed_stepper_direction, GPIO.LOW) # low = down
    GPIO.output(big_bed_stepper_direction, GPIO.LOW) # low = down

def stepper_reverse():
    # Stepper motor directions
    # high = counterclockwise (away from motor on lead screw), low = clcokwise, (towards motor on lead screw)
    GPIO.output(front_timing_belt_stepper_direction, GPIO.HIGH)
    GPIO.output(rear_timing_belt_stepper_direction, GPIO.LOW)
    GPIO.output(major_pusher_stepper_direction, GPIO.LOW)
    GPIO.output(minor_pusher_stepper_direction, GPIO.LOW)
    GPIO.output(bed_stepper_direction, GPIO.HIGH) # high = up
    GPIO.output(big_bed_stepper_direction, GPIO.HIGH) # high = up

def initialize_GPIO():
    global servo_front_left, servo_front_right, servo_left, servo_right, servo_rear_right, servo_rear_left

    GPIO.setmode(GPIO.BCM)

    # Limit switches
    GPIO.setup(limit_front1, GPIO.IN)
    GPIO.setup(limit_front2, GPIO.IN)
    GPIO.setup(limit_front3, GPIO.IN)
    GPIO.setup(limit_right, GPIO.IN)

    # Input button
    GPIO.setup(start_pin, GPIO.IN)
    GPIO.setup(estop_pin, GPIO.IN)

    # Servos
    GPIO.setup(servo_front_left_pin,GPIO.OUT)
    GPIO.setup(servo_front_right_pin,GPIO.OUT)
    GPIO.setup(servo_right_pin,GPIO.OUT)
    GPIO.setup(servo_left_pin,GPIO.OUT)
    GPIO.setup(servo_rear_right_pin,GPIO.OUT)
    GPIO.setup(servo_rear_left_pin, GPIO.OUT)

    servo_front_left = GPIO.PWM(servo_front_left_pin,40)
    servo_front_right = GPIO.PWM(servo_front_right_pin,40)
    servo_right = GPIO.PWM(servo_right_pin,40) 
    servo_left = GPIO.PWM(servo_left_pin,40)
    servo_rear_right = GPIO.PWM(servo_rear_right_pin,40)
    servo_rear_left = GPIO.PWM(servo_rear_left_pin, 40)

    # Stepper motors
    GPIO.setup(front_timing_belt_stepper_step, GPIO.OUT)
    GPIO.setup(front_timing_belt_stepper_direction, GPIO.OUT)
    GPIO.setup(rear_timing_belt_stepper_step, GPIO.OUT)
    GPIO.setup(rear_timing_belt_stepper_direction, GPIO.OUT)
    GPIO.setup(major_pusher_stepper_step, GPIO.OUT)
    GPIO.setup(major_pusher_stepper_direction, GPIO.OUT)
    GPIO.setup(minor_pusher_stepper_step, GPIO.OUT)
    GPIO.setup(minor_pusher_stepper_direction, GPIO.OUT)
    GPIO.setup(bed_stepper_step, GPIO.OUT)
    GPIO.setup(bed_stepper_direction, GPIO.OUT)
    GPIO.setup(big_bed_stepper_step, GPIO.OUT)
    GPIO.setup(big_bed_stepper_direction, GPIO.OUT)
    GPIO.setup(big_bed_stepper_enable, GPIO.OUT)

    # Stepper motor directions
    stepper_forward()

    # Safety for the big bed stepper motor
    GPIO.output(big_bed_stepper_enable, GPIO.LOW) 


def start_button():
    if GPIO.input(start_pin):
        return True
    
def estop():
    if GPIO.input(estop_pin):
        return True

def move_stepper(stepper, steps, delay):

    for x in range(steps):
        GPIO.output(stepper, GPIO.HIGH)
        time.sleep(delay)  

        GPIO.output(stepper, GPIO.LOW)
        time.sleep(delay)  

def major_pusher():
    global major_pusher_steps

    # Move major pusher until limit switch pressed
    while ((GPIO.input(limit_front1) == 1) and (GPIO.input(limit_front2) == 1) and (GPIO.input(limit_front3) == 1)):
        move_stepper(major_pusher_stepper_step,1, 0.0005)
        major_pusher_steps += 1
    
    return

def minor_pusher():
    global minor_pusher_steps

    # Move minor pusher until limit switch pressed
    while (GPIO.input(limit_right) == 1):
        move_stepper(minor_pusher_stepper_step,1, 0.0005)
        minor_pusher_steps += 1
    
    return

def take_picture():
    camera.capture('image.jpg')
    camera.stop_preview()

    # Load the image 
    image = cv2.imread('image.jpg') 
    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    cv2.imwrite("image.jpg", image)
    
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.imwrite("gray_image.jpg", gray)

    # Apply gaussian blur
    blurred_image = cv2.GaussianBlur(gray, (101, 101), 0)
    cv2.imwrite("blurred_image.jpg", gray)

    # MY Threshold
    # Set the lower and upper thresholds (anything below the lower and above the upper threshold will ultimatley be turned to black)
    lower = 178
    upper = 180
    # Binary thresholding to set values below the lower threshold to 0 and others to maxValue (255)
    _, threshold_lower = cv2.threshold(blurred_image, lower, 255, cv2.THRESH_BINARY)
    cv2.imwrite("threshold_lower.jpg", threshold_lower)
    # Binary thresholding to set values above the upper threshold to 0 and others to maxValue (255)
    _, threshold_upper = cv2.threshold(blurred_image, upper, 255, cv2.THRESH_BINARY_INV)
    cv2.imwrite("threshold_upper.jpg", threshold_upper)
    # Combine the two thresholded images using bitwise AND
    thresh = cv2.bitwise_and(threshold_lower, threshold_upper)
    cv2.imwrite("threshold.jpg", thresh)

    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw the contours on the original image
    cv2.drawContours(image, contours, -1, (0, 255, 0), 2)
    cv2.imwrite("contour_image.jpg", image)

    # Detect line contour of crease
    if contours:
        print("Contours found.")

    else:
        print("No contours found.")

def lower_box():
    take_picture()

def move_box():
    print("Major pusher")
    major_pusher()
    print("Minor pusher")
    minor_pusher()
    lower_box()

def move_timing_belts():
    global front_timing_belt_steps, rear_timing_belt_steps


    front_timing_belt_steps = int(((25 - (minor_pusher_steps * (0.001575)) - 3) / 3) / 0.00785)
    #front_timing_belt_steps = 0
    rear_timing_belt_steps = front_timing_belt_steps

    move_stepper(front_timing_belt_stepper_step, front_timing_belt_steps, 0.001)
    move_stepper(rear_timing_belt_stepper_step, rear_timing_belt_steps, 0.001)
    return

def fold_box():
    print("Servos Starting")
    servo_front_left.start(0)
    servo_front_right.start(0)
    servo_rear_left.start(0)
    servo_rear_right.start(0)
    servo_left.start(0)
    servo_right.start(0)

    print("Initial Left and Right Servos")
    servo_left.ChangeDutyCycle(6)
    servo_right.ChangeDutyCycle(2)
    time.sleep(1)

    print("Intiial Front and Rear Servos")
    servo_front_right.ChangeDutyCycle(8)
    servo_front_left.ChangeDutyCycle(2)
    servo_rear_left.ChangeDutyCycle(2)
    servo_rear_right.ChangeDutyCycle(8)
    time.sleep(1)

    print("Left and Right Servos going down")
    servo_left.ChangeDutyCycle(2)
    servo_right.ChangeDutyCycle(8)
    time.sleep(1)

    print("Front and Rear Servos going halfway")
    servo_front_right.ChangeDutyCycle(5)
    servo_front_left.ChangeDutyCycle(4)
    servo_rear_left.ChangeDutyCycle(4)
    servo_rear_right.ChangeDutyCycle(5)
    time.sleep(1)

    print("Left and Right Servos going up")
    servo_left.ChangeDutyCycle(8)
    servo_right.ChangeDutyCycle(2)
    time.sleep(1)
    servo_left.stop()
    servo_right.stop()

    print("Front and Rear Servos going down")
    servo_front_right.ChangeDutyCycle(2)
    servo_front_left.ChangeDutyCycle(8)
    servo_rear_left.ChangeDutyCycle(7)
    servo_rear_right.ChangeDutyCycle(3)
    time.sleep(1)

    print("Front and Rear Servos going up")
    servo_front_right.ChangeDutyCycle(8)
    servo_front_left.ChangeDutyCycle(2)
    servo_rear_left.ChangeDutyCycle(2)
    servo_rear_right.ChangeDutyCycle(8)
    time.sleep(1)

    servo_front_right.stop()
    servo_front_left.stop()
    servo_rear_right.stop()
    servo_rear_left.stop()

    return

def lower_platform():
    global bed_steps
    
    GPIO.output(big_bed_stepper_enable, GPIO.HIGH) 

    for x in range(2000):
    # print(x)
        for y in range(16):
            move_stepper(big_bed_stepper_step, 1, 0.0006)
        move_stepper(bed_stepper_step, 1, 0.001)
        bed_steps += 1
    
    GPIO.output(big_bed_stepper_enable, GPIO.LOW) 


def reset():
    global front_timing_belt_steps, major_pusher_steps, minor_pusher_steps, rear_timing_belt_steps, bed_steps

    stepper_reverse()

    while front_timing_belt_steps > 0:
        move_stepper(front_timing_belt_stepper_step, 1, 0.001)
        front_timing_belt_steps -= 1

    while major_pusher_steps > 0:
        move_stepper(major_pusher_stepper_step, 1, 0.001)
        major_pusher_steps -= 1

    while minor_pusher_steps > 0:
        move_stepper(minor_pusher_stepper_step, 1, 0.001)
        minor_pusher_steps -= 1

    while rear_timing_belt_steps > 0:
        move_stepper(rear_timing_belt_stepper_step, 1, 0.001)
        rear_timing_belt_steps -= 1

    GPIO.output(big_bed_stepper_enable, GPIO.HIGH) 
    while bed_steps > 0:
        for y in range(16):
            move_stepper(big_bed_stepper_step, 1, 0.0006)
        move_stepper(bed_stepper_step, 1, 0.001)
        bed_steps -= 1
    GPIO.output(big_bed_stepper_enable, GPIO.LOW) 
    

    print("Servos Starting")
    servo_front_left.start(0)
    servo_front_right.start(0)
    servo_rear_left.start(0)
    servo_rear_right.start(0)
    servo_left.start(0)
    servo_right.start(0)

    print("Reseting Servos")
    servo_left.ChangeDutyCycle(8)
    servo_right.ChangeDutyCycle(2)
    servo_front_right.ChangeDutyCycle(8)
    servo_front_left.ChangeDutyCycle(2)
    servo_rear_left.ChangeDutyCycle(2)
    servo_rear_right.ChangeDutyCycle(8)
    time.sleep(1)

    print("Stopping Servos")
    servo_front_right.stop()
    servo_front_left.stop()
    servo_rear_right.stop()
    servo_rear_left.stop()

    return

def handle_estop():
    reset()
    GPIO.cleanup()
    stop_flag.set()  

def main():
    try:
        initialize_GPIO()
        print("initialized")

        # Create a thread to check estop condition in the background
        #estop_thread = threading.Thread(target=check_estop)
        #estop_thread.start()

        #while not stop_flag.is_set():
        while True:
            if start_button():
                move_box()
                move_timing_belts()
                fold_box()
                #lower_platform()
                reset()
                stepper_forward()

    except KeyboardInterrupt:
        reset()
        GPIO.cleanup()
    finally:
        GPIO.cleanup()

def check_estop():
    while not stop_flag.is_set():
        if estop():
            handle_estop()
            break


if __name__ == "__main__":
    main()

