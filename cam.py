# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from time import sleep
import cv2
import matplotlib
import matplotlib.pyplot as plt

matplotlib.use('Agg')
camera = PiCamera()
camera.start_preview()
sleep(0.1)
camera.capture('image.jpg')
camera.stop_preview()

# Load the image 
image = cv2.imread('image.jpg') 
image = cv2.rotate(image, cv2.ROTATE_180)
cv2.imwrite("image.jpg", image)

# Crop the image
cropped_image = image[:200, 300:-200]

# Save the cropped image
cv2.imwrite("cropped_image.jpg", cropped_image)
  
# Convert the image to grayscale
gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
cv2.imwrite("gray_image.jpg", gray)

# Apply gaussian blur
blurred_image = cv2.GaussianBlur(gray, (101, 101), 0)
cv2.imwrite("blurred_image.jpg", gray)

# MY Threshold
# Set the lower and upper thresholds (anything below the lower and above the upper threshold will ultimatley be turned to black)
lower = 200
upper = 220
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
cv2.drawContours(cropped_image, contours, -1, (0, 255, 0), 2)
cv2.imwrite("contour_image.jpg", cropped_image)

# Detect line contour of creaseh
if contours:
    print("Contours found.")

else:
    print("No contours found.")