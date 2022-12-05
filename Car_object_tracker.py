# Robot car object/colour tracker without PID

import cv2 as cv
import numpy as np
import pigpio
import time as t
import RPi.GPIO as GPIO

# GPIO Mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# set GPIO Pins for Servo drive motors
RPIN = 17 ## GPIO17, or pin 11 in Raspberrypi
LPIN = 18 ## GPIO18, or pin 12 in Raspberrypi

r = pigpio.pi()
l = pigpio.pi()

if not r.connected:
    exit()

if not l.connected:
    exit()


def backward():
    r.set_servo_pulsewidth(RPIN, 1000) # safe anti-clockwise
    l.set_servo_pulsewidth(LPIN, 2000) # safe anti-clockwise

def forward():
    r.set_servo_pulsewidth(RPIN, 2000) # safe anti-clockwise
    l.set_servo_pulsewidth(LPIN, 1000) # safe anti-clockwise

def stop():
    r.set_servo_pulsewidth(RPIN, 0) # off, stop
    l.set_servo_pulsewidth(LPIN, 0) # off, stop
def right():
    r.set_servo_pulsewidth(RPIN, 0) # off, stop
    l.set_servo_pulsewidth(LPIN, 1000) # safe anti-clockwise
def left():
    r.set_servo_pulsewidth(RPIN, 2000) # off, stop
    l.set_servo_pulsewidth(LPIN, 0) # safe anti-clockwise


# Run camera with Open cv
cap = cv.VideoCapture(0)
width = 320
height = 240
x_c = int(width / 2)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv.resize(frame, (width, height))

    hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)


    # Track colour

    lower = np.array([0, 71, 255])
    higher = np.array([255, 255, 255])
    mask = cv.inRange(hsv_frame, lower, higher)

# Find contours
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv.contourArea(contour)
        if area > 1000:
            x, y, w, h = cv.boundingRect(contour)
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 1)
            x_m = int(x + w/2)
            y_m = int(y + h/2)
            cv.circle(frame, (x_m, y_m), 5, (0, 0, 255), 1)
            if x_m < x_c:
                right()
            elif x_m > x_c:
                left()
            if area > 3000:
                backward()
            elif area < 3000:
                forward()
        
        cv.imshow('frame',frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            stop()
            break

stop()
t.sleep(0.5)
cap.release()

cv.destroyAllWindows()