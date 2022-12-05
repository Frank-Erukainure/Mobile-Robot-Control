# Controlling Raspberry Pi
#FRANK EFE ERUKAINURE
#PID control of Mobile Robot with ultrasonic sensor

import pigpio  # for servo control
import time as t
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
# GPIO.setmode(GPIO.BOARD)

RPIN = 17  # GPIO17, pin 11
LPIN = 18  # GPIO18, pin 12

r = pigpio.pi()
l = pigpio.pi()

if not r.connected:
    exit()

if not l.connected:
    exit()

TRIGGER = 24  # GPIO24 or pin 18
ECHO = 8  # GPIO8, OR pin 24


def get_distance(TRIGGER, ECHO):
    GPIO.setup(TRIGGER, GPIO.OUT)

    # cleanup output
    GPIO.output(TRIGGER, False)

    t.sleep(0.000002)

    # set Trigger to HIGH
    GPIO.output(TRIGGER, True)

    # set Trigger to LOW
    t.sleep(0.000005)
    GPIO.output(TRIGGER, False)

    GPIO.setup(ECHO, GPIO.IN)

    # save StartTime
    while GPIO.input(ECHO) == 0:
        t_start = t.time()

    # save time of arrival
    while GPIO.input(ECHO) == 1:
        t_stop = t.time()

    # time difference between start and arrival
    t_elapsed = t_stop - t_start
    # multiply with the sonic speed (34300 cm/s) and divide by 2, because there and back
    distance = (t_elapsed * 34300) / 2

    return round(distance, 2)


def get_PID(d):
    D = float(20)
    ep = float(0)
    ea = float(0)
    tp = float(0)
    Kp = float(1)
    Ki = float(0.0001)
    Kd = float(0.5)

    # obtain current time
    t1 = t.time()

    e = float(D - d)

    ea = float(ea + e)

    dt = float(t1 - tp)
    # calculate PID value
    u = Kp * e + Kd * ((e - ep) / dt) + Ki * ea * dt

    ep = e
    tp = t1

    if u >= 20:
        u = 20
    if u <= -20:
        u = -20

    return u


def stop():
    r.set_servo_pulsewidth(RPIN, 0)
    l.set_servo_pulsewidth(LPIN, 0)
    
    
def f_map(u, u_min, u_max, out_min, out_max):
    return int((u - u_min) * (out_max - out_min) / (u_max - u_min) + out_min)

while True:
    # getting current distance from sensor
    dist = get_distance(TRIGGER, ECHO)
    print("Distance = ")
    print(dist)
    t.sleep(0.05)

    # Calculate PID value
    u = get_PID(dist)
    print("PID_val = ")
    print(u)
    t.sleep(0.05)

    # map PID value to motor speed
    f = f_map(u, -20, 20, 1000, 2000)
    print("Speed = ")
    print(f)
    t.sleep(0.05)

    # write speed to motors
    r.set_servo_pulsewidth(RPIN, 3000 - f)
    l.set_servo_pulsewidth(LPIN, f)
    
    if d > 500:
        stop()
    


