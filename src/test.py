import cv2
import numpy as np
import threading
from function import *
from queue import Queue
import pickle
from picamera2 import Picamera2

pi = pigpio.pi()

def Motor_test():
    Motor = dc_motor()
    Motor.DC(50)
    time.sleep(3)
    Motor.DC(0)
    
def Servo_test():
    servo = servo_motor()
    servo.angle(40)
    time.sleep(2)
    servo.angle(0)
    
def sensor_test():
    tcs34725 = TCS34725()
    reader = D100Reader()
    bno = BNO055()
    if bno.begin() is not True:
        print("Error initializing device")
        exit()
    time.sleep(1)
    bno.setExternalCrystalUse(True)
    while True:
        print(f"rel:{bno.relative(0)} abs:{int(bno.getVector(bno.VECTOR_EULER)[0])} lum:{tcs34725.readluminance()['c']}")
        front, left, right = reader.get_latest_distances()
        print(f"front:{front:.2f} left:{left:.2f} right:{right:.2f}")
        time.sleep(0.01)

if __name__ == "__main__":
    Servo_test()
    Motor_test()
    sensor_test()
