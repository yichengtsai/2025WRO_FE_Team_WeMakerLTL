import cv2
import time
import numpy as np
import threading
from sensor_msgs.msg import LaserScan
from function import *
import rospy
import math
from queue import Queue
import pickle
from picamera2 import Picamera2

pi = pigpio.pi()
pi.set_mode(11, pigpio.LOW)   # 先切斷
pi.set_mode(21, pigpio.LOW)

frame_queue = Queue(maxsize=2)
imcap = Picamera2()
config = imcap.create_preview_configuration(main={"size": (480, 360)})
imcap.configure(config)
imcap.start() 
imcap.set_controls({
    "AeExposureMode": 0,               # 關自動曝光
    "ExposureValue": 0.5,              # 稍微降低曝光
    "Brightness": 0.3,                 # 降低整體亮度
#     "Contrast": 0.5,                   # 增加對比
    "Saturation": 1,                 # 增加物體顏色強度
    "Sharpness": 0.6,                  # 輕微提升銳利度
    "AwbEnable": True,                # 關自動白平衡
})

reader = D100Reader()
tcs34725 = TCS34725()
bno = BNO055()

offset_gyro = 0
line_count = 0
round_count = 0
# dir_array = [357, 87, 177, 267]
dir_array = [0, 90, 180, 270]
run = True
run_1 = True
last = 0
distance_corrected = False
def read_file():
    global green_lower, green_upper, red_lower, red_upper, black_lower, black_upper, pink_lower, pink_upper, orange_lower, orange_upper, blue_lower, blue_upper, bl_lower, bl_upper
    with open('save_file/HSV_Green.p', mode='rb') as f:
        file = pickle.load(f)
    g_lower = file['Lower']
    g_upper = file['Upper']
    print('Green_Lower:' + str(g_lower))
    print('Green_Upper:' + str(g_upper))
    
    with open('save_file/HSV_Red.p', mode='rb') as f:
        file = pickle.load(f)
    r_lower = file['Lower']
    r_upper = file['Upper']
    print('Red_Lower:' + str(r_lower))
    print('Red_Upper:' + str(r_upper))
    
    with open('save_file/HSV_Pink.p', mode='rb') as f:
        file = pickle.load(f)
    p_lower = file['Lower']
    p_upper = file['Upper']
    print('Black_Lower:' + str(p_lower))
    print('Black_Upper:' + str(p_upper))
    
    with open('save_file/HSV_Orange.p', mode='rb') as f:
        file = pickle.load(f)
    o_lower = file['Lower']
    o_upper = file['Upper']
    print('Orange_Lower:' + str(o_lower))
    print('Orange_Upper:' + str(o_upper))
    
    with open('save_file/HSV_Blue.p', mode='rb') as f:
        file = pickle.load(f)
    b_lower = file['Lower']
    b_upper = file['Upper']
    print('Blue_Lower:' + str(b_lower))
    print('Blue_Upper:' + str(b_upper))
    
    with open('save_file/HSV_Black.p', mode='rb') as f:
        file = pickle.load(f)
    bl_lower = file['Lower']
    bl_upper = file['Upper']
    print('Black_Lower:' + str(bl_lower))
    print('Black_Upper:' + str(bl_upper))
    
    red_lower = np.array(r_lower, np.uint8) 
    red_upper = np.array(r_upper, np.uint8)
    green_lower = np.array(g_lower, np.uint8)
    green_upper = np.array(g_upper, np.uint8)
    pink_lower = np.array(p_lower, np.uint8)
    pink_upper = np.array(p_upper, np.uint8)
    orange_lower = np.array(o_lower, np.uint8)
    orange_upper = np.array(o_upper, np.uint8)
    blue_lower = np.array(b_lower, np.uint8)
    blue_upper = np.array(b_upper, np.uint8)
    black_lower = np.array(bl_lower, np.uint8)
    black_upper = np.array(bl_upper, np.uint8)

def gyro_read():
    global gyro, line_count, run_1
    gyro = 0
    print('gyro start')
    while run_1:
        gyro = bno.relative(dir_array[line_count])
        time.sleep(0.01)
        
def color_read():
    global lum, run
    tcs34725 = TCS34725()
    while run_1:
        lum = tcs34725.readluminance()['c']
        time.sleep(0.01)
        
def line_detect():
    global line_count, round_count, run, lum
    while run:
        while lum > 30 and run:
            time.sleep(0.001)
        _line = 1
        if line_count + 1 == 4:
            line_count = 0
        else:
            line_count += 1
        round_count+=1
        while lum <= 30 and run:
            time.sleep(0.001)  
        while lum > 30 and run:
            time.sleep(0.001)
        time.sleep(2.5)
           
def read():
    global gyro, front, left, right, run, front_1, left_1, right_1, last, right_special
    while run_1:
        ranges, angle_min, angle_increment, front_1, left_1, right_1, right_special = reader.get_latest_distances()
        if last == 1:
            front_min = 9999
            left_min = 9999
            right_min = 9999

            for i, dist in enumerate(ranges):
                if math.isnan(dist) or dist == 0:
                    continue

                # 計算當前角度（degree）
                angle = angle_min + i * angle_increment
                deg = math.degrees(angle)

                # 轉成 -180 ~ 180 的範圍
                deg = (deg + 180) % 360 - 180

                # 加上 gyro_angle（方向補償，保持方位一致）
                corrected_angle = deg + gyro

                # 前方： -15° ~ +15°
                if -15 <= corrected_angle <= 15:
                    front_min = min(front_min, dist)

                # 左方： -100° ~ -80°
                elif -100 <= corrected_angle <= -80:
                    left_min = min(left_min, dist)

                # 右方： +80° ~ +100°
                elif 80 <= corrected_angle <= 100:
                    right_min = min(right_min, dist)

            # 將距離轉為公分
            front = int(front_min*100) if front_min < 9999 else -1
            right = int(left_min*100) if left_min < 9999 else -1
            left = int(right_min*100) if right_min < 9999 else -1
            
def dis_read():
    global run_1, dis
    while run_1:
        dis = distance.read()
        time.sleep(0.1)

def mapping(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def calculate_servo_angle_red(cx, cy, bx, by):
    
    intersects, clipped_pt1, clipped_pt2 = cv2.clipLine(rect_red, (315, cy), (bx+75, by))
    print("intersects:", intersects)
#     if max_x_red!=240:
#         bx = int((max_x_red + bx)/2)
    dx_raw = bx - cx
    dx = abs(bx - cx)
    dy = abs(by - cy)
    if dx == 0:
        servo_angle = gyro * 1.2
        servo_angle = constrain(servo_angle, -50, 50)
        servo.angle(servo_angle)
        print("gyro1")
    elif bx > cx and (not intersects or b_g_y+b_g_h < 240):
        m = dy/dx
        m = constrain(m, 0, 3)
        servo_angle = mapping(m, 0, 3, 50, 10) 
        servo.angle(servo_angle)
        print("dodge:", servo_angle, "dx_raw:", dx_raw)
    elif intersects and abs(gyro) > 5:
        # 假設 clipped_pt1, clipped_pt2 是 (x,y)
        x_man_overlap = max(clipped_pt1[0], clipped_pt2[0])
        rect_right_x = rect_red[0]
        diff = x_man_overlap - rect_right_x
        if dx_raw > 0:
            dx_adjusted = dx_raw - diff - 22 # 第一象限處理
        else:
            dx_adjusted = dx_raw + diff - 22  # 第二象限處理
        print("dx, dxadjust:", dx, dx_adjusted)
        # 避免 dx_adjusted 為 0，做保護
        if abs(dx_adjusted) < 1e-5:
            dx_adjusted = 1e-5 if dx_adjusted >= 0 else -1e-5
        m = dy / dx_adjusted
        m_abs = abs(m)
        m_abs = constrain(m_abs, 0, 3)
        if dx_adjusted > 0:
            servo_angle = mapping(m_abs, 0, 3, 50, 0)
            servo.angle(servo_angle)  # 右轉
        else:
            servo_angle = mapping(m_abs, 0, 3, 30, 15)
            servo.angle(-servo_angle) # 左轉
        print("out:", servo_angle)
        
    elif abs(bx - cx) < 75 or abs(by - cy) < 60:
        servo.angle(0)    
    else:
        dx = abs(bx - cx)
        dy = abs(by - cy)
        if dx == 0:
            servo_angle = gyro * 1.2
            servo_angle = constrain(servo_angle, -50, 50)
            servo.angle(servo_angle)
            print("gyro")
        else:
            if abs(gyro) > 50:
                m = dy/dx
                m = constrain(m, 0, 3)
                servo_angle = mapping(m, 0, 3, 30, 10)
                servo.angle(-servo_angle)
                print("reverse")
            else:
                servo_angle = gyro * 1.2
                servo_angle = constrain(servo_angle, -50, 50)
                servo.angle(servo_angle)

def calculate_servo_angle_green(cx, cy, bx, by):
    
    intersects, clipped_pt1, clipped_pt2 = cv2.clipLine(rect, (175, cy), (bx-65, by))
#     if max_x_green!=240:
#          bx = int((max_x_green + bx)/2)
    dx = abs(bx - cx)
    dy = abs(by - cy)
    if dx == 0:
        servo_angle = gyro * 1.2
        servo_angle = constrain(servo_angle, -50, 50)
        servo.angle(servo_angle)
    elif bx < cx and (not intersects or b_y+b_h < 240):
        m = dy/dx
        m = constrain(m, 0, 3)
        servo_angle = mapping(m, 0, 3, 50, 10)
        servo.angle(-servo_angle)
        print("dodge:", -servo_angle)
    elif intersects and abs(gyro) > 5:
        # 假設 clipped_pt1, clipped_pt2 是 (x,y)
        x_min_overlap = min(clipped_pt1[0], clipped_pt2[0])
        rect_right_x = rect[0] + rect[2]
        diff = rect_right_x - x_min_overlap
        if dx > 0:
            dx_adjusted = dx - (diff - 15)  # 第一象限處理
        else:
            dx_adjusted = dx + (diff - 15)  # 第二象限處理
        if abs(dx_adjusted) < 1e-5:
            dx_adjusted = 1e-5 if dx_adjusted >= 0 else -1e-5
        m = dy / dx_adjusted
        m_abs = abs(m)
        m_abs = constrain(m_abs, 0, 3)
        if dx > 0:
            servo_angle = mapping(m_abs, 0, 3, 20, 10)
            servo.angle(servo_angle)  # 右轉
        else:
            servo_angle = mapping(m_abs, 0, 3, 50, 10)
            servo.angle(-servo_angle) # 左轉
        print("out:", servo_angle)
        
    elif abs(bx - cx) < 75 or abs(by - cy) < 60:
        servo.angle(0)
    else:
        if abs(gyro) > 50:
            m = dy/dx
            m = constrain(m, 0, 3)
            servo_angle = mapping(m, 0, 3, 30, 10)
        else:
            servo_angle = gyro * 1.1
            servo_angle = constrain(servo_angle, -40, 40)
        servo.angle(servo_angle)

last_error = 0
def distance_correction(right, servo, target_distance=34):
    global last_error
    
    distance = right_1*100
    error = distance - target_distance
    
    # PID 參數
    kp = 2.0
    kd = 0.5
    
    derivative = error - last_error
    last_error = error
    
    servo_angle = kp * error + kd * derivative
    servo_angle = constrain(servo_angle, -25, 25)
    
    servo.angle(int(servo_angle))
    print(servo_angle)
    return abs(error) <= 2
#     error = (right_1*100) - target_distance
#     
#     # PID參數 (可根據實際情況調整)
#     kp = 2.1  # 比例係數
#     servo_angle = int(error * kp)
#     servo_angle = constrain(servo_angle, -25, 25)
#     # 設定伺服馬達角度
#     servo.angle(servo_angle)
#     # 判斷是否到達目標位置 (容許誤差 ±0.05)
#     if abs(error) <= 0.5 and right_1 == -1:
#         return True  # 距離校正完成
#     else:
#         return False  # 仍在校正中
    
def gyro_correction():
    servo_angle = gyro * 1.4
    servo_angle = constrain(servo_angle, -45, 45)
    servo.angle(servo_angle)
    

lum=0
read_file()
if bno.begin() is not True:
    print("Error initializing device")
    exit()
time.sleep(1)
bno.setExternalCrystalUse(True)
time.sleep(1)
servo = servo_motor()
Motor = dc_motor()
distance = ultrasonic_sensor(23, 24)
threading.Thread(target=gyro_read).start()
threading.Thread(target=color_read).start()
threading.Thread(target=line_detect).start()
right=0
b_x, b_y, b_h, b_w = 0, 0, 0, 0
b_g_x, b_g_y, b_g_h, b_g_w = 0, 0, 0, 0
rect = (0, 0, 0, 0)
rect_red = (0, 0, 0, 0)
car_x = 240
car_y = 260 #260
Motor.DC(50)
last=0
Motor.DC(50)
while gyro > -80:
    servo.angle(75)
Motor.DC(0)
time.sleep(0.5)
while gyro < -10:
    servo.angle(-65)
    Motor.DC(60)
Motor.DC(0)
time.sleep(0.5)

while round_count < 13:
    frame = imcap.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
    imageFrame = frame
    imageFrame[250:360, 0:480] = [0, 0, 0]
#     imageFrame[300:360, 0:480] = [0, 0, 0]
    imageFrame[0:95, 0:480] = [0, 0, 0]
#     imageFrame[240:360, 100:380] = [0, 0, 0]
    # 偵測紅色積木並更新 grid_map
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_RGB2BGR)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    
    black_mask = cv2.inRange(hsvFrame, black_lower, black_upper)
    kernel = np.ones((5, 5), np.uint8)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
    black_mask = cv2.dilate(black_mask, kernel, iterations=2)
    black_contours, _ = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    
    max_y_bottom_red = -1
    closest_tx_red, closest_ty_red, closest_tx_green, closest_ty_green = -1, -1, -1, -1
    significant_red_contours=[]
    significant_green_contours=[]
    left_contours = []
    right_contours = []
    
    max_x_red, max_x_green = 240, 240
    for cnt in black_contours:
        area = cv2.contourArea(cnt)
        if area > 200:
            x, y, w, h = cv2.boundingRect(cnt)
            if x + w/2 < 240:  # 中心點在左半邊
                left_contours.append((area, x, y, w, h))
            else:
                right_contours.append((area, x, y, w, h))

    if left_contours:
        largest = max(left_contours, key=lambda x: x[0])
        _, x, y, w, h = largest
        if x < max_x_green:
            max_x_green = b_x + w + abs(gyro)*0.1
            w = int(w + abs(gyro)*0.2)
            rect = (x, y, w, h)
            b_x = x
            b_y = y
            b_h = h
            b_w = w
        cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (255, 255, 0), 2)
    if right_contours:
        largest = max(right_contours, key=lambda x: x[0])
        _, x, y, w, h = largest
        if x > max_x_red:
            max_x_red = int(x - abs(gyro)*0.25)
            rect_red = (max_x_red, y, w, h)
            b_g_x = x
            b_g_y = y
            b_g_h = h
            b_g_w = w
        cv2.rectangle(imageFrame, (max_x_red, y), (max_x_red + w, y + h), (255, 255, 0), 2)
        
    
    for cnt in red_contours:
        area = cv2.contourArea(cnt)
        if area > 50:#400
            x, y, w, h = cv2.boundingRect(cnt)
            tx = x + w + 100
            ty = y
            cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            y_bottom = y + h  # 輪廓底部的 y 座標
            if y_bottom > max_y_bottom_red+30:
                max_y_bottom_red = y_bottom
                closest_tx_red = x + w + 100 + int(abs(gyro)*0.15)
                closest_ty_red = y
            if closest_tx_red > 100 :#and (right > 0.25 or x < 240 or max_x_red > 240):
                significant_red_contours.append(cnt)
            cv2.line(imageFrame, (closest_tx_red, closest_ty_red), (car_x, car_y), (0, 0, 255), 2)
            cv2.line(imageFrame, (closest_tx_red+75, closest_ty_red), (315, car_y), (0, 0, 255), 2)
    
    max_y_bottom_green=-1
    for cnt in green_contours:
        area = cv2.contourArea(cnt)
        if area > 100:
            x, y, w, h = cv2.boundingRect(cnt)
            tx = x - 105
            ty = y
            cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            y_bottom = y + h  # 輪廓底部的 y 座標
            if x > 405 and abs(gyro) < 5:
                print("no")
            elif y_bottom > max_y_bottom_green+30:
                max_y_bottom_green = y_bottom
                closest_tx_green = x - 105 - int(abs(gyro)*0.15)
                closest_ty_green = y
             
            
            if closest_tx_green < 380 :#and (left > 0.25 or x > 200 or max_x_green < 240):
                significant_green_contours.append(cnt)
            cv2.line(imageFrame, (closest_tx_green, closest_ty_green), (car_x, car_y), (0, 255, 0), 2)
            cv2.line(imageFrame, (closest_tx_green-65, closest_ty_green), (175, car_y), (80, 255, 0), 2)
    
    
    if significant_red_contours and max_y_bottom_red > max_y_bottom_green+40 and not (significant_green_contours and max_y_bottom_red+40 < max_y_bottom_green):
        calculate_servo_angle_red(car_x, car_y, closest_tx_red, closest_ty_red)
    elif significant_green_contours:
        calculate_servo_angle_green(car_x, car_y, closest_tx_green, closest_ty_green)
    else:
        servo_angle = gyro * 1.4
        servo_angle = constrain(servo_angle, -50, 50)
        servo.angle(servo_angle)
    cv2.imshow("Red Block Detection with Grid", imageFrame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    Motor.DC(60)
 #////////////////////////
threading.Thread(target=read).start()
threading.Thread(target=dis_read).start()
run = False
Motor.DC(70)
dir_array = [0, 0, 0, 0]
while lum > 17:
    servo_angle = (gyro+45) * 1.4
    servo_angle = constrain(servo_angle, -45, 45)
    servo.angle(servo_angle)
Motor.DC(45)
dir_array = [180, 180, 180, 180]
while gyro < -100 or gyro > -90:
    servo_angle = gyro * 1.2
    servo_angle = constrain(servo_angle, -50, 50)
    if servo_angle > 0:
        servo_angle*=-1
    servo.angle(servo_angle)
Motor.DC(0)
time.sleep(0.5)
if front_1 > 0.42:
    Motor.DC(30)
    while front_1 > 0.42:
        servo_angle = (gyro+90) * 1.4
        servo_angle = constrain(servo_angle, -50, 50)
        servo.angle(servo_angle)
    Motor.DC(0)
    time.sleep(0.5)
Motor.DC(50)
while abs(gyro) > 5:
    servo_angle = gyro * 1.4
    servo_angle = constrain(servo_angle, -50, 50)
    servo.angle(servo_angle)
#/////////////////////////////////
last=1
Motor.DC(33)
while True:
    if (35 - right) > 2 and right != -1 and right_1 > 0.2:
        distance_corrected = distance_correction(right, servo)
        if distance_corrected:
            print("距離校正完成，切換到陀螺儀校正模式")
    else:
#         gyro_correction()
        servo_angle = (gyro+4) * 1.3
        servo_angle = constrain(servo_angle, -50, 50)
        servo.angle(servo_angle)
    if dis < 500:
        break
# Motor.DC(0)
# offset_gyro = mapping(right_1, -1, 0.2, 1, -1)
# time.sleep(0.5)
Motor.DC(35)
right_1 = 0
while dis < 1000:
    servo_angle = gyro * 1.4
    servo_angle = constrain(servo_angle, -50, 50)
    servo.angle(servo_angle)
Motor.DC(0)
time.sleep(0.5)
x1, y1 = 360, 0
x2, y2 = 315, 360
m = (y2 - y1) / (x2 - x1)
b = y1 - m * x1
start = time.time()
Motor.DC(30)
while dis >= 1000:
    frame = imcap.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    imageFrame = frame
    imageFrame[0:80, 0:480] = [0, 0, 0]
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_RGB2BGR)
    
    pink_mask = cv2.inRange(hsvFrame, pink_lower, pink_upper)
    pink_contours, _ = cv2.findContours(pink_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    for cnt in pink_contours:
        largest_cnt = max(pink_contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area > 100:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (255, 0, 255), 2)
            
            x_on_line = (y - b) / m
            dist_x = x - x_on_line
            cv2.line(imageFrame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            
            if y > 150:
                servo_angle = (gyro-2) * 1.4
                servo_angle = constrain(servo_angle, -5, 5)
                servo.angle(servo_angle)
            else:
                servo_angle = constrain(dist_x*0.2, -5, 5)
                servo.angle(servo_angle)
    
    cv2.imshow("Red Block Detection with Grid", imageFrame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

Motor.DC(0)
bno.setExternalCrystalUse(True)
time.sleep(1)
servo.angle(-70)
while gyro < 55:
    print(gyro, "gyro")
    Motor.DC(45)
Motor.DC(-10)
time.sleep(0.3)
Motor.DC(0)
set_gyro = gyro
time.sleep(0.5)
Motor.DC(-29)
while right_special > 0.2:
    servo_angle = (gyro-set_gyro-2) * 1.4
    servo_angle = constrain(servo_angle, -45, 45)
    servo.angle(-servo_angle)
Motor.DC(0)
servo.angle(-80)
time.sleep(0.5)
while abs(gyro) > 5:
    Motor.DC(-37)
    print("gyro::", gyro)
servo.angle(0)
Motor.DC(10)
time.sleep(0.5)
Motor.DC(0)
    
while front_1 > 0.12:
    Motor.DC(30)
servo.angle(0)

servo.angle(0)  
Motor.DC(0)
run_1 = False
imcap.stop()
cv2.destroyAllWindows()
