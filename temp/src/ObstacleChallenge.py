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

frame_queue = Queue(maxsize=2)
imcap = Picamera2()
config = imcap.create_preview_configuration(main={"size": (480, 360)})
imcap.configure(config)
imcap.start() 
imcap.set_controls({
    "AeExposureMode": 0, 
    "ExposureValue": 1,   
    "Brightness": 0.3,               
})

reader = D100Reader()
servo = servo_motor()
servo_left = servo_left_motor()
servo_right = servo_right_motor()
Motor = dc_motor()
tcs34725 = TCS34725()
bno = BNO055()
pi = pigpio.pi()
dir_array = [355, 85, 175, 265]
run = True
line_count = 0
gyro_kp = 0.9
round_count = 0

avoidance_active = False
avoidance_start_time = 0
state = "NORMAL"
state_green = "NORMAL"

last=0
gy=0
dodge=0

def read_file():
    global green_lower, green_upper, red_lower, red_upper, black_lower, black_upper, pink_lower, pink_upper, orange_lower, orange_upper, blue_lower, blue_upper
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

def gyro_read():
    global gyro, gyro_angle, line_count, run
    gyro = 0
    print('gyro start')
    while run:
        gyro = bno.relative(dir_array[line_count])
        time.sleep(0.01)
        
def color_read():
    global lum, run
    tcs34725 = TCS34725()
    while run:
        lum = tcs34725.readluminance()['c']
        time.sleep(0.01)
        
def line_detect():
    global line_count, round_count, run, lum
    while run:
        while lum > 30 and run:
            time.sleep(0.001)
        Motor.DC(60)
        if line_count + 1 == 4:
            line_count = 0
        else:
            line_count += 1
        round_count += 1
        if round_count == 13:
            break
        time.sleep(1)
        Motor.DC(60)
        time.sleep(2)
        

def is_block_at_left_bottom(norm_x, norm_y, x_threshold=0.2, min_y_threshold=0.5, max_y_threshold=0.85):
    """
    norm_x 越小 → y_threshold 越大 → 越晚結束
    """
    if norm_x > x_threshold:
        return False  # 不夠左，不進入判斷
    
    # 映射：norm_x=0 → max_y_threshold, norm_x=x_threshold → min_y_threshold
    scale = min(norm_x / x_threshold, 1.0)
    dynamic_y_threshold = max_y_threshold - (max_y_threshold - min_y_threshold) * scale
    print(norm_y, dynamic_y_threshold)
    return norm_y > dynamic_y_threshold

def is_block_at_right_bottom(norm_x, norm_y, x_threshold=1.23, min_y_threshold=0.75, max_y_threshold=0.95):
    """
    norm_x 越小 → y_threshold 越大 → 越晚結束
    x_threshold 預設調大，讓結束條件更靠右側才生效
    """
    if norm_x < x_threshold:
        return False  # 還沒到更右邊，不結束
    
    # 映射：norm_x=0 → max_y_threshold, norm_x=x_threshold → min_y_threshold
    scale = min(norm_x / x_threshold, 1.0)  # 限制最大為1
    dynamic_y_threshold = max_y_threshold - (max_y_threshold - min_y_threshold) * scale
    
    return norm_y < dynamic_y_threshold

def need_avoid(norm_x, norm_y):
    if norm_x > 0.3:  
        return True
    return False

def need_avoid_green(norm_x, norm_y):
    if norm_x < 0.9:  
        return True
    return False

def avoid_red_block(red_contours, frame):
    global state
    """
    根據積木位置與距離控制伺服角度：偏移決定方向，距離決定加強程度。
    """
    if not red_contours:
        # 沒有積木，正常循跡
        state = "NORMAL"
        servo_angle = gyro * 1.2
        servo_angle = constrain(servo_angle, -40, 40)
        servo.angle(servo_angle)
        return

    # 找出最大紅色積木
    largest_contour = max(red_contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    block_center_x = x + w // 2
    block_center_y = y + h // 2

    # 正規化積木位置
    frame_width = frame.shape[1]
    frame_height = frame.shape[0]
    norm_x = block_center_x / frame_width
    norm_y = block_center_y / frame_height
    frame_center_x = frame_width // 2

    # 左右偏移程度（-1 ~ 1）
    offset_ratio = (block_center_x - frame_center_x) / (frame_width // 2)
    if norm_x > 0.3 and norm_y > 0.4 and abs(gyro) > 30:
        extra_angle = 25
    elif norm_y > 0.5:
        extra_angle = 10
    else:
        extra_angle = 0
    final_angle = (52 * (offset_ratio + 0.8) / 1.6) + extra_angle
    final_angle = constrain(final_angle, 0, 50)
    
    if state == "NORMAL":
        if need_avoid(norm_x, norm_y):
            state = "AVOIDING"
            servo.angle(final_angle)
        else:
            servo_angle = gyro * 1.2
            servo_angle = constrain(servo_angle, -40, 40)
            servo.angle(servo_angle)

    elif state == "AVOIDING":
        if is_block_at_left_bottom(norm_x, norm_y):  # 綠色結束條件：積木右下角滑出
            state = "NORMAL"
            servo_angle = gyro * 1.2
            servo_angle = constrain(servo_angle, -40, 40)
            servo.angle(servo_angle)
        else:
            servo.angle(final_angle)
        
def avoid_green_block(green_contours, frame):
    global state_green
    """
    綠色積木閃避：一律左閃，右側視為安全。
    """
    if not green_contours:
        state_green = "NORMAL"
        servo_angle = gyro * 1.2
        servo_angle = constrain(servo_angle, -40, 40)
        servo.angle(servo_angle)
        return

    largest_contour = max(green_contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    block_center_x = x + w // 2
    block_center_y = y + h // 2

    frame_width = frame.shape[1]
    frame_height = frame.shape[0]
    norm_y = block_center_y / frame_height
    norm_x = block_center_x / frame_height
    frame_center_x = frame_width // 2

    offset_ratio = (frame_center_x - block_center_x) / (frame_width // 2)

    if norm_y > 0.5 and norm_x < 0.8 and abs(gyro) > 30:
        extra_angle = 25
    elif norm_y > 0.5 and norm_x < 0.8:
        extra_angle = 10
    else:
        extra_angle = 0
    print(extra_angle, gyro)
    total_angle = (52 * (offset_ratio + 0.8) / 1.6) + extra_angle
    final_angle = constrain(total_angle, 0, 50)

    # 左閃，轉負角度
    final_angle = -final_angle

    if state_green == "NORMAL":
        if need_avoid_green(norm_x, norm_y):
            state_green = "AVOIDING"
            servo.angle(final_angle)
        else:
            servo_angle = gyro * 1.2
            servo_angle = constrain(servo_angle, -40, 40)
            servo.angle(servo_angle)

    elif state_green == "AVOIDING":
        if is_block_at_right_bottom(norm_x, norm_y):  # 綠色結束條件：積木右下角滑出
            state_green = "NORMAL"
            servo_angle = gyro * 1.2
            servo_angle = constrain(servo_angle, -40, 40)
            servo.angle(servo_angle)
        else:
            servo.angle(final_angle)
            
def read():
    global gyro, front, left, right, run
    while run:
        ranges, angle_min, angle_increment = reader.get_latest_distances()

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
            
def is_block_in_center(norm_x_red, norm_x_green):
    # 中央區域容許範圍（根據畫面大小自定義）
    return norm_x_red > 0.4 or norm_x_green > 0.4

def is_block_in_center_right(norm_x_red, norm_x_green):
    # 中央區域容許範圍（根據畫面大小自定義）
    return norm_x_red < 0.6 or norm_x_green < 0.6

significant_red_contours = []
significant_green_contours = []
def block_detect():
    global px, py, run, ry, gy, last, norm_x_green, norm_x_red, significant_green_contours, significant_red_contours
    while run:
        if not frame_queue.empty():
            imageFrame = frame_queue.get()

            imageFrame[0:90, 0:640] = [0, 0, 0]
            hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_RGB2BGR)  # 先轉回 BGR

            red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
            green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
            pink_mask = cv2.inRange(hsvFrame, pink_lower, pink_upper)
            
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            pink_contours, pink_hierarchy = cv2.findContours(pink_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

            significant_red_contours = []
            for contour in red_contours:
                area = cv2.contourArea(contour)
                if area > 500:
                    rx, ry, rw, rh = cv2.boundingRect(contour)
                    cv2.rectangle(imageFrame, (rx, ry), (rx + rw, ry + rh), (0, 0, 255), 2)
                    x, y, w, h = cv2.boundingRect(contour)
                    cx = x + w / 2
                    cy = y + h / 2
                    norm_x_red = cx / 480
                    norm_x = cx / 480
                    norm_y = cy / 360
                    if norm_x < 0.15 and abs(gyro) < 10 or norm_y < 0.2 and abs(gyro) < 10:
                        print("no block")
                    else:
                        significant_red_contours.append(contour)
               
                        
            significant_green_contours = []  
            for contour in green_contours:
                area = cv2.contourArea(contour)
                if area > 200:
                    gx, gy, gw, gh = cv2.boundingRect(contour)
                    cv2.rectangle(imageFrame, (gx, gy), (gx + gw, gy + gh), (0, 255, 0), 2)
                    x, y, w, h = cv2.boundingRect(contour)
                    cx = x + w / 2
                    cy = y + h / 2
                    norm_x_green = cx / 480
                    norm_x = cx / 480
                    norm_y = cy / 360
                    if norm_x > 0.9 and abs(gyro) < 10 or norm_y > 0.85 and abs(gyro) < 10:
                        print("no block")
                    else:
                        significant_green_contours.append(contour)
                else:
                    gy = 0
                       
#             for pic, contour in enumerate(pink_contours): 
#                 area = cv2.contourArea(contour)
#                 if(area > 500):
#                     px, py, w, h = cv2.boundingRect(contour)
#                     pink_area = w*h
#                     imageFrame = cv2.rectangle(imageFrame, (px, py), (px + w, py + h), (255, 255, 0), 2) 
            
#             if last == 1 or dodge == 1:
            cv2.imshow("Detect", imageFrame)
            if cv2.waitKey(10) & 0xFF == ord('q'): 
                imcap.stop()
                run = False
                cv2.destroyAllWindows()
                break 
            
def go_run():
    global run, right, last, dodge, gyro, ry, gy, significant_red_contours, significant_green_contours, imageFrame
    while run:
        if not frame_queue.empty():
            imageFrame = frame_queue.get()
            imageFrame[0:100, 0:640] = [0, 0, 0]
        if last == 1:
            if right is not None and abs(right - 36) > 3 and right != -1:
                print("bad", f"Right: {right} cm")
                distance_corrected = distance_correction(right, servo)
                if distance_corrected:
                    print("距離校正完成，切換到陀螺儀校正模式")
            elif last==1:
                print("gyro")
                gyro_correction()
        
        if dodge == 1:
            if significant_red_contours and ry > gy:
                avoid_red_block(significant_red_contours, imageFrame)
            elif significant_green_contours:
                avoid_green_block(significant_green_contours, imageFrame)
                print("dodge")
            elif dodge == 1:
                print("gyro")
                servo_angle = gyro * 1.3
                servo_angle = constrain(servo_angle, -40, 40)
                servo.angle(servo_angle)
            
def once_detect(frame):
    global norm_x_green, norm_x_red
    imageFrame = frame

    imageFrame[0:100, 0:640] = [0, 0, 0]
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_RGB2BGR)  # 先轉回 BGR

    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    significant_red_contours = []
    for contour in red_contours:
        area = cv2.contourArea(contour)
        if area > 500:
            rx, ry, rw, rh = cv2.boundingRect(contour)
            cv2.rectangle(imageFrame, (rx, ry), (rx + rw, ry + rh), (0, 0, 255), 2)
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w / 2
            cy = y + h / 2
            norm_x_red = cx / 480
            norm_x = cx / 480
            norm_y = cy / 360
                
    significant_green_contours = []  
    for contour in green_contours:
        area = cv2.contourArea(contour)
        if area > 400:
            gx, gy, gw, gh = cv2.boundingRect(contour)
            cv2.rectangle(imageFrame, (gx, gy), (gx + gw, gy + gh), (0, 255, 0), 2)
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w / 2
            cy = y + h / 2
            norm_x_green = cx / 480
            norm_x = cx / 480
            norm_y = cy / 360

orange_y=0
blue_y=0
def left_right_detect(frame):
    global blue_y, orange_y
    
    imageFrame = frame
    
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_RGB2BGR) 
    
    orange_mask = cv2.inRange(hsvFrame, orange_lower, orange_upper)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
    orange_contours, orange_hierarchy = cv2.findContours(orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    blue_contours, blue_hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    
    oy=0
    by=0
    
    for pic, contour in enumerate(orange_contours): 
        area = cv2.contourArea(contour)
        if(area > 10): 
            x, oy, w, h = cv2.boundingRect(contour) 
            imageFrame = cv2.rectangle(imageFrame, (x, oy), (x + w, oy + h), (255, 0, 255), 2) 
    for pic, contour in enumerate(blue_contours): 
        area = cv2.contourArea(contour)
        if(area > 10): 
            x, by, w, h = cv2.boundingRect(contour) 
            imageFrame = cv2.rectangle(imageFrame, (x, by), (x + w, by + h), (255, 0, 0), 2) 
    print(by, oy, "print")
    if blue_y < by:
        blue_y = by
    if orange_y < oy:
        orange_y = oy
        
def gyro_correction():
    servo_angle = gyro * 1.4
    servo_angle = constrain(servo_angle, -45, 45)
    servo.angle(servo_angle)
    
def gyro_correction_1():
    servo_angle = (gyro+2) * 1.4
    servo_angle = constrain(servo_angle, -45, 45)
    servo.angle(servo_angle)
    
def distance_correction(right, servo, target_distance=36):
    """
    使用光達進行距離校正
    right: 右邊距離讀數
    servo: 伺服馬達物件
    target_distance: 目標距離 (預設0.35)
    """
    # 距離誤差
    error = right - target_distance
    
    # PID參數 (可根據實際情況調整)
    kp = 1.6  # 比例係數
    
    # 計算伺服馬達角度調整量
    servo_angle = error * kp
    
    # 限制角度範圍
    servo_angle = constrain(servo_angle, -25, 25)
    print("servo:", servo_angle)
    # 設定伺服馬達角度
    servo.angle(servo_angle)
    # 判斷是否到達目標位置 (容許誤差 ±0.05)
    if abs(error) <= 3 and right == -1:
        return True  # 距離校正完成
    else:
        return False  # 仍在校正中
    


read_file()

servo_left.angle(-70)
servo_right.angle(70)

pi.set_mode(17, pigpio.OUTPUT)
pi.set_mode(27, pigpio.OUTPUT)
pi.set_mode(9, pigpio.INPUT)
pi.set_pull_up_down(9, pigpio.PUD_UP)

if bno.begin() is not True:
    print("Error initializing device")
    exit()
time.sleep(1)
bno.setExternalCrystalUse(True)
threading.Thread(target=gyro_read).start()
threading.Thread(target=block_detect).start()
threading.Thread(target=read).start()
threading.Thread(target=color_read).start()
threading.Thread(target=line_detect).start()
threading.Thread(target=go_run).start()

pi.write(17, 1)
pi.write(27, 0)
# button = 1
# while button == 1:
#     button = pi.read(9)
#     time.sleep(0.01)
pi.write(17, 0)
pi.write(27, 1)
try:
    start = time.time()
    while time.time() - start < 1:
        frame = imcap.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        left_right_detect(frame)
    if blue_y > orange_y:
        print("Reverse")
    else:
        print("Forward")
    
    if left > right:
        norm_x_red = 0
        norm_x_green = 0
        print("go")
        Motor.DC(75)
        while abs(gyro) < 75:
            servo.angle(-70)
        Motor.DC(0)
        start = time.time()
        while time.time() - start < 1:
            frame = imcap.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            once_detect(frame)
        print("1")
        if is_block_in_center(norm_x_red, norm_x_green) == True:
            Motor.DC(80)
            while abs(gyro) < 170:
                servo.angle(-65)
            Motor.DC(50)
            start = time.time()
            while time.time() - start < 1:
                servo_angle = (gyro+170) * 1.2
                servo_angle = constrain(servo_angle, -50, 50)
                servo.angle(servo_angle)
        while abs(gyro) > 10:
            servo.angle(70)
            Motor.DC(80)
        Motor.DC(0)
    else:
        norm_x_red = 1
        norm_x_green = 1
        Motor.DC(70)
        while abs(gyro) < 80:
            servo.angle(75)
        Motor.DC(0)
        start = time.time()
        while time.time() - start < 1:
            frame = imcap.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            once_detect(frame)
        if is_block_in_center_right(norm_x_red, norm_x_green) == True:
            Motor.DC(75)
            while abs(gyro) < 170:
                servo.angle(65)
            Motor.DC(50)
            start = time.time()
            while time.time() - start < 0.5:
                servo_angle = (gyro-170) * 1.2
                servo_angle = constrain(servo_angle, -50, 50)
                servo.angle(servo_angle)
        while abs(gyro) > 10:
            servo.angle(-75)
            Motor.DC(80)
        Motor.DC(0)

    print("2")
    a=0
    dodge = 1
    time.sleep(1)
    Motor.DC(60)
    while round_count < 13:
        frame = imcap.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # 先轉回 BGR
        if not frame_queue.full():
            frame_queue.put(frame.copy())
        if round_count == 13:
            break
    dodge = 0
    Motor.DC(70)
    print("1")
    lum = 40
    while lum > 10:
        servo.angle(40)
    dir_array = [180, 180, 180, 180]
    Motor.DC(70)
    gyro = 10
    while abs(gyro) > 5:
        servo_angle = gyro * 1.2
        servo_angle = constrain(servo_angle, -45, 45)
        if servo_angle > 0:
            servo_angle*=-1
        servo.angle(servo_angle)
    Motor.DC(0)
    time.sleep(0.5)
    
    last=1
    py=0
    start = time.time()
    while run:
        Motor.DC(62)
        frame = imcap.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # 先轉回 BGR
        if py > 60 and px > 240 and time.time() - start > 0.5:
            break
        if not frame_queue.full():
            frame_queue.put(frame.copy())
    Motor.DC(0)
    last=0
    time.sleep(0.5)
    Motor.DC(62)
    if right < 25:
        start = time.time()
        while time.time() - start < 1.5:
            gyro_correction_1()
        while right >= 25:
            gyro_correction_1()
    else:
        while right >= 25:
            gyro_correction_1()
        start = time.time()
        while time.time() - start < 1.5:
            gyro_correction_1()
        while right >= 25:
            gyro_correction_1()

    start = time.time()
    while time.time() - start < 0.1:
        gyro_correction()
    Motor.DC(-10)
    time.sleep(0.5)
    Motor.DC(0)
    time.sleep(0.5)

    gyro = 0
    servo.angle(25)
    while abs(gyro) < 30:
        Motor.DC(55)
    Motor.DC(0)
    time.sleep(0.5) 
    servo.angle(80)
    while abs(gyro) < 55:
        Motor.DC(-75)
    servo.angle(0)
    Motor.DC(-10)
    time.sleep(0.5)
    Motor.DC(0)
    time.sleep(0.5)
    servo.angle(-80)
    time.sleep(1)
    while abs(gyro) > 10:
        print(gyro)
        Motor.DC(-80)
    servo.angle(0)
    Motor.DC(10)
    time.sleep(0.5)
    Motor.DC(0)
    time.sleep(0.5)
    
            
except KeyboardInterrupt:
    run = False
        
finally:
    Motor.DC(0)
    servo.angle(0)
    imcap.stop()
    cv2.destroyAllWindows()
    run=False

