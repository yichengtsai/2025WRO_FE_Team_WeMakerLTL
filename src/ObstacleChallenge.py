# === 導入所需函式庫 ===
# 匯入影像處理(cv2, numpy)、多執行緒(threading)、自訂功能(function)、佇列(Queue)、資料序列化(pickle)、樹莓派相機(Picamera2)等模組
import cv2
import numpy as np
import threading
from function import *
from queue import Queue
import pickle
from picamera2 import Picamera2

# 建立 pigpio 介面物件，用於控制 Raspberry Pi 的 GPIO
pi = pigpio.pi()

# PWM 驅動馬達時會在電源線上產生高頻脈衝，造成電壓波動與 EMI（電磁干擾）。
# 這段程式在感測器初始化前先將 PWM 腳位強制設為低電平，等感測器啟動完成後再恢復馬達控制。
pi.set_mode(11, pigpio.LOW)
pi.set_mode(21, pigpio.LOW)

# 初始化 Picamera2，設定低解析度(480x360)提升處理速度
# 固定曝光與亮度，提升顏色飽和與銳利度，確保影像辨識穩定
frame_queue = Queue(maxsize=2)
imcap = Picamera2()
config = imcap.create_preview_configuration(main={"size": (480, 360)})
imcap.configure(config)
imcap.start() 
imcap.set_controls({
    "AeExposureMode": 0,               # 關自動曝光
    "ExposureValue": 0.5,              # 稍微降低曝光
    "Brightness": 0.3,                 # 降低整體亮度
    "Saturation": 1,                   # 增加物體顏色強度
    "Sharpness": 0.6,                  # 輕微提升銳利度
    "AwbEnable": True,                 # 關自動白平衡
})

# === 從自訂 function 模組匯入並初始化感測器物件 ===
reader = D100Reader()  # 超音波測距模組
tcs34725 = TCS34725()  # 顏色感測器
bno = BNO055()         # 陀螺儀(IMU) 

# === 計數與狀態控制 ===
line_count = 0                # 記錄當前通過的線數(0~3)，用來對應 dir_array 切換車頭目標方向
total_line_count = 0          # 總共完成的線數
dir_array = [0, 90, 180, 270] # 車頭朝向的方向角度
avoidance_active = True       # 當前是否處於積木閃避階段，結束後停止該迴圈
program_active = True         # 控制整體程式運行，直到全部跑完才結束

# === 距離校正參數 ===
distance_corrected = False    # 是否已完成距離校正

# 是否啟用經陀螺儀校正後的光達數據（True 表示使用校正後數據）
enable_calibrated_lidar  = False

# PD 控制器中上一個誤差值，用於計算微分項（derivative）
last_error = 0

# === 偵測到的物體邊界框 (x, y, w, h) ===
rect_left_wall = (0, 0, 0, 0)     # 鏡頭偵測到的左牆資訊用來跟綠色斜線判斷
rect_right_wall = (0, 0, 0, 0)    # 鏡頭偵測到的右牆資訊用來跟紅色斜線判斷

# === 牆面資訊 ===
left_wall_y = left_wall_h = 0
right_wall_y = right_wall_h = 0

# === 車輛位置 (影像座標系) ===
car_x = 240
car_y = 260

def read_file():
     """
    從儲存的 pickle 檔案中讀取各顏色的 HSV 範圍設定，
    並轉換成 numpy 陣列以供影像處理使用。
    支援顏色：Green, Red, Pink, Orange, Blue, Black。
    """
    global green_lower, green_upper, red_lower, red_upper, black_lower, black_upper, pink_lower, pink_upper, orange_lower, orange_upper, blue_lower, blue_upper, bl_lower, bl_upper
    with open('save_file/HSV_Green.p', mode='rb') as f:
        file = pickle.load(f)
    lower_green = file['Lower']
    upper_green = file['Upper']
    print('Green_Lower:' + str(lower_green))
    print('Green_Upper:' + str(upper_green))
    
    with open('save_file/HSV_Red.p', mode='rb') as f:
        file = pickle.load(f)
    lower_red = file['Lower']
    upper_red = file['Upper']
    print('Red_Lower:' + str(lower_red))
    print('Red_Upper:' + str(upper_red))
    
    with open('save_file/HSV_Pink.p', mode='rb') as f:
        file = pickle.load(f)
    lower_pink = file['Lower']
    upper_pink = file['Upper']
    print('Pink_Lower:' + str(lower_pink))
    print('Pink_Upper:' + str(upper_pink))
    
    with open('save_file/HSV_Orange.p', mode='rb') as f:
        file = pickle.load(f)
    lower_orange = file['Lower']
    upper_orange = file['Upper']
    print('Orange_Lower:' + str(lower_orange))
    print('Orange_Upper:' + str(upper_orange))
    
    with open('save_file/HSV_Blue.p', mode='rb') as f:
        file = pickle.load(f)
    lower_blue = file['Lower']
    upper_blue = file['Upper']
    print('Blue_Lower:' + str(lower_blue))
    print('Blue_Upper:' + str(upper_blue))
    
    with open('save_file/HSV_Black.p', mode='rb') as f:
        file = pickle.load(f)
    lower_black = file['Lower']
    upper_black = file['Upper']
    print('Black_Lower:' + str(lower_black))
    print('Black_Upper:' + str(upper_black))
    
    red_lower = np.array(lower_red, np.uint8) 
    red_upper = np.array(upper_red, np.uint8)
    green_lower = np.array(lower_green, np.uint8)
    green_upper = np.array(upper_green, np.uint8)
    pink_lower = np.array(lower_pink, np.uint8)
    pink_upper = np.array(upper_pink, np.uint8)
    orange_lower = np.array(lower_orange, np.uint8)
    orange_upper = np.array(upper_orange, np.uint8)
    blue_lower = np.array(lower_blue, np.uint8)
    blue_upper = np.array(upper_blue, np.uint8)
    black_lower = np.array(lower_black, np.uint8)
    black_upper = np.array(upper_black, np.uint8)

def gyro_read():
    """
    以固定時間間隔讀取 BNO055 陀螺儀的相對角度，
    並依照 line_count 對應的方向做修正。
    持續讀取直到 program_active 為 False。
    """
    global gyro, line_count, program_active
    while program_active:
        gyro = bno.relative(dir_array[line_count])
        time.sleep(0.01)
        
def color_read():
    """
    以固定時間間隔讀取 TCS34725 光感值，
    持續讀取直到 program_active 為 False。
    """
    global lum, program_active
    tcs34725 = TCS34725()
    while program_active:
        lum = tcs34725.readluminance()['c']
        time.sleep(0.01)
        
def line_detect():
    """
    偵測並計數地場地地面線條：
    在轉彎處會有兩條線，需等兩條皆通過後再延遲 2.5 秒啟動後續偵測。
    """
    global line_count, total_line_count, avoidance_active , lum
    while avoidance_active :
        while lum > 30 and avoidance_active :
            time.sleep(0.001)
        if line_count + 1 == 4:
            line_count = 0
        else:
            line_count += 1
        total_line_count+=1
        while lum <= 30 and avoidance_active :
            time.sleep(0.001)  
        while lum > 30 and avoidance_active :
            time.sleep(0.001)
        time.sleep(2.5)
           
def read():
    """
    持續從光達讀取距離資料，根據是否啟用陀螺儀校正決定如何處理：
    - 若啟用，則根據校正後角度修正計算前、左、右方最短距離（單位公分）
    - 未啟用則直接使用光達讀取的原始距離
    """
    global gyro, front, left, right, program_active, front_1, left_1, right_1, enable_calibrated_lidar, right_special
    while program_active:
        ranges, angle_min, angle_increment, front_1, left_1, right_1, right_special = reader.get_latest_distances()
        if enable_calibrated_lidar  == True:
            front_min = 9999
            left_min = 9999
            right_min = 9999
            for i, dist in enumerate(ranges):
                if math.isnan(dist) or dist == 0:
                    continue
                angle = angle_min + i * angle_increment
                deg = math.degrees(angle)
                deg = (deg + 180) % 360 - 180
                corrected_angle = deg + gyro
                if -15 <= corrected_angle <= 15:
                    front_min = min(front_min, dist)
                elif -100 <= corrected_angle <= -80:
                    left_min = min(left_min, dist)
                elif 80 <= corrected_angle <= 100:
                    right_min = min(right_min, dist)

            # 距離轉為公分
            front = int(front_min*100) if front_min < 9999 else -1
            right = int(left_min*100) if left_min < 9999 else -1
            left = int(right_min*100) if right_min < 9999 else -1

# 函式(calculate_servo_angle_red、calculate_servo_angle_green)分別計算紅色與綠色積木的伺服角度，用以閃避相對牆壁
# 兩者邏輯相似，差異在於使用不同牆壁區域與變數
# 根據積木與車頭位置、牆壁判斷轉向角度，並結合陀螺儀校正進行動態調整
def calculate_servo_angle_red(cx, cy, bx, by):
    # 判斷從車頭點 (315, cy) 到積木點 (bx+75, by) 的線是否與右牆矩形相交
    intersects, clipped_pt1, clipped_pt2 = cv2.clipLine(rect_right_wall, (315, cy), (bx+75, by))

    dx_raw = bx - cx    # x方向原始差值，帶符號用於方向判斷
    dx = abs(dx_raw)    # x方向距離（絕對值）
    dy = abs(by - cy)   # y方向距離（絕對值）

    # 若dx為0，代表垂直線，無斜率，使用陀螺儀角度調整伺服角度，並限制範圍
    if dx == 0:
        servo_angle = gyro * 1.2
        servo_angle = constrain(servo_angle, -50, 50)
        servo.angle(servo_angle)
    # 積木在右側，且線未碰牆或牆位置較遠，依斜率映射角度，向右轉
    elif bx > cx and (not intersects or right_wall_y+right_wall_h < 240):
        m = dy/dx
        m = constrain(m, 0, 3)
        servo_angle = mapping(m, 0, 3, 50, 10) 
        servo.angle(servo_angle)
    # 線與牆相交且陀螺儀角度大於5度，調整斜率以避開牆壁
    elif intersects and abs(gyro) > 5:
        # 取得線與牆交點最大x值，計算與牆的x方向差距
        x_man_overlap = max(clipped_pt1[0], clipped_pt2[0])
        rect_right_x = rect_right_wall[0]
        diff = x_man_overlap - rect_right_x
        # 根據dx_raw方向調整斜率的x分量，避免撞牆
        if dx_raw > 0:
            dx_adjusted = dx_raw - diff - 22  # 第一象限處理
        else:
            dx_adjusted = dx_raw + diff - 22  # 第二象限處理
        # 避免dx_adjusted為0造成除錯誤
        if abs(dx_adjusted) < 1e-5:
            dx_adjusted = 1e-5 if dx_adjusted >= 0 else -1e-5
        m = dy / dx_adjusted
        m_abs = abs(m)
        m_abs = constrain(m_abs, 0, 3)
        # 根據修正後的dx_adjusted決定轉向方向和角度
        if dx_adjusted > 0:
            servo_angle = mapping(m_abs, 0, 3, 50, 0)
            servo.angle(servo_angle)  # 右轉
        else:
            servo_angle = mapping(m_abs, 0, 3, 30, 15)
            servo.angle(-servo_angle) # 左轉
    # 若積木與車頭距離過近，角度歸零，不轉向
    elif abs(bx - cx) < 75 or abs(by - cy) < 60:
        servo.angle(0)    
    # 其他情況，依陀螺儀角度或斜率進行轉向調整
    elif abs(gyro) > 50:
        m = dy/dx
        m = constrain(m, 0, 3)
        servo_angle = mapping(m, 0, 3, 30, 10)
        servo.angle(-servo_angle)
    else:
        servo_angle = gyro * 1.2
        servo_angle = constrain(servo_angle, -50, 50)
        servo.angle(servo_angle)

def calculate_servo_angle_green(cx, cy, bx, by):
    intersects, clipped_pt1, clipped_pt2 = cv2.clipLine(rect_left_wall, (175, cy), (bx-65, by))
    dx = abs(bx - cx)
    dy = abs(by - cy)
    if dx == 0:
        servo_angle = gyro * 1.2
        servo_angle = constrain(servo_angle, -50, 50)
        servo.angle(servo_angle)
    elif bx < cx and (not intersects or left_wall_y+left_wall_h < 240):
        m = dy/dx
        m = constrain(m, 0, 3)
        servo_angle = mapping(m, 0, 3, 50, 10)
        servo.angle(-servo_angle)
        print("dodge:", -servo_angle)
    elif intersects and abs(gyro) > 5:
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

def distance_correction(right_1, servo, target_distance=34):
    """
    利用 PD 控制器調整伺服角度，使車身與右牆保持固定距離（預設34公分）。
    - right_1*100：實際右側距離（公分）
    - error：實際距離與目標距離的誤差
    - kp, kd：比例與微分控制係數
    - servo_angle：計算後伺服角度，限制在[-25, 25]度範圍內
    回傳值為誤差是否小於等於2公分，代表是否達到校正目標。
    """
    global last_error
    distance = right_1*100
    error = distance - target_distance
    kp = 2.0
    kd = 0.5
    derivative = error - last_error
    last_error = error
    servo_angle = kp * error + kd * derivative
    servo_angle = constrain(servo_angle, -25, 25)
    servo.angle(servo_angle)
    return abs(error) <= 2

read_file()  # 讀取HSV色彩範圍設定檔，初始化顏色辨識參數

if bno.begin() is not True:  # 初始化BNO055陀螺儀，若失敗則顯示錯誤並結束程式
    print("Error initializing device")
    exit()

time.sleep(1)  # 稍作延遲等待硬體穩定
bno.setExternalCrystalUse(True)  # 啟用外部晶振，提高陀螺儀精度
time.sleep(1)

# 啟動多執行緒以非同步方式持續讀取
threading.Thread(target=gyro_read).start()    # 陀螺儀讀取執行緒
threading.Thread(target=color_read).start()   # 光感讀取執行緒
threading.Thread(target=line_detect).start()  # 線條偵測執行緒

# === 從自訂 function 模組匯入並初始化馬達控制物件 ===
servo = servo_motor()  # 伺服馬達物件
Motor = dc_motor()     # 直流馬達物件

# === 車輛從停車格起步 === 
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

# === 開始執行閃避積木並繞圈 === 
while total_line_count < 13:
    frame = imcap.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    imageFrame = frame
    imageFrame[250:360, 0:480] = [0, 0, 0]
    imageFrame[0:95, 0:480] = [0, 0, 0]
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
            if x + w/2 < 240: 
                left_contours.append((area, x, y, w, h))
            else:
                right_contours.append((area, x, y, w, h))
    if left_contours:
        largest = max(left_contours, key=lambda x: x[0])
        _, x, y, w, h = largest
        if x < max_x_green:
            max_x_green = x + w + abs(gyro)*0.1
            w = int(w + abs(gyro)*0.2)
            rect_left_wall = (x, y, w, h)
            left_wall_y = y
            left_wall_h = h
        cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (255, 255, 0), 2)
    if right_contours:
        largest = max(right_contours, key=lambda x: x[0])
        _, x, y, w, h = largest
        if x > max_x_red:
            max_x_red = int(x - abs(gyro)*0.25)
            rect_right_wall = (max_x_red, y, w, h)
            right_wall_y = y
            right_wall_h = h
        cv2.rectangle(imageFrame, (max_x_red, y), (max_x_red + w, y + h), (255, 255, 0), 2)
    
    for cnt in red_contours:
        area = cv2.contourArea(cnt)
        if area > 50:
            x, y, w, h = cv2.boundingRect(cnt)
            tx = x + w + 100
            ty = y
            cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            y_bottom = y + h 
            if y_bottom > max_y_bottom_red+30:
                max_y_bottom_red = y_bottom
                closest_tx_red = x + w + 100 + int(abs(gyro)*0.15)
                closest_ty_red = y
            if closest_tx_red > 100 :
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
            y_bottom = y + h  
            if x > 405 and abs(gyro) < 5:
                print("no")
            elif y_bottom > max_y_bottom_green+30:
                max_y_bottom_green = y_bottom
                closest_tx_green = x - 105 - int(abs(gyro)*0.15)
                closest_ty_green = y

            if closest_tx_green < 380 :
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

# === 開始執行停車 ===
threading.Thread(target=read).start()
avoidance_active = False
Motor.DC(70)
while lum > 17:
    servo_angle = (gyro+45) * 1.4
    servo_angle = constrain(servo_angle, -45, 45)
    servo.angle(servo_angle)
Motor.DC(45)
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
enable_calibrated_lidar = True
Motor.DC(33)
while True:
    if (35 - right) > 2 and right != -1 and right_1 > 0.2:
        distance_corrected = distance_correction(right, servo)
    else:
        servo_angle = (gyro+4) * 1.3
        servo_angle = constrain(servo_angle, -50, 50)
        servo.angle(servo_angle)
    if right_special < 0.2:
        break
enable_calibrated_lidar = False
Motor.DC(35)
while right_special < 0.2:
    servo_angle = gyro * 1.4
    servo_angle = constrain(servo_angle, -50, 50)
    servo.angle(servo_angle)
Motor.DC(0)
time.sleep(0.5)
x1, y1 = 360, 0
x2, y2 = 315, 360
m = (y2 - y1) / (x2 - x1)
b = y1 - m * x1
Motor.DC(30)
while right_special >= 0.2:
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
servo.angle(0)
Motor.DC(10)
time.sleep(0.5)
Motor.DC(0)
Motor.DC(30)
while front_1 > 0.12:
    servo_angle = gyro * 1.4
    servo_angle = constrain(servo_angle, -45, 45)
    servo.angle(servo_angle)

# 停止伺服馬達與直流馬達，停止程式運行
servo.angle(0)         # 伺服馬達歸零角度停止轉動
Motor.DC(0)            # 直流馬達停止運轉
program_active = False # 結束主迴圈與各執行緒
imcap.stop()           # 停止攝影機影像擷取
cv2.destroyAllWindows()# 關閉所有OpenCV視窗

